#pragma once

#include "ReferenceFrameZ.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace SVT::RigidBodyZ
{
using Id = std::uint64_t;
using Vector = std::array<float, 4>;
using MotionSnapshot =
    std::array<std::uint8_t, 160>; // hkMotionState prefix: transform and swept pose, before objectRadius.

inline bool Same(const Vector& a, const Vector& b) noexcept
{
	return std::memcmp(a.data(), b.data(), sizeof(a)) == 0;
}

inline bool FiniteXYZ(const Vector& a) noexcept
{
	return std::all_of(a.begin(), a.begin() + 3, [](float f) { return std::isfinite(f); });
}

struct Binding
{
	// Borrowed engine identities; the Actor handle and binding epoch prevent reuse across instances.
	std::uintptr_t actor{}, controller{}, character{}, body{}, world{};
	std::uint32_t handle{};
	bool operator==(const Binding&) const = default;

	explicit operator bool() const noexcept
	{
		return actor && controller && character && body && world && handle;
	}
};

struct Origin
{
	Id command{}, episode{};
	bool operator==(const Origin&) const = default;

	explicit operator bool() const noexcept
	{
		return command && episode;
	}
};

struct Interval
{
	float begin{}, end{}, dt{}, inverseDt{};
	bool operator==(const Interval&) const = default;

	explicit operator bool() const noexcept
	{
		return std::isfinite(begin) && std::isfinite(end) && std::isfinite(dt) && std::isfinite(inverseDt) &&
		       end > begin && dt > 0 && inverseDt > 0;
	}

	bool Contains(float t) const noexcept
	{
		return *this && std::isfinite(t) && t >= begin && t <= end;
	}
};

struct Source : SVT::ReferenceFrameZ::Phase
{
	std::uintptr_t actor{};
	std::uint32_t handle{};
	float authoredZ{};

	void Bind(Id next, std::uintptr_t owner, std::uint32_t generation) noexcept
	{
		if (actor != owner || handle != generation) {
			*this = {};
		}
		Begin(next);
		actor = owner;
		handle = generation;
	}

	void Movement() noexcept
	{
		if (!active) {
			episode = 0;
			airRequested = false;
		}
	}

	bool BelongsTo(const Binding& binding) const noexcept
	{
		return actor == binding.actor && handle == binding.handle;
	}
};

struct Input
{
	Id id{}, operation{}, version{};
	std::uintptr_t accumulator{}, entities{};
	std::int32_t index{};
	Interval time{};
	Origin origin{};
};

struct Operation
{
	// One native world step can contain several substeps and worker callbacks.
	Id id{}, epoch{};
	std::uintptr_t world{};
	Origin origin{};
	Interval time{};
	bool input{}, mixed{}, output{};
};

struct Pose
{
	Id id{}, operation{};
	MotionSnapshot motion{};
	Interval time{};
	Origin origin{};
};

struct Publication
{
	// A matching physical result; Actor writeback must still confirm the final position.
	Id pose{};
	Origin origin{};
	float sample{};

	explicit operator bool() const noexcept
	{
		return pose && static_cast<bool>(origin);
	}
};

// A velocity submission gains publication rights only after a matching native step consumes it.
class Tracker
{
  public:
	Binding binding{};
	Id epoch{};
	Source source{};
	bool fault{};

	void Bind(const Binding& newBinding)
	{
		if (binding == newBinding) {
			return;
		}
		binding = newBinding;
		++epoch;
		source = {};
		current = {};
		expected = {};
		hasExpected = false;
		inputCount = poseCount = 0;
		lastCommitted = -INFINITY;
		lastCommittedPose = 0;
		fault = false;
	}

	void Release() noexcept
	{
		current = {};
		++version;
	}

	void Invalidate() noexcept
	{
		Release();
		inputCount = poseCount = 0;
		fault = true;
		source.End();
	}

	void SetVelocity(const Vector& before, const Vector& after, Origin requested)
	{
		if (!FiniteXYZ(before) || !FiniteXYZ(after)) {
			Invalidate();
			return;
		}
		if (!requested) {
			current = {};
		} else if (!Same(before, after)) {
			current = requested;
		}
		// An unchanged velocity can inherit only the previously verified submission.
		else if (!hasExpected || !Same(before, expected)) {
			current = {};
		}
		expected = after;
		hasExpected = true;
		++version;
	}

	void NativePolicy(Operation& operation, const Vector& before, const Vector& after)
	{
		if (operation.epoch != epoch) {
			return;
		}
		if (!operation.output || operation.mixed || !operation.origin || current != operation.origin ||
		    !hasExpected || !Same(before, expected) || !FiniteXYZ(after)) {
			Release();
			operation.mixed = true;
			return;
		}
		expected = after;
		hasExpected = true;
		++version;
	}

	bool ControllerPolicy(const Vector& before, const Vector& after)
	{
		if (!current || !hasExpected || !Same(before, expected) || !FiniteXYZ(after)) {
			Release();
			return false;
		}
		expected = after;
		++version;
		return true;
	}

	Input Consume(Operation& operation, std::uintptr_t accumulator, std::uintptr_t entities,
	              std::int32_t index, Interval time, const Vector& velocity)
	{
		if (fault || operation.epoch != epoch || !time || !FiniteXYZ(velocity)) {
			return {};
		}
		Origin from = hasExpected && Same(velocity, expected) ? current : Origin{};
		if (!from) {
			current = {};
		}
		if (inputCount == inputs.size()) {
			Invalidate();
			return {};
		}
		Input input{++serial, operation.id, version, accumulator, entities, index, time, from};
		inputs[inputCount++] = input;
		if (!operation.input) {
			operation.origin = from;
			operation.time = time;
			operation.input = true;
		} else {
			if (operation.origin != from) {
				operation.mixed = true;
			}
			operation.time.begin = std::min(operation.time.begin, time.begin);
			operation.time.end = std::max(operation.time.end, time.end);
		}
		return input;
	}

	void Damped(const Input& input, const Vector& after)
	{
		if (!input.id || fault) {
			return;
		}
		if (version != input.version || !FiniteXYZ(after)) {
			for (std::size_t n = 0; n < inputCount; ++n) {
				if (inputs[n].id == input.id) {
					inputs[n].origin = {};
				}
			}
			Release();
			return;
		}
		current = input.origin;
		expected = after;
		hasExpected = true;
		++version;
	}

	Input Find(std::uintptr_t accumulator, std::uintptr_t entities, std::int32_t index, Interval time,
	           Id operation) const noexcept
	{
		Input result{};
		for (std::size_t n = 0; n < inputCount; ++n) {
			const auto& i = inputs[n];
			if (i.accumulator == accumulator && i.entities == entities && i.index == index &&
			    i.time == time && i.operation == operation) {
				if (result.id) {
					return {}; // Multiple matches cannot prove which input was consumed.
				}
				result = i;
			}
		}
		return result;
	}

	void Export(Operation& operation, const Input& input, const MotionSnapshot& motion, const Vector& after)
	{
		if (operation.epoch != epoch) {
			return;
		}
		if (!input.id || fault || !FiniteXYZ(after)) {
			Release();
			poseCount = 0;
			operation.mixed = true;
			return;
		}
		RemoveInputs([&](const Input& i) { return i.id == input.id; });
		current = input.origin;
		expected = after;
		hasExpected = true;
		++version;
		if (operation.origin != input.origin) {
			operation.mixed = true;
		}
		operation.output = true;
		Save(operation, input.origin, input.time, motion);
	}

	Publication Publish(Operation* operation, const MotionSnapshot& motion, float currentTime, float psiTime,
	                    float marker, bool swept, float sampleTime)
	{
		if (fault || !std::isfinite(currentTime) || !std::isfinite(psiTime) || !std::isfinite(marker) ||
		    currentTime < marker) {
			return {};
		}
		const float sample = swept ? sampleTime : psiTime;
		if (sample < lastCommitted || (swept && sample != currentTime)) {
			return {};
		}
		// Match actual pose first, including previous PSI and non-root outputs.
		for (std::size_t n = poseCount; n > 0; --n) {
			const auto& pose = poses[n - 1];
			if (pose.motion == motion) {
				if (!pose.time.Contains(sample) || (!swept && sample != pose.time.end)) {
					return {};
				}
				Publication p{pose.id, pose.origin, sample};
				return CanCommit(p) ? p : Publication{};
			}
		}
		// postSim can run after export but before the enclosing step returns;
		// publication still requires this operation's unmixed input and completed output.
		if (operation && operation->epoch == epoch && operation->input && operation->output &&
		    !operation->mixed && operation->origin && operation->world == binding.world &&
		    operation->time.Contains(sample) && psiTime == operation->time.end) {
			Save(*operation, operation->origin, operation->time, motion);
			return {poses[poseCount - 1].id, operation->origin, sample};
		}
		return {};
	}

	bool CanCommit(const Publication& p) const noexcept
	{
		if (fault || !p || p.sample < lastCommitted ||
		    (p.sample == lastCommitted && p.pose < lastCommittedPose)) {
			return false;
		}
		return std::any_of(poses.begin(), poses.begin() + poseCount, [&](const Pose& pose) {
			return pose.id == p.pose && pose.origin == p.origin && pose.time.Contains(p.sample);
		});
	}

	void Committed(const Publication& p) noexcept
	{
		if (CanCommit(p)) {
			lastCommitted = p.sample;
			lastCommittedPose = p.pose;
		}
	}

	void EndOperation(Id operation, bool success)
	{
		RemoveInputs([&](const Input& i) { return i.operation == operation; });
		if (!success) {
			const auto end = std::remove_if(poses.begin(), poses.begin() + poseCount,
			                                [&](const Pose& p) { return p.operation == operation; });
			poseCount = static_cast<std::size_t>(end - poses.begin());
			Release();
		}
	}

	bool Idle() const noexcept
	{
		return !source.active && !current && !inputCount &&
		       std::none_of(poses.begin(), poses.begin() + poseCount,
		                    [](const Pose& p) { return static_cast<bool>(p.origin); });
	}

  private:
	static constexpr std::size_t kMaxPendingInputs = 32;
	static constexpr std::size_t kPoseHistory = 8;

	void Save(const Operation& operation, Origin origin, Interval time, const MotionSnapshot& motion)
	{
		if (poseCount == poses.size()) {
			std::move(poses.begin() + 1, poses.end(), poses.begin());
			--poseCount;
		}
		poses[poseCount++] = {++serial, operation.id, motion, time, origin};
	}

	template <class Predicate> void RemoveInputs(Predicate remove) noexcept
	{
		const auto end = std::remove_if(inputs.begin(), inputs.begin() + inputCount, remove);
		inputCount = static_cast<std::size_t>(end - inputs.begin());
	}

	Id serial{}, version{};
	Origin current{};
	Vector expected{};
	bool hasExpected{};
	float lastCommitted{-INFINITY};
	Id lastCommittedPose{};
	// Input exhaustion revokes ownership; pose history evicts the oldest completed result.
	std::array<Input, kMaxPendingInputs> inputs{};
	std::array<Pose, kPoseHistory> poses{};
	std::size_t inputCount{}, poseCount{};
};
} // namespace SVT::RigidBodyZ
