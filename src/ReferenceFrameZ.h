#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace SVT::ReferenceFrameZ
{
using Id = std::uint64_t;
using Vector = std::array<float, 3>;

// Load messages change this generation; an odd value suspends new source admission.
// Existing native calls still unwind their own leases and physical operations.
inline std::atomic<Id> sourceLifetime{2};

inline Id CurrentLifetime() noexcept
{
	const auto value = sourceLifetime.load(std::memory_order_acquire);
	return value & 1 ? 0 : value;
}

inline bool IsCurrentLifetime(Id value) noexcept
{
	return value && value == CurrentLifetime();
}

inline bool FiniteXYZ(const Vector& value) noexcept
{
	return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

// The native synchronized consumer owns this borrowed scope, including its Move.
// No clip, cache publication or TLS candidate survives the consumer's return.
struct Consumer
{
	std::uintptr_t actor{}, process{};
	std::uint32_t handle{};
	Id serial{};
	bool eligible{}, synchronized{}, dispatched{};

	void Read(std::uintptr_t fromProcess, bool validPosition) noexcept
	{
		synchronized = eligible && process && process == fromProcess && validPosition;
	}

	bool CanDispatch(std::uintptr_t targetActor, float seconds, const Vector& delta) const noexcept
	{
		return eligible && synchronized && !dispatched && actor && actor == targetActor && handle && serial &&
		       std::isfinite(seconds) && seconds > 0.0f && FiniteXYZ(delta);
	}
};

// Convert allocated game-unit displacement to Havok units/second; signed zero remains a valid request.
inline bool Velocity(float allocatedDz, float seconds, float worldToHavok, float& output) noexcept
{
	if (!std::isfinite(allocatedDz) || !std::isfinite(seconds) || seconds <= 0.0f ||
	    !std::isfinite(worldToHavok) || worldToHavok <= 0.0f) {
		return false;
	}
	const auto result = allocatedDz * worldToHavok / seconds;
	if (!std::isfinite(result)) {
		return false;
	}
	output = result;
	return true;
}

// One native root-motion phase per Actor/controller binding. A finite zero,
// direction reversal or another native consumer does not start a new phase.
struct Phase
{
	static constexpr std::uint32_t kOnGround = 0;
	static constexpr std::uint32_t kJumping = 1;
	static constexpr std::uint32_t kInAir = 2;
	static constexpr std::uint32_t kNoRequestedState = 0xB;
	// Only lift-off uses this threshold; signed Z ownership accepts every finite nonzero request.
	static constexpr float kLiftOffIntentThreshold = 1.0e-4f;
	Id episode{};
	bool active{}, ownsZ{}, airRequested{}, observedAir{};

	void Begin(Id nextEpisode) noexcept
	{
		if (!active) {
			episode = nextEpisode;
			ownsZ = airRequested = observedAir = false;
		}
		active = true;
	}

	void End() noexcept
	{
		active = ownsZ = airRequested = observedAir = false;
	}

	// Leading zeros keep native vertical policy; a claimed phase retains internal zeros.
	bool ClaimZ(float authoredDz) noexcept
	{
		if (!active || !std::isfinite(authoredDz)) {
			return false;
		}
		ownsZ = ownsZ || authoredDz != 0.0f;
		return ownsZ;
	}

	bool RequestAir(float authoredDz, std::uint32_t state, std::uint32_t want) noexcept
	{
		if (state == kJumping || state == kInAir) {
			observedAir = true;
		} else if (state == kOnGround && observedAir) {
			airRequested = observedAir = false;
		}
		if (!active || airRequested || !std::isfinite(authoredDz) || authoredDz <= kLiftOffIntentThreshold ||
		    state != kOnGround || (want != kOnGround && want != kNoRequestedState)) {
			return false;
		}
		airRequested = true;
		return true;
	}
};
} // namespace SVT::ReferenceFrameZ
