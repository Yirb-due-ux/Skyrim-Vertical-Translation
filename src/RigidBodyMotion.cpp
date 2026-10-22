#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "RE/B/bhkWorld.h"
#include "RE/H/hkpWorld.h"
#include "Controller.h"
#include "RigidBodyMotion.h"
#include "Hooks.h"
#include <atomic>
#include <mutex>
#include <new>
#include <unordered_map>

namespace SVT::RigidBodyZ::Native
{
static_assert(sizeof(MotionSnapshot) == offsetof(RE::hkMotionState, objectRadius));

struct Instance
{
	std::uintptr_t actor{};
	std::uint32_t handle{};
	bool retired{};
	Id lifetime{};
	Id sourceSerial{}; // Consumer ordering only, independent of binding epoch and physical time.
	Tracker tracker;
	Operation* operation{}; // owned by the native world step, including join-waited workers
};

namespace
{
// Indexes and Tracker state share this lock; native callbacks run after releasing it.
// Lock failure is fatal, while allocation failure revokes only the affected ownership.
std::mutex lock;
// Bound borrowed body traversal; the original native call still processes the full batch.
constexpr std::int32_t kMaxBodyBatch = 65536;
using Index = std::unordered_map<std::uintptr_t, Owner>;
Index actors, controllers, characters, bodies;
std::atomic<std::size_t> boundCount{};
std::atomic<bool> enabled{};
float worldToHavok{}, havokToWorld{};
ControllerVelocity controllerVelocityOriginal{};
MotionVelocityFn motionOriginal{};
Id sequence{};
// These borrowed contexts follow the native call stack; matching scopes restore them on every exit.
thread_local Owner movementOwner{};
thread_local Id movementSerial{}, movementEpoch{}, controllerUpdateEpoch{};
thread_local const void* movementInput{};
thread_local bool controllerUpdateEligible{}, stateSubmitted{};

struct Submission
{
	Owner owner{};
	Origin origin{};
	const void* velocity{};
	float dt{};
	bool policy{};
};

thread_local Submission submission{};
thread_local Instance* velocityOwner{};
thread_local PositionReadState* positionRead{};
thread_local MoveFinishState* moveFinish{};
using SVT::Controller::Read;

template <class T> bool Read(std::uintptr_t p, T& out) noexcept
{
	SVT::Hooks::PreserveLastError preserve;
	return SVT::Hooks::Read(p, &out, sizeof(out));
}

template <class T> T ReadOrDefault(std::uintptr_t p) noexcept
{
	SVT::Hooks::PreserveLastError preserve;
	return SVT::Hooks::ReadOrDefault<T>(p);
}

template <class T> T ReadOrDefault(const void* p) noexcept
{
	T value{};
	Read(p, value);
	return value;
}

Owner Find(const Index& index, std::uintptr_t key) noexcept
{
	const auto found = index.find(key);
	return found == index.end() ? Owner{} : found->second;
}

void Erase(Index& index, std::uintptr_t key, const Owner& owner) noexcept
{
	if (Find(index, key) == owner) {
		index.erase(key);
	}
}

void Unindex(const Owner& owner) noexcept
{
	const auto& binding = owner->tracker.binding;
	Erase(controllers, binding.controller, owner);
	Erase(characters, binding.character, owner);
	Erase(bodies, binding.body, owner);
	boundCount.store(bodies.size(), std::memory_order_release);
}

void Retire(const Owner& owner) noexcept
{
	owner->retired = true;
	owner->tracker.Invalidate();
	Unindex(owner);
	Erase(actors, owner->actor, owner);
}

Owner Actor(std::uintptr_t actor, std::uint32_t handle, bool create, Id lifetime)
{
	auto owner = Find(actors, actor);
	if (owner && (owner->handle != handle || (create && owner->tracker.fault && !owner->operation))) {
		if (!create) {
			return {}; // A stale source end cannot retire the live Actor generation.
		}
		// Retry a fault only after its world operation ends; old scopes stay retired.
		Retire(owner);
		owner.reset();
	}
	if (!owner && create) {
		owner = std::make_shared<Instance>();
		owner->actor = actor;
		owner->handle = handle;
		owner->lifetime = lifetime;
		actors.emplace(actor, owner);
	}
	return owner;
}

bool Matches(const Owner& owner) noexcept
{
	if (!owner || owner->retired || !ReferenceFrameZ::IsCurrentLifetime(owner->lifetime) ||
	    !owner->tracker.binding) {
		return false;
	}
	const auto& binding = owner->tracker.binding;
	const auto* controller = reinterpret_cast<const RE::bhkCharRigidBodyController*>(binding.controller);
	const auto* character = reinterpret_cast<const RE::hkpCharacterRigidBody*>(binding.character);
	const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(binding.body);
	return binding.actor == owner->actor && binding.handle == owner->handle &&
	       SVT::Controller::HasType(reinterpret_cast<std::uintptr_t>(controller),
	                                SVT::Controller::types.rigid) &&
	       SVT::Controller::HasType(reinterpret_cast<std::uintptr_t>(
	                                    static_cast<const RE::hkpCharacterRigidBodyListener*>(controller)),
	                                SVT::Controller::types.rigidListener) &&
	       ReadOrDefault<std::uintptr_t>(&controller->charRigidBody.referencedObject) == binding.character &&
	       ReadOrDefault<std::uintptr_t>(&character->character) == binding.body &&
	       ReadOrDefault<std::uintptr_t>(&body->world) == binding.world &&
	       SVT::Controller::HasType(reinterpret_cast<std::uintptr_t>(&body->motion),
	                                SVT::Controller::types.characterMotion);
}

bool Live(const Owner& owner) noexcept
{
	return enabled.load(std::memory_order_acquire) && owner && !owner->tracker.fault && Matches(owner);
}

bool Supported(std::uintptr_t controller) noexcept
{
	const auto state =
	    SVT::Controller::State(reinterpret_cast<const RE::bhkCharacterController*>(controller));
	return state && state->AcceptsVelocity();
}

void ReleaseVelocityOwnership(const Owner& owner) noexcept
{
	// Failed observation stops future attribution without discarding already completed poses.
	owner->tracker.Release();
	if (auto* operation = owner->operation; operation && operation->epoch == owner->tracker.epoch) {
		operation->mixed = true;
	}
}

// Allocation failure revokes proof, never rolls back a native velocity/pose.
void AllocationFailure(const Owner& owner) noexcept
{
	if (!owner) {
		return;
	}
	owner->tracker.Invalidate();
}

void WorldAllocationFailure(std::uintptr_t world) noexcept
{
	for (const auto& [actor, owner] : actors) {
		(void)actor;
		if (Live(owner) && owner->tracker.binding.world == world) {
			AllocationFailure(owner);
		}
	}
}

void Sweep() noexcept
{
	for (auto it = actors.begin(); it != actors.end();) {
		auto owner = it->second;
		++it; // Retire erases only this Actor; no temporary allocation in cleanup.
		if (!owner->operation && ((!owner->tracker.fault && owner->tracker.Idle()) ||
		                          (owner->tracker.binding && !Matches(owner)))) {
			Retire(owner);
		}
	}
}
} // namespace

void Configure(float worldScale) noexcept
{
	std::lock_guard guard(lock);
	for (const auto& [address, owner] : actors) {
		(void)address;
		owner->retired = true;
	}
	actors.clear();
	controllers.clear();
	characters.clear();
	bodies.clear();
	boundCount.store(0);
	worldToHavok = worldScale;
	havokToWorld = 1.0f / worldToHavok;
}

void Enable() noexcept
{
	enabled.store(true, std::memory_order_release);
}

void Disable() noexcept
{
	enabled.store(false, std::memory_order_release);
}

void RetirePreviousLifetimes() noexcept
{
	std::lock_guard guard(lock);
	for (auto it = actors.begin(); it != actors.end();) {
		const auto owner = it++->second;
		if (!ReferenceFrameZ::IsCurrentLifetime(owner->lifetime)) {
			Retire(owner);
		}
	}
}

void RetireActor(std::uintptr_t actor) noexcept
{
	std::lock_guard guard(lock);
	if (const auto owner = Find(actors, actor)) {
		Retire(owner);
	}
}

void RetireController(std::uintptr_t controller) noexcept
{
	std::lock_guard guard(lock);
	if (const auto owner = Find(controllers, controller)) {
		Retire(owner);
	}
}

void SetControllerVelocityOriginal(ControllerVelocity fn) noexcept
{
	controllerVelocityOriginal = fn;
}

void SetMotionVelocityOriginal(MotionVelocityFn fn) noexcept
{
	motionOriginal = fn;
}

void SetSource(std::uintptr_t actor, std::uint32_t form, std::uint32_t handle, bool active, float authoredDz,
               Id lifetime, const std::atomic<bool>* cancelled, Id sourceSerial) noexcept
{
	if (!actor || (active && !form) || !enabled.load()) {
		return;
	}
	active = active && std::isfinite(authoredDz);
	std::lock_guard guard(lock);
	if (!ReferenceFrameZ::IsCurrentLifetime(lifetime) || (cancelled && cancelled->load())) {
		return;
	}
	const auto current = Find(actors, actor);
	if (!active && current && sourceSerial && current->sourceSerial > sourceSerial) {
		return;
	}
	if (!active) {
		const auto owner = Find(actors, actor);
		if (owner && (!handle || owner->handle == handle)) {
			owner->tracker.source.End(); // Already consumed physics keeps its publication ownership.
		}
		return;
	}
	if (!handle) {
		return;
	}
	Owner owner;
	try {
		owner = Actor(actor, handle, active, lifetime);
	} catch (const std::bad_alloc&) {
		// Creation has not published an instance. A recycled old handle, if any,
		// was retired; a later native consumer may retry without a partial map entry.
		return;
	}
	if (!owner || owner->tracker.fault) {
		return;
	}
	owner->sourceSerial = std::max(owner->sourceSerial, sourceSerial);
	auto& source = owner->tracker.source;
	source.Bind(++sequence, actor, handle);
	source.authoredZ = authoredDz;
	source.ClaimZ(authoredDz);
}

Movement::Movement(std::uintptr_t actor, std::uint32_t form, std::uint32_t handle,
                   std::uintptr_t controller) noexcept
{
	// Every movement shadows the outer scope, including Player/foreign recursion.
	previousOwner = movementOwner;
	previous = movementSerial;
	previousEpoch = movementEpoch;
	movementOwner.reset();
	movementSerial = movementEpoch = 0;
	if (!actor || !form || !handle || !enabled.load()) {
		return;
	}
	std::lock_guard guard(lock);
	auto owner = Find(actors, actor);
	if (owner && owner->handle != handle) {
		Retire(owner);
		return;
	}
	if (!owner) {
		return; // Only a qualified native synchronized consumer creates an instance.
	}
	auto& tracker = owner->tracker;
	Binding binding{actor, controller, 0, 0, 0, handle};
	const auto* object = reinterpret_cast<const RE::bhkCharRigidBodyController*>(controller);
	if (SVT::Controller::HasType(controller, SVT::Controller::types.rigid) &&
	    SVT::Controller::HasType(
	        reinterpret_cast<std::uintptr_t>(static_cast<const RE::hkpCharacterRigidBodyListener*>(object)),
	        SVT::Controller::types.rigidListener)) {
		binding.character = ReadOrDefault<std::uintptr_t>(&object->charRigidBody.referencedObject);
		const auto* character = reinterpret_cast<const RE::hkpCharacterRigidBody*>(binding.character);
		binding.body = character ? ReadOrDefault<std::uintptr_t>(&character->character) : 0;
		const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(binding.body);
		binding.world = body ? ReadOrDefault<std::uintptr_t>(&body->world) : 0;
	}
	if (tracker.binding != binding) {
		auto source = tracker.source;
		if (tracker.binding) {
			source.episode = ++sequence;
			source.airRequested = false;
			if (tracker.binding.controller != binding.controller) {
				source.ownsZ = false;
				source.ClaimZ(source.authoredZ);
			}
		}
		Unindex(owner);
		// A recycled concrete controller/body cannot remain owned by two Actors.
		for (const auto& candidate : {Find(controllers, binding.controller),
		                              Find(characters, binding.character), Find(bodies, binding.body)}) {
			if (candidate && candidate != owner) {
				Retire(candidate);
			}
		}
		tracker.Bind(binding);
		tracker.source = source;
		owner->operation = nullptr;
		try {
			if (Live(owner)) {
				controllers[binding.controller] = owner;
				characters[binding.character] = owner;
				bodies[binding.body] = owner;
			}
		} catch (const std::bad_alloc&) {
			// Remove every successfully inserted index of this binding. Old
			// proof cannot be restored after a binding change; this owner stops.
			Unindex(owner);
			AllocationFailure(owner);
			return;
		}
		boundCount.store(bodies.size(), std::memory_order_release);
	}
	if (!Live(owner)) {
		return;
	}
	tracker.source.Movement();
	if (!tracker.source.active) {
		tracker.Release();
	}
	movementOwner = owner;
	movementSerial = ++sequence;
	movementEpoch = tracker.epoch;
}

Movement::~Movement()
{
	movementOwner = previousOwner;
	movementSerial = previous;
	movementEpoch = previousEpoch;
}

ControllerUpdate::ControllerUpdate(std::uintptr_t controller, const void* data) noexcept
{
	previous = movementInput;
	previousEligible = controllerUpdateEligible;
	previousEpoch = controllerUpdateEpoch;
	previousSubmitted = stateSubmitted;
	stateSubmitted = false;
	movementInput = nullptr;
	controllerUpdateEligible = false;
	controllerUpdateEpoch = 0;
	bool request{};
	auto owner = movementOwner;
	{
		std::lock_guard guard(lock);
		if (!movementSerial || !Live(owner) || movementEpoch != owner->tracker.epoch ||
		    owner->tracker.binding.controller != controller) {
			return;
		}
		auto& tracker = owner->tracker;
		float dz{};
		const auto state =
		    SVT::Controller::State(reinterpret_cast<const RE::bhkCharacterController*>(controller));
		if (!tracker.source.active || !tracker.source.ownsZ || !tracker.source.BelongsTo(tracker.binding) ||
		    (!state || !state->AcceptsVelocity()) || !SVT::Controller::MovementZ(data, dz) ||
		    !std::isfinite(dz) ||
		    (state->wanted != RE::hkpCharacterStateType::kTotal && state->wanted != state->current)) {
			tracker.Release();
			return;
		}
		movementInput = data;
		controllerUpdateEligible = true;
		controllerUpdateEpoch = tracker.epoch;
		request = tracker.source.RequestAir(tracker.source.authoredZ, state->current, state->wanted);
	}
	if (request) {
		auto* character = reinterpret_cast<RE::bhkCharacterController*>(controller);
		character->RequestJump(0.0f);
		character->ApplyRequestedState();
	}
}

ControllerUpdate::~ControllerUpdate()
{
	movementInput = previous;
	controllerUpdateEligible = previousEligible;
	controllerUpdateEpoch = previousEpoch;
	stateSubmitted = previousSubmitted;
}

void SubmitVelocity(RE::bhkCharacterController* object, const RE::hkVector4& value, VelocitySource source)
{
	const auto controller = reinterpret_cast<std::uintptr_t>(object);
	auto previous = submission;
	submission = {};

	struct Restore
	{
		Submission previous;
		Owner owner{};
		Id epoch{};
		bool returned{};

		~Restore()
		{
			submission = previous;
			if (!returned && owner) {
				std::lock_guard guard(lock);
				if (epoch == owner->tracker.epoch) {
					owner->tracker.Invalidate();
				}
			}
		}
	} restore{previous};

	float dz{}, dt{}, desired{};
	Origin origin{};
	auto owner = movementOwner;
	{
		std::lock_guard guard(lock);
		if (movementSerial && movementInput && controllerUpdateEligible && Live(owner) &&
		    controllerUpdateEpoch == owner->tracker.epoch && owner->tracker.source.active &&
		    owner->tracker.source.ownsZ && owner->tracker.source.BelongsTo(owner->tracker.binding) &&
		    controller == owner->tracker.binding.controller && Supported(controller) &&
		    source == VelocitySource::Movement && SVT::Controller::IsOutputVelocity(object, &value)) {
			auto& tracker = owner->tracker;
			if (SVT::Controller::MovementZ(movementInput, dz) && SVT::Controller::ReadDeltaTime(object, dt) &&
			    SVT::ReferenceFrameZ::Velocity(dz, dt, worldToHavok, desired)) {
				std::memcpy(&object->outVelocity.quad.m128_f32[2], &desired, sizeof(desired));
				origin = {++sequence, tracker.source.episode};
				submission = {owner, origin, &value, dt};
			}
		} else if (stateSubmitted && movementInput && controllerUpdateEligible && Live(owner) &&
		           controllerUpdateEpoch == owner->tracker.epoch &&
		           controller == owner->tracker.binding.controller &&
		           source == VelocitySource::NativePolicy) {
			float seconds{};
			if (SVT::Controller::ReadDeltaTime(object, seconds)) {
				submission = {owner, {}, &value, seconds, true};
			}
		}
		if (submission.owner) {
			restore.owner = submission.owner;
			restore.epoch = restore.owner->tracker.epoch;
		}
	}
	controllerVelocityOriginal(object, value);
	restore.returned = true;
	submission = previous;
}

VelocityChange::VelocityChange(void* character, const void* value, float dt) noexcept
{
	previousSetter = velocityOwner;
	velocityOwner = nullptr;
	std::lock_guard guard(lock);
	owner = Find(characters, reinterpret_cast<std::uintptr_t>(character));
	const auto* body =
	    owner ? reinterpret_cast<const RE::hkpRigidBody*>(owner->tracker.binding.body) : nullptr;
	if (!Live(owner)) {
		owner.reset();
		return;
	}
	if (!Read(&body->motion.linearVelocity, before)) {
		ReleaseVelocityOwnership(owner);
		owner.reset();
		return;
	}
	epoch = owner->tracker.epoch;
	velocityOwner = owner.get();
	if (submission.owner == owner && submission.velocity == value && submission.dt == dt) {
		origin = submission.origin;
		policy = submission.policy;
	}
}

VelocityChange::~VelocityChange()
{
	if (returned) {
		return;
	}
	velocityOwner = previousSetter;
	std::lock_guard guard(lock);
	if (owner && epoch == owner->tracker.epoch) {
		owner->tracker.Invalidate();
	}
}

void VelocityChange::Returned() noexcept
{
	velocityOwner = previousSetter;
	returned = true;
	if (!owner) {
		return;
	}
	std::lock_guard guard(lock);
	auto& tracker = owner->tracker;
	Vector after{};
	const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
	if (epoch != tracker.epoch || !Live(owner)) {
		return;
	}
	if (!Read(&body->motion.linearVelocity, after)) {
		ReleaseVelocityOwnership(owner);
		return;
	}
	if (policy) {
		// Failure invalidates provenance inside ControllerPolicy.
		(void)tracker.ControllerPolicy(before, after);
	} else {
		tracker.SetVelocity(before, after, origin);
	}
	if (origin && movementOwner == owner) {
		stateSubmitted = true;
	}
	auto* operation = owner->operation;
	if (!policy && operation && operation->epoch == epoch && operation->input &&
	    operation->origin != origin) {
		operation->mixed = true;
	}
}

void SubmitMotionVelocity(RE::hkpMotion* object, const RE::hkVector4& value, bool nativePolicy)
{
	if (!enabled.load(std::memory_order_acquire) || !boundCount.load(std::memory_order_acquire)) {
		motionOriginal(object, value);
		return;
	}
	Vector before{};
	Id epoch{}, physical{};
	Owner owner;
	{
		std::lock_guard guard(lock);
		owner = Find(bodies, reinterpret_cast<std::uintptr_t>(object) - SVT::Controller::kMotion);
		const auto* body =
		    owner ? reinterpret_cast<const RE::hkpRigidBody*>(owner->tracker.binding.body) : nullptr;
		if (Live(owner) && velocityOwner != owner.get()) {
			if (Read(&body->motion.linearVelocity, before)) {
				epoch = owner->tracker.epoch;
				physical = owner->operation ? owner->operation->id : 0;
			} else {
				ReleaseVelocityOwnership(owner);
			}
		}
	}

	struct Restore
	{
		Owner owner;
		Id epoch;
		bool returned{};

		~Restore()
		{
			if (!returned && epoch) {
				std::lock_guard guard(lock);
				if (epoch == owner->tracker.epoch) {
					owner->tracker.Invalidate();
				}
			}
		}
	} restore{owner, epoch};

	motionOriginal(object, value); // Native collision/contact velocity is never replaced here.
	restore.returned = true;
	if (!epoch) {
		return;
	}
	std::lock_guard guard(lock);
	auto& tracker = owner->tracker;
	Vector after{};
	const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
	if (!Live(owner) || epoch != tracker.epoch) {
		return;
	}
	if (!Read(&body->motion.linearVelocity, after)) {
		ReleaseVelocityOwnership(owner);
		return;
	}
	auto* operation = owner->operation;
	if (operation && operation->id == physical && operation->epoch == epoch && nativePolicy) {
		tracker.NativePolicy(*operation, before, after);
	} else {
		tracker.SetVelocity(before, after, {});
		if (operation) {
			operation->mixed = true;
		}
	}
}

WorldUpdate::WorldUpdate() noexcept
{
	std::lock_guard guard(lock);
	Sweep();
}

WorldUpdate::~WorldUpdate()
{
	std::lock_guard guard(lock);
	Sweep();
}

WorldStep::WorldStep(void* world) noexcept
{
	std::lock_guard guard(lock);
	// Complete the only allocation before publishing any Operation pointer.
	try {
		items.reserve(actors.size());
	} catch (const std::bad_alloc&) {
		WorldAllocationFailure(reinterpret_cast<std::uintptr_t>(world));
		return;
	}
	for (const auto& [actor, owner] : actors) {
		(void)actor;
		if (!Live(owner) || owner->tracker.binding.world != reinterpret_cast<std::uintptr_t>(world)) {
			continue;
		}
		if (owner->operation) {
			owner->tracker.Invalidate();
			continue;
		}
		items.push_back({owner, {++sequence, owner->tracker.epoch, owner->tracker.binding.world}});
		owner->operation = &items.back().operation;
	}
}

void WorldStep::Returned(std::uint32_t result) noexcept
{
	std::lock_guard guard(lock);
	for (auto& i : items) {
		if (i.owner->retired || i.operation.epoch != i.owner->tracker.epoch) {
			continue;
		}
		i.owner->tracker.EndOperation(i.operation.id, result == 0);
		if (i.owner->operation == &i.operation) {
			i.owner->operation = nullptr;
		}
	}
	returned = true;
}

WorldStep::~WorldStep()
{
	std::lock_guard guard(lock);
	for (auto& i : items) {
		if (!returned && !i.owner->retired && i.operation.epoch == i.owner->tracker.epoch) {
			i.owner->tracker.EndOperation(i.operation.id, false);
		}
		if (i.owner->operation == &i.operation) {
			i.owner->operation = nullptr;
		}
	}
}

MotionIntegration::MotionIntegration(const void* step, const void* entities, std::int32_t count,
                                     std::int32_t offset, const void* accumulator, Mode mode) noexcept
{
	std::lock_guard guard(lock);
	if (offset != offsetof(RE::hkpRigidBody, motion) || count <= 0 || count > kMaxBodyBatch ||
	    !boundCount.load()) {
		return;
	}
	RE::hkStepInfo stepInfo{};
	if (!Read(static_cast<const RE::hkStepInfo*>(step), stepInfo)) {
		return;
	}
	const Interval time{stepInfo.startTime, stepInfo.endTime, stepInfo.deltaTime, stepInfo.invDeltaTime};
	if (!time) {
		return;
	}
	expectedEnd = reinterpret_cast<std::uintptr_t>(accumulator) + sizeof(RE::hkpVelocityAccumulator) * count;
	// Collect first: vector growth must not follow Tracker::Consume mutations.
	try {
		for (std::int32_t index = 0; index < count; ++index) {
			auto owner =
			    Find(bodies, ReadOrDefault<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(entities) +
			                                               sizeof(RE::hkpMotion*) * index));
			if (!Live(owner)) {
				continue;
			}
			auto& tracker = owner->tracker;
			auto* operation = owner->operation;
			if (!operation || operation->epoch != tracker.epoch ||
			    operation->world != tracker.binding.world) {
				continue;
			}
			const auto selected = accumulator ? reinterpret_cast<std::uintptr_t>(accumulator) +
			                                        sizeof(RE::hkpVelocityAccumulator) * index
			                                  : 0;
			Input input{};
			input.accumulator = selected;
			input.entities = reinterpret_cast<std::uintptr_t>(entities);
			input.index = index;
			input.time = time;
			items.push_back({owner, input, tracker.epoch, operation});
		}
	} catch (const std::bad_alloc&) {
		// This entity batch was not observed completely. Do not let either an
		// earlier pose or a partial batch grant skip-Z after native processing.
		for (std::int32_t index = 0; index < count; ++index) {
			auto owner =
			    Find(bodies, ReadOrDefault<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(entities) +
			                                               sizeof(RE::hkpMotion*) * index));
			if (Live(owner) && owner->operation && owner->operation->epoch == owner->tracker.epoch) {
				AllocationFailure(owner);
			}
		}
		items.clear();
		return;
	}
	for (auto& i : items) {
		auto& tracker = i.owner->tracker;
		const auto pending = i.input;
		if (mode == Mode::Export) {
			i.input =
			    tracker.Find(pending.accumulator, pending.entities, pending.index, time, i.operation->id);
		} else {
			Vector before{};
			const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
			i.input = Read(&body->motion.linearVelocity, before)
			              ? tracker.Consume(*i.operation, pending.accumulator, pending.entities,
			                                pending.index, time, before)
			              : Input{};
		}
	}
}

void MotionIntegration::Built(void* end) noexcept
{
	std::lock_guard guard(lock);
	for (auto& i : items) {
		if (!i.input.id || i.owner->retired || i.epoch != i.owner->tracker.epoch) {
			continue;
		}
		auto& tracker = i.owner->tracker;
		Vector after{};
		const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
		if (!Live(i.owner) || i.owner->operation != i.operation ||
		    reinterpret_cast<std::uintptr_t>(end) != expectedEnd ||
		    !Read(&body->motion.linearVelocity, after)) {
			tracker.Invalidate();
			continue;
		}
		tracker.Damped(i.input, after);
	}
}

void MotionIntegration::Exported() noexcept
{
	std::lock_guard guard(lock);
	for (auto& i : items) {
		if (!Live(i.owner) || i.epoch != i.owner->tracker.epoch || i.owner->operation != i.operation) {
			continue;
		}
		MotionSnapshot pose{};
		Vector after{};
		auto& tracker = i.owner->tracker;
		const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
		if (Read(&body->motion.motionState, pose) && Read(&body->motion.linearVelocity, after)) {
			tracker.Export(*i.operation, i.input, pose, after);
		}
	}
}

PostSimulation::PostSimulation(RE::hkpCharacterRigidBodyListener* listener, RE::hkpWorld* world,
                               RE::hkpCharacterRigidBody* character) noexcept
{
	previous = positionRead;
	positionRead = &state;
	std::lock_guard guard(lock);
	auto owner =
	    Find(controllers,
	         reinterpret_cast<std::uintptr_t>(static_cast<RE::bhkCharRigidBodyController*>(listener)));
	if (!Live(owner) || reinterpret_cast<std::uintptr_t>(world) != owner->tracker.binding.world ||
	    reinterpret_cast<std::uintptr_t>(character) != owner->tracker.binding.character) {
		return;
	}
	state.owner = owner;
	state.epoch = owner->tracker.epoch;
}

PostSimulation::~PostSimulation()
{
	positionRead = previous;
}

void PositionRead(const RE::bhkCharacterController* controller, RE::hkVector4* position,
                  std::uint8_t centerOffset, PositionReadStage stage) noexcept
{
	if (!positionRead) {
		return;
	}
	std::lock_guard guard(lock);
	auto owner = positionRead->owner;
	if (!Live(owner) || positionRead->epoch != owner->tracker.epoch ||
	    reinterpret_cast<std::uintptr_t>(controller) != owner->tracker.binding.controller || !centerOffset) {
		return;
	}
	if (stage == PositionReadStage::Before) {
		positionRead->reading = true;
		positionRead->sampled = false;
		positionRead->valid = false;
		return;
	}
	positionRead->reading = false;
	auto& tracker = owner->tracker;
	SVT::Controller::SimulationTime times{};
	MotionSnapshot motion{};
	const auto* world = reinterpret_cast<const RE::hkpWorld*>(tracker.binding.world);
	const auto* body = reinterpret_cast<const RE::hkpRigidBody*>(tracker.binding.body);
	const auto simulation = ReadOrDefault<std::uintptr_t>(&world->simulation);
	if (!Read(reinterpret_cast<std::uintptr_t>(position), positionRead->getter) ||
	    !FiniteXYZ(positionRead->getter) ||
	    !SVT::Controller::ReadSimulationTime(reinterpret_cast<const RE::hkpSimulation*>(simulation), times) ||
	    !Read(&body->motion.motionState, motion)) {
		return;
	}
	auto* physical =
	    owner->operation && owner->operation->epoch == tracker.epoch ? owner->operation : nullptr;
	positionRead->publication = tracker.Publish(physical, motion, times.current, times.psi, times.marker,
	                                            positionRead->sampled, positionRead->sample);
	positionRead->valid = static_cast<bool>(positionRead->publication);
}

void RecordSampleTime(float time) noexcept
{
	if (positionRead && positionRead->reading) {
		positionRead->sampled = true;
		positionRead->sample = time;
	}
}

MoveFinishEvent::MoveFinishEvent(void* source, const void* position) noexcept
{
	previous = moveFinish;
	moveFinish = &state;
	if (!positionRead || !positionRead->valid) {
		return;
	}
	std::lock_guard guard(lock);
	auto owner = positionRead->owner;
	const auto* controller =
	    owner ? reinterpret_cast<const RE::bhkCharacterController*>(owner->tracker.binding.controller)
	          : nullptr;
	const auto* expectedSource =
	    static_cast<const RE::BSTEventSource<RE::bhkCharacterMoveFinishEvent>*>(controller);
	if (!Live(owner) || positionRead->epoch != owner->tracker.epoch || source != expectedSource ||
	    !Read(reinterpret_cast<std::uintptr_t>(position), state.position)) {
		return;
	}
	for (std::size_t i = 0; i < 3; ++i) {
		if (state.position[i] != positionRead->getter[i] * havokToWorld) {
			return;
		}
	}
	state.owner = owner;
	state.actor = owner->tracker.binding.actor;
	state.epoch = owner->tracker.epoch;
	state.publication = positionRead->publication;
	state.qualified = owner->tracker.CanCommit(state.publication);
}

MoveFinishEvent::~MoveFinishEvent()
{
	moveFinish = previous;
}

void PrepareActorWriteback(std::uintptr_t actor, const void* position) noexcept
{
	if (!moveFinish || !moveFinish->qualified || moveFinish->actor != actor) {
		return;
	}
	std::lock_guard guard(lock);
	auto owner = moveFinish->owner;
	if (!Live(owner) || moveFinish->epoch != owner->tracker.epoch) {
		return;
	}
	std::array<float, 3> point{};
	if (Read(reinterpret_cast<std::uintptr_t>(position), point) && point == moveFinish->position) {
		moveFinish->finishPosition = position;
	}
}

void ConfirmActorWriteback(std::uintptr_t actor, const void* position) noexcept
{
	if (!moveFinish || !moveFinish->qualified || moveFinish->actor != actor) {
		return;
	}
	std::lock_guard guard(lock);
	auto owner = moveFinish->owner;
	if (!Live(owner) || moveFinish->epoch != owner->tracker.epoch) {
		return;
	}
	std::array<float, 3> point{};
	if (moveFinish->finishPosition == position) {
		const auto* reference = reinterpret_cast<const RE::TESObjectREFR*>(actor);
		const bool same = Read(&reference->data.location, point) && point == moveFinish->position;
		if (same) {
			owner->tracker.Committed(moveFinish->publication);
		}
		moveFinish->finishPosition = nullptr;
	}
}

ActorWriteback::ActorWriteback(std::uintptr_t a_actor, const void* a_position) noexcept
    : owner(moveFinish), actor(a_actor), position(a_position)
{
	if (!owner) {
		return;
	}
	previousDepth = owner->finishDepth++;
	if (!previousDepth) {
		PrepareActorWriteback(actor, position);
	}
}

ActorWriteback::~ActorWriteback()
{
	if (!owner) {
		return;
	}
	if (!previousDepth && moveFinish == owner) {
		ConfirmActorWriteback(actor, position);
	}
	owner->finishDepth = previousDepth;
}

bool CanOverrideSkipZ(std::uintptr_t actor, const void* position, bool skipZCaller, bool found) noexcept
{
	if (!moveFinish || moveFinish->finishDepth != 1 || !found || moveFinish->actor != actor ||
	    moveFinish->finishPosition != position || !skipZCaller) {
		return false;
	}
	std::lock_guard guard(lock);
	auto owner = moveFinish->owner;
	if (!Live(owner) || moveFinish->epoch != owner->tracker.epoch ||
	    !Supported(owner->tracker.binding.controller) || !owner->tracker.CanCommit(moveFinish->publication)) {
		return false;
	}
	return true;
}
} // namespace SVT::RigidBodyZ::Native
