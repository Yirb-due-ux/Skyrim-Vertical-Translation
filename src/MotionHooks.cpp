#include "MotionHooks.h"
#include "MotionContext.h"
#include "RigidBodyMotion.h"
#include "RE/A/AIProcess.h"
#include "RE/B/BGSBodyPartDefs.h"
#include "RE/M/MiddleHighProcessData.h"
#include <unordered_map>

namespace SVT::MotionHooks
{
static_assert(offsetof(RE::TESObjectREFR, loadedData) == 0x68);

using SVT::MotionContext;
using SVT::ProxyInstance;
using SVT::ProxyOwner;
ActorMoveFn g_actorMoveOriginal{}, g_characterMoveOriginal{}, g_playerMoveOriginal{};
ControllerUpdateFn g_controllerUpdateOriginal{};
MoveFinishFn g_moveFinishOriginal{};
ProcessMovementFn g_processMovementOriginal{};
AnimationDeltaFn g_animationDeltaOriginal{};
std::atomic<bool> g_installed{};
std::atomic<std::uint64_t> g_nextEpisode{}, g_nextMovement{}, g_nextConsumer{};
float g_worldToHavok{};
// The indexes retain each binding until source end or rebinding; active scopes borrow the same owner.
std::mutex g_proxyLock;
std::unordered_map<std::uintptr_t, ProxyOwner> g_proxyActors, g_proxyControllers;
unsigned g_loadDepth{}; // Serialized with proxy retirement; no native calls under this lock.

bool IsProxy(std::uintptr_t controller) noexcept
{
	return SVT::Controller::HasType(controller, SVT::Controller::types.proxy);
}

bool IsRigid(std::uintptr_t controller) noexcept
{
	return SVT::Controller::HasType(controller, SVT::Controller::types.rigid);
}

MotionContext::ControllerUpdate* CurrentProxyMove(RE::bhkCharacterController* controller) noexcept
{
	return MotionContext::ControllerUpdate::For(controller,
	                                            IsProxy(reinterpret_cast<std::uintptr_t>(controller)));
}

void UnindexProxy(const ProxyOwner& owner) noexcept
{
	const auto actor = g_proxyActors.find(owner->actor);
	if (actor != g_proxyActors.end() && actor->second == owner) {
		g_proxyActors.erase(actor);
	}
	const auto controller = g_proxyControllers.find(owner->controller);
	if (controller != g_proxyControllers.end() && controller->second == owner) {
		g_proxyControllers.erase(controller);
	}
}

void RetireProxy(const ProxyOwner& owner) noexcept
{
	owner->retired = true;
	if (owner->busy) {
		return;
	}
	if (owner->completedNativeWrite) {
		// A stale snapshot leaves the current restoration obligation active.
		(void)owner->lease.AcceptNativeWrite(owner->completedNativeWrite);
		owner->completedNativeWrite = {};
	}
	if (owner->lease.Active()) {
		return; // Keep the ticket until recovery or controller destruction resolves ownership.
	}
	owner->source.End();
	UnindexProxy(owner);
}

namespace
{
// Busy owners retain completed native writes until the last restoration exit has run.
class ChaseBipWrite
{
	inline static thread_local ChaseBipWrite* current{};
	ChaseBipWrite* previous{current};
	ProxyOwner owner{};
	SVT::ChaseBip::NativeWriteSnapshot nativeWrite{};
	const std::uintptr_t returnAddress;

  public:
	explicit ChaseBipWrite(std::uintptr_t caller = ChaseBipHook::getterReturnAddress) noexcept
	    : returnAddress(caller)
	{
		current = this;
	}

	~ChaseBipWrite()
	{
		Hooks::PreserveLastError preserve;
		current = previous;
		owner.reset();
	}

	ChaseBipWrite(const ChaseBipWrite&) = delete;
	ChaseBipWrite& operator=(const ChaseBipWrite&) = delete;

	static ChaseBipWrite* Current() noexcept
	{
		return current;
	}

	bool MatchesCall(std::uintptr_t caller) const noexcept
	{
		return caller == returnAddress;
	}

	void CompleteForController(const RE::bhkCharacterController* controller) noexcept
	{
		if (owner && reinterpret_cast<std::uintptr_t>(controller) == owner->controller) {
			Complete();
		}
	}

	void Capture(RE::bhkCharacterController* controller) noexcept
	{
		Hooks::PreserveLastError preserve;
		owner.reset();
		nativeWrite = {};
		if (!controller) {
			return;
		}
		std::lock_guard lock(g_proxyLock);
		const auto address = reinterpret_cast<std::uintptr_t>(controller);
		const auto found = g_proxyControllers.find(address);
		if (found == g_proxyControllers.end()) {
			return;
		}
		const auto& candidate = found->second;
		if (candidate->retired || candidate->busy) {
			nativeWrite = candidate->lease.PendingNativeWrite(address);
			if (nativeWrite) {
				owner = candidate;
			}
		}
	}

	void Complete() noexcept
	{
		if (!owner) {
			return;
		}
		Hooks::PreserveLastError preserve;
		std::lock_guard lock(g_proxyLock);
		const auto found = g_proxyControllers.find(owner->controller);
		if (found != g_proxyControllers.end() && found->second == owner &&
		    owner->lease.ConfirmNativeWrite(nativeWrite)) {
			owner->completedNativeWrite = nativeWrite;
			if (owner->retired) {
				RetireProxy(owner);
			}
		}
	}
};
} // namespace

void ChaseBipHook::Thunk(RE::Actor* actor, std::uint8_t enabled)
{
	ChaseBipWrite write;
	original(actor, enabled);
	write.Complete(); // A throw before the native write cannot acknowledge the old ticket.
}

RE::bhkCharacterController* ChaseBipHook::GetControllerThunk(RE::AIProcess* process)
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	auto* result = getControllerOriginal(process);
	// Capture only a result returning directly to the setter; a later CALL wrapper could replace RAX.
	if (auto* write = ChaseBipWrite::Current(); write && write->MatchesCall(caller)) {
		write->Capture(result);
	}
	return result;
}

void PlayerDeathHook::Thunk(RE::Actor* actor, RE::Actor* attacker, float damage, std::uint8_t sendEvent,
                            std::uint8_t ragdollInstant)
{
	ChaseBipWrite write(getterReturnAddress);
	original(actor, attacker, damage, sendEvent, ragdollInstant);
}

RE::bhkCharacterController* PlayerDeathHook::GetControllerThunk(RE::AIProcess* process)
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	auto* result = getControllerOriginal(process);
	if (auto* write = ChaseBipWrite::Current(); write && write->MatchesCall(caller)) {
		write->Capture(result);
	}
	return result;
}

void ControllerLoadHook::Thunk(RE::AIProcess* process, RE::BGSLoadGameBuffer* buffer)
{
	ChaseBipWrite write(writeReturnAddress);
	RE::MiddleHighProcessData* middleHigh{};
	RE::bhkCharacterController* controller{};
	// This is only a pre-write candidate; the later native destination must confirm the same object.
	static_assert(offsetof(RE::AIProcess, middleHigh) == 0x8);
	static_assert(offsetof(RE::MiddleHighProcessData, charController) == 0x250);
	if (process && Controller::Read(&process->middleHigh, middleHigh) && middleHigh &&
	    Controller::Read(&middleHigh->charController, controller)) {
		write.Capture(controller);
	}
	original(process, buffer);
}

void PlayerDeathHook::ConfirmWrite(RE::bhkCharacterController* controller) noexcept
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	Hooks::PreserveLastError preserve;
	// The leaf relay has executed the original OR; reject a later caller that can substitute RCX.
	if (auto* write = ChaseBipWrite::Current();
	    caller == writeReturnAddress && write && write->MatchesCall(getterReturnAddress) &&
	    Hooks::HookTransaction::Equals(writeReturnAddress - 5, writeCallExpected)) {
		write->CompleteForController(controller);
	}
}

std::uint32_t ControllerLoadHook::ConfirmWrite(RE::bhkCharacterController* controller,
                                               std::uint32_t flags) noexcept
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	Hooks::PreserveLastError preserve;
	// Confirm the actual RBX store before any native position/velocity call, without rereading flags.
	if (auto* write = ChaseBipWrite::Current();
	    caller == writeReturnAddress && write && write->MatchesCall(writeReturnAddress) &&
	    Hooks::HookTransaction::Equals(writeReturnAddress - 5, writeCallExpected)) {
		write->CompleteForController(controller);
	}
	return flags; // The following native BT still tests the original loaded EAX.
}

void ReleaseResolvedProxy(std::uintptr_t controller) noexcept
{
	Hooks::PreserveLastError preserve;
	std::lock_guard lock(g_proxyLock);
	const auto found = g_proxyControllers.find(controller);
	if (found == g_proxyControllers.end()) {
		return;
	}
	const auto owner = found->second;
	if (owner->retired && !owner->busy && owner->lease.ReleaseIfResolved(controller)) {
		RetireProxy(owner);
	}
}

void Actor3DHook::Thunk(RE::Actor* actor, RE::NiAVObject* object, std::uint8_t queueTasks)
{
	if (object) {
		original(actor, object, queueTasks);
		return;
	}
	MotionContext::ActorUnload unload(actor);
	{
		Hooks::PreserveLastError preserve;
		std::lock_guard lock(g_proxyLock);
		const auto found = g_proxyActors.find(reinterpret_cast<std::uintptr_t>(actor));
		if (found != g_proxyActors.end()) {
			const auto owner = found->second;
			RetireProxy(owner);
		}
		RigidBodyZ::Native::RetireActor(reinterpret_cast<std::uintptr_t>(actor));
	}
	original(actor, object, queueTasks);
}

void ControllerDestroyHook::Thunk(RE::bhkCharacterController* controller)
{
	{
		Hooks::PreserveLastError preserve;
		std::lock_guard lock(g_proxyLock);
		const auto address = reinterpret_cast<std::uintptr_t>(controller);
		const auto found = g_proxyControllers.find(address);
		if (found != g_proxyControllers.end()) {
			const auto owner = found->second;
			owner->retired = true;
			owner->lease.ForgetDestroyedController(address);
			// The address may be reused after native deletion. Borrowing scopes
			// retain this owner, but their old tickets can no longer write flags.
			UnindexProxy(owner);
		}
		RigidBodyZ::Native::RetireController(address);
	}
	original(controller);
}

void ReleaseProxyMovement(const ProxyOwner& owner) noexcept
{
	Hooks::PreserveLastError preserve;
	bool failedRestore{};
	DWORD readError{};
	{
		std::lock_guard lock(g_proxyLock);
		owner->busy = false;
		if (owner->lease.Active()) {
			owner->retired = true; // The final owning exit failed; a future Move cannot safely borrow it.
		} else {
			owner->completedNativeWrite = {}; // A successful restoration already ended the captured ticket.
		}
		if (owner->retired) {
			RetireProxy(owner);
			failedRestore = owner->lease.Active() && !owner->retirementReported;
			owner->retirementReported = owner->retirementReported || failedRestore;
			if (failedRestore) {
				readError = owner->lease.RestoreReadError();
			}
		}
	}
	if (failedRestore) {
		try {
			// This is a later page observation, not proof of the failed read's cause or object lifetime.
			const auto flagsAddress = owner->controller + SVT::Controller::kFlags;
			MEMORY_BASIC_INFORMATION memory{};
			const auto queried = VirtualQuery(reinterpret_cast<void*>(flagsAddress), &memory, sizeof(memory));
			const auto queryError = queried ? ERROR_SUCCESS : GetLastError();
			SKSE::log::error(
			    "Unable to restore controller flags; cleanup deferred. actor={:X} controller={:X}"
			    " flagsAddress={:X} flagsReadWin32={} memoryStateNow={:X} memoryProtectNow={:X}"
			    " memoryQueryWin32={}",
			    owner->actor, owner->controller, flagsAddress, readError, memory.State, memory.Protect,
			    queryError);
		} catch (...) {
		}
	}
}

void OnGameMessage(std::uint32_t type) noexcept
{
	using Message = SKSE::MessagingInterface;
	std::size_t unresolvedLeases{};
	{
		std::lock_guard lock(g_proxyLock);
		if (type == Message::kPostLoadGame) {
			if (g_loadDepth && !--g_loadDepth) {
				ReferenceFrameZ::sourceLifetime.fetch_add(1, std::memory_order_acq_rel);
			}
			return; // Both successful and failed attempts may admit a fresh source afterward.
		}
		if (type == Message::kPreLoadGame) {
			if (g_loadDepth++) {
				return;
			}
			ReferenceFrameZ::sourceLifetime.fetch_add(1, std::memory_order_acq_rel);
		} else if (type == Message::kNewGame) {
			ReferenceFrameZ::sourceLifetime.fetch_add(2, std::memory_order_acq_rel);
		} else {
			return;
		}
		for (auto it = g_proxyActors.begin(); it != g_proxyActors.end();) {
			const auto owner = it++->second;
			RetireProxy(owner);
			if (!owner->busy && owner->lease.Active() && !owner->retirementReported) {
				owner->retirementReported = true;
				++unresolvedLeases;
			}
		}
		// This order has no inverse: rigid code never takes g_proxyLock.
		RigidBodyZ::Native::RetirePreviousLifetimes();
	}
	if (unresolvedLeases) {
		try {
			SKSE::log::error("Controller cleanup deferred: {} unresolved flag restorations.",
			                 unresolvedLeases);
		} catch (...) {
		}
	}
}

void EndProxy(std::uintptr_t actor, std::uint64_t lifetime = ReferenceFrameZ::CurrentLifetime(),
              const std::atomic<bool>* cancelled = nullptr, std::uint64_t sourceSerial = 0) noexcept
{
	std::lock_guard lock(g_proxyLock);
	if (!ReferenceFrameZ::IsCurrentLifetime(lifetime) || (cancelled && cancelled->load())) {
		return;
	}
	const auto found = g_proxyActors.find(actor);
	if (found == g_proxyActors.end() || found->second->busy) {
		return;
	}
	if (sourceSerial && found->second->sourceSerial > sourceSerial) {
		return;
	}
	const auto owner = found->second;
	RetireProxy(owner);
}

ProxyOwner BeginProxy(std::uintptr_t actor, std::uint32_t handle, std::uintptr_t controller,
                      std::uint64_t lifetime = ReferenceFrameZ::CurrentLifetime(),
                      const std::atomic<bool>* cancelled = nullptr, std::uint64_t sourceSerial = 0) noexcept
{
	std::lock_guard lock(g_proxyLock);
	if (!ReferenceFrameZ::IsCurrentLifetime(lifetime) || (cancelled && cancelled->load())) {
		return {};
	}
	auto found = g_proxyActors.find(actor);
	if (found != g_proxyActors.end()) {
		if (found->second->busy) {
			return {};
		}
		if (found->second->retired || found->second->handle != handle ||
		    found->second->controller != controller) {
			const auto old = found->second;
			RetireProxy(old);
			if (old->lease.Active()) {
				return {};
			}
			found = g_proxyActors.end();
		}
	}
	const auto reused = g_proxyControllers.find(controller);
	if (reused != g_proxyControllers.end() && reused->second->actor != actor) {
		if (reused->second->busy) {
			return {}; // A live borrowed controller cannot change owners.
		}
		const auto old = reused->second;
		RetireProxy(old);
		if (old->lease.Active()) {
			return {};
		}
	}
	ProxyOwner owner;
	try {
		if (found == g_proxyActors.end()) {
			owner = std::make_shared<ProxyInstance>();
			owner->actor = actor;
			owner->handle = handle;
			owner->lifetime = lifetime;
			owner->controller = controller;
			g_proxyActors.emplace(actor, owner);
			g_proxyControllers.emplace(controller, owner);
		} else {
			owner = found->second;
		}
		if (!owner->source.active) {
			owner->source.Begin(g_nextEpisode.fetch_add(1, std::memory_order_relaxed) + 1);
		}
		owner->sourceSerial = std::max(owner->sourceSerial, sourceSerial);
		owner->busy = true;
		return owner;
	} catch (const std::bad_alloc&) {
		if (owner) {
			RetireProxy(owner); // Remove either partially inserted index; never revive an old binding.
		}
		return {};
	}
}

void EndSources(RE::Actor* actor, std::uint64_t lifetime, std::uint64_t sourceSerial) noexcept
{
	if (!actor || MotionContext::IsMoving(actor)) {
		return;
	}
	const auto address = reinterpret_cast<std::uintptr_t>(actor);
	EndProxy(address, lifetime, nullptr, sourceSerial);
	// Revocation needs only the captured identity, not a new borrow of the Actor.
	SVT::RigidBodyZ::Native::SetSource(address, 0, 0, false, 0, lifetime, nullptr, sourceSerial);
}

bool ResolveActorHandle(RE::Actor* actor, std::uint32_t& handle)
{
	if (!actor || actor->IsDragon()) {
		return false;
	}
	handle = actor->GetHandle().native_handle();
	return handle != 0;
}

void NativeRootHook::ProcessMovement(RE::Actor* actor, float seconds)
{
	MotionContext::Consumer source(actor, actor ? reinterpret_cast<std::uintptr_t>(actor->currentProcess) : 0,
	                               g_nextConsumer.fetch_add(1, std::memory_order_relaxed) + 1, &EndSources);
	g_processMovementOriginal(actor, seconds);
	source.Returned();
}

const RE::NiPoint3* NativeRootHook::GetAnimationDelta(RE::AIProcess* process)
{
	const auto* result = g_animationDeltaOriginal(process);
	if (auto* source = MotionContext::Consumer::ForProcess(process);
	    g_installed.load(std::memory_order_acquire) && source) {
		RE::HighProcessData* high{};
		SVT::ReferenceFrameZ::Vector position{};
		const bool valid = process && SVT::Controller::Read(&process->high, high) && high &&
		                   SVT::Controller::Read(result, position) &&
		                   SVT::ReferenceFrameZ::FiniteXYZ(position);
		std::uint32_t handle{};
		bool eligible{};
		try {
			eligible = valid && ResolveActorHandle(source->Actor(), handle);
		} catch (const std::bad_alloc&) {
			// Failed handle creation revokes only this consumer; the original getter already returned.
		}
		source->Accept(eligible, handle, valid);
	}
	return result;
}

RE::bhkCharacterController* NativeRootHook::DispatchMove(RE::Actor* actor, float seconds,
                                                         const RE::NiPoint3& delta, std::uintptr_t target)
{
	MotionContext::Dispatch source(actor, actor ? reinterpret_cast<std::uintptr_t>(actor->currentProcess) : 0,
	                               seconds, delta, g_installed.load(std::memory_order_acquire), &EndSources);
	return ActorMoveFn{target}(actor, seconds, delta);
}

RE::bhkCharacterController* ActorMoveHook::Call(ActorMoveFn& original, RE::Actor* actor, float seconds,
                                                const RE::NiPoint3& delta)
{
	MotionContext::Movement move(actor, g_proxyLock, &ReleaseProxyMovement);
	auto* controller = actor ? actor->GetCharController() : nullptr;
	const auto address = reinterpret_cast<std::uintptr_t>(actor);
	const auto control = reinterpret_cast<std::uintptr_t>(controller);
	const auto form = actor ? actor->GetFormID() : 0;
	const bool root = move.Claim(seconds, delta, g_installed.load(std::memory_order_acquire));
	const auto handle = move.Handle();
	RE::LOADED_REF_DATA* loaded{};
	// Actor::Move returns before controller update when loadedData is absent.
	const bool canReadState =
	    root && actor && SVT::Controller::Read(&actor->loadedData, loaded) && loaded && controller;
	const auto state = canReadState ? SVT::Controller::State(controller) : std::nullopt;
	const bool supported = state && state->AcceptsVelocity();
	ProxyOwner owner;
	if (supported && IsProxy(control)) {
		owner =
		    BeginProxy(address, handle, control, move.Lifetime(), move.Cancellation(), move.SourceSerial());
	} else if (move.CurrentSource() && !move.ProtectsParent()) {
		EndProxy(address, move.Lifetime(), move.Cancellation(), move.SourceSerial());
	}
	if (supported && IsRigid(control)) {
		SVT::RigidBodyZ::Native::SetSource(address, form, handle, true, delta.z, move.Lifetime(),
		                                   move.Cancellation(), move.SourceSerial());
	} else if (!move.ProtectsParent() && actor) {
		SVT::RigidBodyZ::Native::SetSource(address, form, handle, false, 0, move.Lifetime(),
		                                   move.Cancellation(), move.SourceSerial());
	}
	SVT::RigidBodyZ::Native::Movement rigidMovement(address, form, handle, control);
	if (owner) {
		move.Bind(owner, controller, g_nextMovement.fetch_add(1) + 1);
		bool request{};
		{
			std::lock_guard lock(g_proxyLock);
			request = move.Available() && owner->source.RequestAir(delta.z, state->current, state->wanted);
		}
		if (request) {
			controller->RequestJump(0.0f);
			controller->ApplyRequestedState();
		}
	}
	auto* result = original(actor, seconds, delta);
	move.RestoreLease();
	return result;
}

RE::bhkCharacterController* ActorMoveHook::ActorThunk(RE::Actor* actor, float seconds,
                                                      const RE::NiPoint3& delta)
{
	return Call(g_actorMoveOriginal, actor, seconds, delta);
}

RE::bhkCharacterController* ActorMoveHook::CharacterThunk(RE::Actor* actor, float seconds,
                                                          const RE::NiPoint3& delta)
{
	return Call(g_characterMoveOriginal, actor, seconds, delta);
}

RE::bhkCharacterController* ActorMoveHook::PlayerThunk(RE::Actor* actor, float seconds,
                                                       const RE::NiPoint3& delta)
{
	return Call(g_playerMoveOriginal, actor, seconds, delta);
}

std::uint32_t ControllerUpdateHook::Thunk(RE::bhkCharacterController* controller,
                                          RE::bhkCharacterControllerMovement* data)
{
	// This native receiver can settle an idle retired lease; it cannot authorize an old-value write.
	ReleaseResolvedProxy(reinterpret_cast<std::uintptr_t>(controller));
	SVT::RigidBodyZ::Native::ControllerUpdate rigidMovement(reinterpret_cast<std::uintptr_t>(controller),
	                                                        data);
	MotionContext::ControllerUpdate move(controller, data,
	                                     IsProxy(reinterpret_cast<std::uintptr_t>(controller)));
	const auto result = g_controllerUpdateOriginal(controller, *data);
	move.Close(); // Restore the lease before Actor::Move reads native position and land height.
	return result;
}

void ProxyHooks::VelocityThunk(RE::bhkCharacterController* controller, const RE::hkVector4& velocity)
{
	auto* scope = CurrentProxyMove(controller);
	float dz{}, seconds{}, desired{};
	if (scope && SVT::Controller::IsOutputVelocity(controller, &velocity) &&
	    SVT::Controller::MovementZ(scope->Input(), dz) &&
	    SVT::Controller::ReadDeltaTime(controller, seconds) &&
	    SVT::ReferenceFrameZ::Velocity(dz, seconds, g_worldToHavok, desired)) {
		const_cast<RE::hkVector4&>(velocity).quad.m128_f32[2] = desired;
		scope->Result().authoredVelocitySubmitted = true;
		scope->Result().integrated = false;
	}
	g_velocityOriginal(controller, velocity);
}

bool ProxyHooks::IntegrateThunk(RE::bhkCharacterController* controller)
{
	auto* scope = CurrentProxyMove(controller);
	if (!scope) {
		return g_integrateOriginal(controller);
	}
	auto& correction = scope->Result();
	correction.authoredVelocitySubmitted = correction.integrated = false;
	const auto result = g_integrateOriginal(controller);
	std::array<float, 3> position{};
	if (result && correction.authoredVelocitySubmitted &&
	    SVT::Controller::ProxyPosition(controller, position)) {
		correction.proxyPosition = position;
		correction.integrated = true;
	}
	return result;
}

bool CanCommitCorrectedZ(RE::TESObjectREFR* actor, const RE::NiPoint3* position) noexcept
{
	auto* controller = MotionContext::Controller();
	const auto* scope = MotionContext::ControllerUpdate::For(controller, true);
	if (!scope || !actor || !position) {
		return false;
	}
	std::array<float, 3> proxyNow{};
	return SVT::Controller::ProxyPosition(controller, proxyNow) &&
	       scope->CanCommit(actor, position, proxyNow);
}

void MoveFinishHook::Thunk(RE::Actor* actor, const RE::NiPoint3& position)
{
	SVT::RigidBodyZ::Native::ActorWriteback rigidFinish(reinterpret_cast<std::uintptr_t>(actor), &position);
	MotionContext::Finish finish(actor, &position, CanCommitCorrectedZ(actor, &position));
	g_moveFinishOriginal(actor, position);
}

bool SkipZHook::Thunk(const RE::BGSFootIkRaycastInterfaceDB* context, RE::TESObjectREFR* actor,
                      std::uint8_t* output)
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	const auto result = g_original(context, actor, output);
	SkipZHook::PreserveZ(actor, output, result, caller == g_returnAddress);
	return result;
}

void SkipZHook::PreserveZ(RE::TESObjectREFR* actor, std::uint8_t* output, bool result, bool sameCallsite)
{
	auto* finish = MotionContext::Finish::Current();
	if (result && sameCallsite && output && *output && finish && finish->Owns(actor) &&
	    CanCommitCorrectedZ(actor, finish->Position())) {
		*output = 0;
	}
	if (output && *output && finish &&
	    SVT::RigidBodyZ::Native::CanOverrideSkipZ(reinterpret_cast<std::uintptr_t>(actor), finish->Position(),
	                                              sameCallsite, result)) {
		*output = 0;
	}
}

} // namespace SVT::MotionHooks
