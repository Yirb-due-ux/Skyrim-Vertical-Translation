#include "MotionHooks.h"
#include "Controller.h"
#include "PhysicsHooks.h"
#include "Plugin.h"
#include "ReferenceFrameCall.h"
#include "RigidBodyMotion.h"
#include "RE/B/bhkWorld.h"
#include "RE/A/AIProcess.h"
#include "RE/Offsets_VTABLE.h"

namespace SVT::MotionHooks
{
SVT::Hooks::HookTransaction g_hooks;
std::uintptr_t g_imageBase{};

namespace
{
// Private SE Address Library entries for lifecycle and flags ownership boundaries.
constexpr REL::ID kActorSet3D{36199};
constexpr REL::ID kControllerDestructor{76419};
constexpr REL::ID kSetChaseBip{36549};
constexpr REL::ID kGetCharController{38826};
constexpr REL::ID kPlayerKill{39646};
constexpr REL::ID kActorKill{36872};
constexpr REL::ID kLoadQueuedController{35119};
constexpr REL::ID kLoadController{38982};
// Guard whole instructions before SafetyHook relocates the original entry.
// clang-format off
constexpr std::array<std::uint8_t, 8> kActorSet3DEntryExpected{
    0x48, 0x8B, 0xC4,             // mov rax, rsp
    0x57,                         // push rdi
    0x48, 0x83, 0xEC, 0x60};      // sub rsp, 0x60
constexpr std::array<std::uint8_t, 6> kControllerDestructorEntryExpected{
    0x40, 0x57,                   // push rdi
    0x48, 0x83, 0xEC, 0x30};      // sub rsp, 0x30
// Guard the complete setter around its saved getter CALL: a non-null return must be written before RET.
constexpr std::array<std::uint8_t, 21> kChaseBipBeforeGetterExpected{
    0x40, 0x53, 0x48, 0x83, 0xEC, 0x20,             // save RBX and reserve stack
    0x48, 0x8B, 0x89, 0xF0, 0x00, 0x00, 0x00,       // load Actor's process
    0x0F, 0xB6, 0xDA,                               // preserve the raw input byte
    0x48, 0x85, 0xC9, 0x74, 0x28};                  // skip when process is null
constexpr std::array<std::uint8_t, 25> kChaseBipSetBranchExpected{
    0x48, 0x85, 0xC0, 0x74, 0x1E,                   // skip when the getter returned null
    0x84, 0xDB, 0x74, 0x10,                         // choose clear/set from the input byte
    0x81, 0x88, 0x18, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, // set bit11
    0x48, 0x83, 0xC4, 0x20, 0x5B, 0xC3};           // return after the write
constexpr std::array<std::uint8_t, 16> kChaseBipClearBranchExpected{
    0x81, 0xA0, 0x18, 0x02, 0x00, 0x00, 0xFF, 0xF7, 0xFF, 0xFF, // clear bit11
    0x48, 0x83, 0xC4, 0x20, 0x5B, 0xC3};           // return after the write or null case
// Player-only process lookup before the saved getter CALL.
constexpr std::array<std::uint8_t, 21> kPlayerDeathDispatchExpected{
    0x4C, 0x3B, 0x3D, 0x03, 0xAB, 0x8F, 0x02, // cmp r15, qword ptr [Player alias]
    0x75, 0x22, // jnz continuation
    0x49, 0x8B, 0x8F, 0xF0, 0x00, 0x00, 0x00, // mov rcx, qword ptr [r15 + f0h]
    0x48, 0x85, 0xC9, // test rcx, rcx
    0x74, 0x22 // jz continuation
};
// non-null getter result is written before continuation.
constexpr std::array<std::uint8_t, 17> kPlayerDeathWriteExpected{
    0x48, 0x85, 0xC0, // test rax, rax
    0x74, 0x18, // jz continuation
    0x81, 0x88, 0x18, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, // or dword ptr [rax + 218h], 800h
    0xEB, 0x0C // jmp continuation
};
// native RBX captures one middle-high controller or exits.
constexpr std::array<std::uint8_t, 32> kLoadControllerBindingExpected{
    0x48, 0x8B, 0x5B, 0x08, // mov rbx, qword ptr [rbx + 8h]
    0x48, 0x8B, 0xF0, // mov rsi, rax
    0x48, 0x85, 0xDB, // test rbx, rbx
    0x0F, 0x84, 0xE6, 0x02, 0x00, 0x00, // jz continuation
    0x48, 0x8B, 0x9B, 0x50, 0x02, 0x00, 0x00, // mov rbx, qword ptr [rbx + 250h]
    0x48, 0x85, 0xDB, // test rbx, rbx
    0x0F, 0x84, 0xD6, 0x02, 0x00, 0x00 // jz continuation
};
// four-byte flags read into the local stack value.
constexpr std::array<std::uint8_t, 19> kLoadFlagsInputExpected{
    0x48, 0x8D, 0x55, 0xA8, // lea rdx, [rbp + -58h]
    0x41, 0xB9, 0x04, 0x00, 0x00, 0x00, // mov r9d, 4h
    0x89, 0x83, 0x1C, 0x02, 0x00, 0x00, // mov dword ptr [rbx + 21ch], eax
    0x45, 0x8B, 0xC1 // mov r8d, r9d
};
// initialize and load the flags value before its store.
constexpr std::array<std::uint8_t, 15> kLoadFlagsReadExpected{
    0xC7, 0x45, 0xA8, 0x00, 0x00, 0x00, 0x00, // mov dword ptr [rbp + -58h], 0h
    0x48, 0x8B, 0xCF, // mov rcx, rdi
    0xE8, 0x7E, 0x56, 0xF1, 0xFF // call LoadDataEndian
};
// whole-word store precedes both branches.
constexpr std::array<std::uint8_t, 23> kLoadFlagsWriteExpected{
    0x8B, 0x45, 0xA8, // mov eax, dword ptr [rbp + -58h]
    0xF3, 0x0F, 0x10, 0x35, 0x6F, 0xDF, 0xEB, 0x00, // movss xmm6, dword ptr [world scale]
    0x89, 0x83, 0x18, 0x02, 0x00, 0x00, // mov dword ptr [rbx + 218h], eax
    0x0F, 0xBA, 0xE0, 0x0B, // bt eax, 11 (bit11)
    0x73, 0x5A // jnc continuation
};
// clang-format on
// SE 1.5.97 guards for the native state and writeback assumptions in NATIVE-TYPES.md.
// Validators below are read-only; death/load stores above also supply their relocated original instruction.
// clang-format off
constexpr std::array<std::uint8_t, 27> kRequestJumpExpectedBytes{
    0xF3, 0x0F, 0x59, 0x0D, 0x64, 0x17, 0x78, 0x00,             // scale jump input
    0xC7, 0x81, 0x1C, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, // request Jumping
    0xF3, 0x0F, 0x11, 0x89, 0x3C, 0x02, 0x00, 0x00,             // store jump input
    0xC3
};
constexpr std::array<std::uint8_t, 28> kApplyRequestedStateExpectedBytes{
    0x8B, 0x81, 0x1C, 0x02, 0x00, 0x00,                         // read requested state
    0x83, 0xF8, 0x0B,
    0x74, 0x10,                                                 // return if no request
    0x89, 0x81, 0x00, 0x02, 0x00, 0x00,                         // publish current state
    0xC7, 0x81, 0x1C, 0x02, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, // clear request
    0xC3
};
constexpr std::array<std::uint8_t, 18> kMoveFinishSkipZExpectedBytes{
    0x80, 0x7C, 0x24, 0x48, 0x00,                               // test skip-Z output
    0x74, 0x0B,
    0xF3, 0x0F, 0x10, 0x43, 0x5C,                               // read current Actor Z
    0xF3, 0x0F, 0x11, 0x44, 0x24, 0x28                          // replace event Z
};
constexpr std::array<std::uint8_t, 12> kMovementContinuationExpectedBytes{
    0x48, 0x8B, 0x06,                                           // reload Actor vptr
    0x48, 0x8B, 0xCE,
    0xFF, 0x90, 0xD8, 0x08, 0x00, 0x00
};
// clang-format on
} // namespace

void PrepareNativeMove(std::uintptr_t site)
{
	Hooks::RequireBytes(site, SVT::ReferenceFrameZ::kMoveCallOriginal, "Actor Move dispatch");
	const auto shim =
	    SVT::ReferenceFrameZ::MoveShim(reinterpret_cast<std::uintptr_t>(&NativeRootHook::DispatchMove));
	auto* block =
	    static_cast<std::uint8_t*>(SKSE::GetTrampoline().allocate(sizeof(std::uintptr_t) + shim.size()));
	const auto address = reinterpret_cast<std::uintptr_t>(block + sizeof(std::uintptr_t));
	std::memcpy(block + sizeof(std::uintptr_t), shim.data(), shim.size());
	std::memcpy(block, &address, sizeof(address)); // Complete target before publishing its call.
	if (!FlushInstructionCache(GetCurrentProcess(), block, sizeof(address) + shim.size())) {
		throw std::runtime_error(
		    std::format("Actor Move relay cache flush failed at {:#x}; win32={}", site, GetLastError()));
	}
	const auto after = SVT::ReferenceFrameZ::MoveCall(site, reinterpret_cast<std::uintptr_t>(block));
	g_hooks.Add(site, SVT::ReferenceFrameZ::kMoveCallOriginal, after);
}

namespace
{
void PrepareStateRecoveryHooks()
{
	Hooks::StageEntry(g_hooks, REL::Relocation<std::uintptr_t>{kActorSet3D}.address(),
	                  kActorSet3DEntryExpected, &Actor3DHook::Thunk, Actor3DHook::original);
	Hooks::StageEntry(g_hooks, REL::Relocation<std::uintptr_t>{kControllerDestructor}.address(),
	                  kControllerDestructorEntryExpected, &ControllerDestroyHook::Thunk,
	                  ControllerDestroyHook::original);
	const auto setChaseBip = REL::Relocation<std::uintptr_t>{kSetChaseBip}.address();
	const auto chaseGetterCall = setChaseBip + 0x15;
	ChaseBipHook::getterReturnAddress = chaseGetterCall + 5;
	Hooks::RequireBytes(setChaseBip, kChaseBipBeforeGetterExpected, "SetChaseBip getter dispatch");
	Hooks::RequireBytes(ChaseBipHook::getterReturnAddress, kChaseBipSetBranchExpected,
	                    "SetChaseBip set branch");
	Hooks::RequireBytes(setChaseBip + 0x33, kChaseBipClearBranchExpected, "SetChaseBip clear branch");
	// Preserve a chained getter's real return value; do not issue another native lookup.
	Hooks::StageCall(g_hooks, chaseGetterCall, REL::Relocation<std::uintptr_t>{kGetCharController}.address(),
	                 &ChaseBipHook::GetControllerThunk, ChaseBipHook::getControllerOriginal, true);
	Hooks::StageEntry(g_hooks, setChaseBip, std::span{kChaseBipBeforeGetterExpected}.first<6>(),
	                  &ChaseBipHook::Thunk, ChaseBipHook::original);
	const auto playerKill = REL::Relocation<std::uintptr_t>{kPlayerKill}.address();
	const auto actorKill = REL::Relocation<std::uintptr_t>{kActorKill}.address();
	PlayerDeathHook::getterReturnAddress = actorKill + 0x11B8;
	Hooks::RequireBytes(actorKill + 0x119E, kPlayerDeathDispatchExpected, "Player death getter dispatch");
	Hooks::RequireBytes(PlayerDeathHook::getterReturnAddress, kPlayerDeathWriteExpected,
	                    "Player death bit11 write");
	Hooks::StageCall(g_hooks, playerKill + 0xEC, actorKill, &PlayerDeathHook::Thunk,
	                 PlayerDeathHook::original, true);
	Hooks::StageCall(g_hooks, PlayerDeathHook::getterReturnAddress - 5,
	                 REL::Relocation<std::uintptr_t>{kGetCharController}.address(),
	                 &PlayerDeathHook::GetControllerThunk, PlayerDeathHook::getControllerOriginal, true);
	// Both relays are leaves: preserve native nonvolatiles/stack and tail-jump to typed C++.
	// Volatile values are dead here except loaded EAX below; NATIVE-TYPES.md records the continuations.
	const auto deathStore = std::span{kPlayerDeathWriteExpected}.subspan<5, 10>();
	Assembly::Code deathWrite(32);
	deathWrite.db(deathStore.data(), deathStore.size());
	deathWrite.mov(deathWrite.rcx, deathWrite.rax);
	deathWrite.AbsoluteJump(reinterpret_cast<std::uintptr_t>(&PlayerDeathHook::ConfirmWrite));
	deathWrite.RequireSize(27);
	PlayerDeathHook::writeReturnAddress = actorKill + 0x11BD + 5;
	Assembly::Call(PlayerDeathHook::writeCallExpected, PlayerDeathHook::writeReturnAddress - 5,
	               Hooks::Publish(deathWrite));
	g_hooks.Add(PlayerDeathHook::writeReturnAddress - 5, deathStore, PlayerDeathHook::writeCallExpected);
	const auto loadController = REL::Relocation<std::uintptr_t>{kLoadController}.address();
	ControllerLoadHook::writeReturnAddress = loadController + 0xAD + 5;
	Hooks::RequireBytes(loadController + 0x1B, kLoadControllerBindingExpected, "loaded controller binding");
	Hooks::RequireBytes(loadController + 0x80, kLoadFlagsInputExpected, "controller flags load input");
	Hooks::RequireBytes(loadController + 0x93, kLoadFlagsReadExpected, "controller flags read");
	Hooks::RequireBytes(loadController + 0xA2, kLoadFlagsWriteExpected, "controller flags store");
	Hooks::StageCall(g_hooks, REL::Relocation<std::uintptr_t>{kLoadQueuedController}.address() + 0x8A,
	                 loadController, &ControllerLoadHook::Thunk, ControllerLoadHook::original, true);
	const auto loadStore = std::span{kLoadFlagsWriteExpected}.subspan<11, 6>();
	Assembly::Code loadWrite(32);
	loadWrite.db(loadStore.data(), loadStore.size());
	loadWrite.mov(loadWrite.edx, loadWrite.eax);
	loadWrite.mov(loadWrite.rcx, loadWrite.rbx);
	loadWrite.AbsoluteJump(reinterpret_cast<std::uintptr_t>(&ControllerLoadHook::ConfirmWrite));
	loadWrite.RequireSize(25);
	Assembly::Call(ControllerLoadHook::writeCallExpected, ControllerLoadHook::writeReturnAddress - 5,
	               Hooks::Publish(loadWrite));
	g_hooks.Add(ControllerLoadHook::writeReturnAddress - 5, loadStore, ControllerLoadHook::writeCallExpected);
}

void FailInstallation(std::string_view reason) noexcept
{
	g_installed.store(false, std::memory_order_release);
	std::uintptr_t physicsSite{}, motionSite{};
	const bool physicsRestored = PhysicsHooks::Restore(&physicsSite);
	const bool motionRestored = g_hooks.Restore(&motionSite);
	try {
		SKSE::log::error("Hook installation failed: {}. Rollback complete={}; motion site={:#x}; "
		                 "physics site={:#x}.",
		                 reason, physicsRestored && motionRestored, motionSite, physicsSite);
	} catch (...) {
		// Logging cannot prevent rollback or change the installation result.
	}
}
} // namespace

bool InstallHooks(std::uintptr_t base, const std::array<std::uintptr_t, 3>& actorTables)
{
	if (g_installed.load(std::memory_order_acquire)) {
		return g_imageBase == base;
	}
	g_imageBase = base;
	SVT::Controller::types = {
	    REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharProxyController[1]}.address(),
	    REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharRigidBodyController[0]}.address(),
	    REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharRigidBodyController[1]}.address(),
	    REL::Relocation<std::uintptr_t>{RE::VTABLE_hkpCharacterMotion[0]}.address()};
	const auto sourceUpdate = REL::Relocation<std::uintptr_t>{RE::Actor::UpdateMovementID}.address();
	const auto sourceConsumer = REL::Relocation<std::uintptr_t>{RE::Actor::ProcessMovementID}.address();
	const auto moveFinishReceiver =
	    REL::Relocation<std::uintptr_t>{RE::Actor::MoveFinishEventHandlerID}.address();
	const auto finishPosition =
	    REL::Relocation<std::uintptr_t>{RE::Actor::SetPositionFromControllerID}.address();
	SkipZHook::g_returnAddress = finishPosition + 0x59;
	const auto consumerCall = sourceUpdate + 0xF0;
	try {
		g_worldToHavok = RE::bhkWorld::GetWorldScale();
		Hooks::RequireBytes(
		    REL::Relocation<std::uintptr_t>{RE::bhkCharacterController::RequestJumpID}.address(),
		    kRequestJumpExpectedBytes, "RequestJump");
		Hooks::RequireBytes(
		    REL::Relocation<std::uintptr_t>{RE::bhkCharacterController::ApplyRequestedStateID}.address(),
		    kApplyRequestedStateExpectedBytes, "ApplyRequestedState");
		Hooks::RequireBytes(SkipZHook::g_returnAddress, kMoveFinishSkipZExpectedBytes,
		                    "MoveFinish skip-Z branch");
		if (std::bit_cast<std::uint32_t>(g_worldToHavok) != 0x3C6A161E) {
			throw std::runtime_error(
			    std::format("world scale mismatch: expectedBits=0x3C6A161E actualBits={:#x}",
			                std::bit_cast<std::uint32_t>(g_worldToHavok)));
		}
		// Preserve the installed CALL target, including earlier hooks, and its five-byte return boundary.
		Hooks::RequireBytes(consumerCall + 5, kMovementContinuationExpectedBytes, "movement continuation");
		SVT::Hooks::StageCall(g_hooks, consumerCall, sourceConsumer, &NativeRootHook::ProcessMovement,
		                      g_processMovementOriginal, true);
		SVT::Hooks::StageCall(g_hooks, sourceConsumer + 0x3FC,
		                      REL::Relocation<std::uintptr_t>{RE::AIProcess::GetAnimationDeltaID}.address(),
		                      &NativeRootHook::GetAnimationDelta, g_animationDeltaOriginal);
		PrepareNativeMove(sourceConsumer + 0xC0D);
		SVT::Hooks::StageCall(
		    g_hooks, REL::Relocation<std::uintptr_t>{RE::Actor::MoveID}.address() + 0x7B5,
		    REL::Relocation<std::uintptr_t>{RE::bhkCharacterController::UpdateMovementID}.address(),
		    &ControllerUpdateHook::Thunk, g_controllerUpdateOriginal);
		SVT::Hooks::StageCall(g_hooks, moveFinishReceiver + 0x42, finishPosition, &MoveFinishHook::Thunk,
		                      g_moveFinishOriginal);
		SVT::Hooks::StageCall(
		    g_hooks, finishPosition + 0x54,
		    REL::Relocation<std::uintptr_t>{RE::BGSFootIkRaycastInterfaceDB::GetSkipZID}.address(),
		    &SkipZHook::Thunk, SkipZHook::g_original);
		const auto actor = actorTables[0];
		const auto character = actorTables[1];
		const auto player = actorTables[2];
		const auto proxy = SVT::Controller::types.proxy;
		PrepareStateRecoveryHooks();
		SVT::Hooks::StageVtable(g_hooks, actor + RE::Actor::MoveSlot * sizeof(void*),
		                        &ActorMoveHook::ActorThunk, g_actorMoveOriginal);
		SVT::Hooks::StageVtable(g_hooks, character + RE::Actor::MoveSlot * sizeof(void*),
		                        &ActorMoveHook::CharacterThunk, g_characterMoveOriginal);
		SVT::Hooks::StageVtable(g_hooks, player + RE::Actor::MoveSlot * sizeof(void*),
		                        &ActorMoveHook::PlayerThunk, g_playerMoveOriginal);
		SVT::Hooks::StageVtable(g_hooks,
		                        proxy + RE::bhkCharacterController::SetLinearVelocitySlot * sizeof(void*),
		                        &ProxyHooks::VelocityThunk, ProxyHooks::g_velocityOriginal);
		SVT::Hooks::StageVtable(g_hooks, proxy + RE::bhkCharacterController::Unk0BSlot * sizeof(void*),
		                        &ProxyHooks::IntegrateThunk, ProxyHooks::g_integrateOriginal);
		if (!g_hooks.Apply()) {
			throw std::runtime_error(std::string(g_hooks.Failure()));
		}
		SVT::PhysicsHooks::Install(g_hooks.Changes());
		g_installed.store(true, std::memory_order_release);
		try {
			SKSE::log::info("Vertical root motion hooks installed.");
		} catch (...) {
		}
	} catch (const std::exception& error) {
		FailInstallation(error.what());
	} catch (...) {
		FailInstallation("unexpected exception during hook installation");
	}
	return g_installed.load(std::memory_order_acquire);
}

} // namespace SVT::MotionHooks

using namespace SVT::MotionHooks;

void SVT::InstallMotion()
{
	static bool attempted{};
	if (attempted) {
		return;
	}
	attempted = true;
	try {
		const auto version = REL::Module::get().version();
		if (version != REL::Version(1, 5, 97, 0)) {
			throw std::runtime_error(
			    std::format("unsupported runtime version: expected=1.5.97.0 actual={}", version.string(".")));
		}
		const auto base = REL::Module::get().base();
		const std::array<std::uintptr_t, 3> actors{
		    REL::Relocation<std::uintptr_t>{RE::VTABLE_Actor[0]}.address(),
		    REL::Relocation<std::uintptr_t>{RE::VTABLE_Character[0]}.address(),
		    REL::Relocation<std::uintptr_t>{RE::VTABLE_PlayerCharacter[0]}.address()};
		SKSE::GetTrampoline().create(4096);
		InstallHooks(base, actors);
	} catch (const std::exception& error) {
		try {
			SKSE::log::error("Unable to initialize hooks: {}", error.what());
		} catch (...) {
		}
	}
}
