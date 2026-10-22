#pragma once
#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "RE/A/Actor.h"
#include "RE/A/AIProcess.h"
#include "RE/B/bhkCharacterController.h"
#include "RE/B/BGSLoadGameBuffer.h"
#include "RE/B/BGSFootIkRaycastInterfaceDB.h"
#include <array>
#include <atomic>
#include <cstdint>

namespace SVT::MotionHooks
{
using ActorMoveFn = REL::Relocation<decltype(&RE::Actor::Move)>;
using ControllerUpdateFn = REL::Relocation<decltype(&RE::bhkCharacterController::UpdateMovement)>;
using SkipZFn = REL::Relocation<decltype(&RE::BGSFootIkRaycastInterfaceDB::GetSkipZ)>;
using MoveFinishFn = REL::Relocation<decltype(&RE::Actor::SetPositionFromController)>;
using ProcessMovementFn = REL::Relocation<decltype(&RE::Actor::ProcessMovement)>;
using AnimationDeltaFn = REL::Relocation<decltype(&RE::AIProcess::GetAnimationDelta)>;
extern ActorMoveFn g_actorMoveOriginal, g_characterMoveOriginal, g_playerMoveOriginal;
extern ControllerUpdateFn g_controllerUpdateOriginal;
extern MoveFinishFn g_moveFinishOriginal;
extern ProcessMovementFn g_processMovementOriginal;
extern AnimationDeltaFn g_animationDeltaOriginal;
extern std::atomic<bool> g_installed;
extern float g_worldToHavok;
void OnGameMessage(std::uint32_t type) noexcept;

struct Actor3DHook
{
	using Function = void (*)(RE::Actor*, RE::NiAVObject*, std::uint8_t);
	static inline Function original{};
	static void Thunk(RE::Actor* actor, RE::NiAVObject* object, std::uint8_t queueTasks);
};

struct ControllerDestroyHook
{
	using Function = void (*)(RE::bhkCharacterController*);
	static inline Function original{};
	static void Thunk(RE::bhkCharacterController* controller);
};

struct ChaseBipHook
{
	using SetFn = void (*)(RE::Actor*, std::uint8_t);
	using ControllerFn = REL::Relocation<decltype(&RE::AIProcess::GetCharController)>;
	static inline SetFn original{};
	static inline ControllerFn getControllerOriginal{};
	static inline std::uintptr_t getterReturnAddress{};
	static void Thunk(RE::Actor* actor, std::uint8_t enabled);
	__declspec(noinline) static RE::bhkCharacterController* GetControllerThunk(RE::AIProcess* process);
};

struct PlayerDeathHook
{
	using KillFn = void (*)(RE::Actor*, RE::Actor*, float, std::uint8_t, std::uint8_t);
	static inline KillFn original{};
	static inline ChaseBipHook::ControllerFn getControllerOriginal{};
	static inline std::uintptr_t getterReturnAddress{}, writeReturnAddress{};
	static inline std::array<std::uint8_t, 10> writeCallExpected{};
	__declspec(noinline) static void ConfirmWrite(RE::bhkCharacterController* controller) noexcept;
	static void Thunk(RE::Actor* actor, RE::Actor* attacker, float damage, std::uint8_t sendEvent,
	                  std::uint8_t ragdollInstant);
	__declspec(noinline) static RE::bhkCharacterController* GetControllerThunk(RE::AIProcess* process);
};

struct ControllerLoadHook
{
	using LoadFn = void (*)(RE::AIProcess*, RE::BGSLoadGameBuffer*);
	static inline LoadFn original{};
	static inline std::uintptr_t writeReturnAddress{};
	static inline std::array<std::uint8_t, 6> writeCallExpected{};
	static void Thunk(RE::AIProcess* process, RE::BGSLoadGameBuffer* buffer);
	__declspec(noinline) static std::uint32_t ConfirmWrite(RE::bhkCharacterController* controller,
	                                                       std::uint32_t flags) noexcept;
};

struct NativeRootHook
{
	static void ProcessMovement(RE::Actor* actor, float seconds);
	static const RE::NiPoint3* GetAnimationDelta(RE::AIProcess* process);
	static RE::bhkCharacterController* DispatchMove(RE::Actor* actor, float seconds,
	                                                const RE::NiPoint3& delta, std::uintptr_t target);
};

struct ActorMoveHook
{
	static RE::bhkCharacterController* Call(ActorMoveFn& original, RE::Actor* actor, float seconds,
	                                        const RE::NiPoint3& delta);
	static RE::bhkCharacterController* ActorThunk(RE::Actor* actor, float seconds, const RE::NiPoint3& delta);
	static RE::bhkCharacterController* CharacterThunk(RE::Actor* actor, float seconds,
	                                                  const RE::NiPoint3& delta);
	static RE::bhkCharacterController* PlayerThunk(RE::Actor* actor, float seconds,
	                                               const RE::NiPoint3& delta);
};

struct ControllerUpdateHook
{
	static std::uint32_t Thunk(RE::bhkCharacterController* controller,
	                           RE::bhkCharacterControllerMovement* data);
};

struct ProxyHooks
{
	using VelocityFn = REL::Relocation<decltype(&RE::bhkCharacterController::SetLinearVelocityImpl)>;
	using IntegrateFn = bool (*)(RE::bhkCharacterController*);
	static inline VelocityFn g_velocityOriginal{};
	static inline IntegrateFn g_integrateOriginal{};
	static void VelocityThunk(RE::bhkCharacterController* controller, const RE::hkVector4& velocity);
	static bool IntegrateThunk(RE::bhkCharacterController* controller);
};

struct MoveFinishHook
{
	static void Thunk(RE::Actor* actor, const RE::NiPoint3& position);
};

struct SkipZHook
{
	static inline SkipZFn g_original{};
	static inline std::uintptr_t g_returnAddress{};
	static __declspec(noinline) bool Thunk(const RE::BGSFootIkRaycastInterfaceDB* context,
	                                       RE::TESObjectREFR* actor, std::uint8_t* output);
	static void PreserveZ(RE::TESObjectREFR* actor, std::uint8_t* output, bool result, bool sameCallsite);
};

bool InstallHooks(std::uintptr_t base, const std::array<std::uintptr_t, 3>& actorTables);
} // namespace SVT::MotionHooks
