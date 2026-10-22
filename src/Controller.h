#pragma once

#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "RE/A/AIProcess.h"
#include "RE/B/bhkCharacterController.h"
#include "RE/B/bhkCharacterControllerMovement.h"
#include "RE/H/hkpSimulation.h"
#include "RE/H/hkpVelocityAccumulator.h"
#include "RE/B/bhkCharProxyController.h"
#include "RE/B/bhkCharRigidBodyController.h"
#include "RE/B/bhkRefObject.h"
#include "RE/H/hkpCharacterProxy.h"
#include "RE/H/hkpCharacterRigidBody.h"
#include "RE/H/hkpShapePhantom.h"
#include "RE/H/hkpRigidBody.h"
#include "RE/T/TESObjectREFR.h"

#include "Hooks.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace SVT::Controller
{
static_assert(offsetof(RE::AIProcess, high) == 0x10);
inline constexpr auto kFlags = offsetof(RE::bhkCharacterController, flags);
inline constexpr auto kMotion = offsetof(RE::hkpRigidBody, motion);
inline constexpr auto kChaseBip = static_cast<std::uint32_t>(RE::CHARACTER_FLAGS::kChaseBip);
inline constexpr auto kNoSim = static_cast<std::uint32_t>(RE::CHARACTER_FLAGS::kNoSim);

static_assert(offsetof(RE::bhkCharacterController, context) +
                  offsetof(RE::hkpCharacterContext, currentState) ==
              0x200);
static_assert(offsetof(RE::bhkCharacterController, wantState) == 0x21C && kFlags == 0x218);
static_assert(offsetof(RE::bhkCharacterController, outVelocity) == 0x90);
static_assert(offsetof(RE::bhkCharacterController, stepInfo) + offsetof(RE::hkStepInfo, deltaTime) == 0x88);
static_assert(offsetof(RE::TESObjectREFR, data) + offsetof(RE::OBJ_REFR, location) == 0x54);
static_assert(kChaseBip == (1u << 11) && kNoSim == (1u << 17));
static_assert(sizeof(RE::hkpCharacterStateType) == sizeof(std::uint32_t));
static_assert(sizeof(RE::NiPoint3) == 12);

static_assert(offsetof(RE::bhkCharProxyController, context) - offsetof(RE::bhkCharacterController, context) ==
              0x10);
static_assert(offsetof(RE::bhkCharProxyController, proxy) == 0x340);
static_assert(offsetof(RE::bhkRefObject, referencedObject) == 0x10);
static_assert(sizeof(RE::hkRefPtr<RE::hkReferencedObject>) == sizeof(void*));
static_assert(offsetof(RE::hkpCharacterProxy, shapePhantom) == 0x80);
static_assert(offsetof(RE::hkpShapePhantom, motionState) + offsetof(RE::hkMotionState, transform) +
                  offsetof(RE::hkTransform, translation) ==
              0x120);
static_assert(offsetof(RE::bhkCharRigidBodyController, charRigidBody) +
                  offsetof(RE::bhkRefObject, referencedObject) ==
              0x350);
static_assert(offsetof(RE::hkpCharacterRigidBody, character) == 0x20);
static_assert(offsetof(RE::hkpWorldObject, world) == 0x10);
static_assert(kMotion == 0x150 && kMotion + offsetof(RE::hkpMotion, motionState) == 0x170);
static_assert(kMotion + offsetof(RE::hkpMotion, linearVelocity) == 0x230);

struct Types
{
	std::uintptr_t proxy{}, rigid{}, rigidListener{}, characterMotion{};
};

inline Types types;

inline bool HasType(std::uintptr_t object, std::uintptr_t expected) noexcept
{
	Hooks::PreserveLastError preserve;
	std::uintptr_t vtable{};
	return expected && Hooks::Read(object, &vtable, sizeof(vtable)) && vtable == expected;
}

struct StateSnapshot
{
	RE::hkpCharacterStateType current{}, wanted{};
	std::uint32_t flags{};

	bool AcceptsVelocity() const noexcept
	{
		return current != RE::hkpCharacterStateType::kFlying && (flags & kNoSim) == 0;
	}
};

template <class T> bool Read(const void* address, T& value) noexcept
{
	Hooks::PreserveLastError preserve;
	return Hooks::Read(reinterpret_cast<std::uintptr_t>(address), &value, sizeof(value));
}

inline bool MovementZ(const void* input, float& z) noexcept
{
	const auto* movement = static_cast<const RE::bhkCharacterControllerMovement*>(input);
	return movement && Read(&movement->translation.z, z);
}

struct SimulationTime
{
	float current{}, psi{}, delta{}, marker{};
};

inline bool ReadSimulationTime(const RE::hkpSimulation* simulation, SimulationTime& time) noexcept
{
	// Retain the native contiguous snapshot, including its unused delta-time field.
	static_assert(offsetof(RE::hkpSimulation, simulateUntilTime) - offsetof(RE::hkpSimulation, currentTime) ==
	              12);
	return simulation && Read(&simulation->currentTime, time);
}

inline std::optional<std::uint32_t> ReadFlags(std::uintptr_t controller, DWORD* error = nullptr) noexcept
{
	Hooks::PreserveLastError preserve;
	if (error) {
		*error = ERROR_SUCCESS;
		SetLastError(ERROR_SUCCESS); // A rejected address or short copy must not report a stale error.
	}
	std::uint32_t value{};
	const auto* object = reinterpret_cast<const RE::bhkCharacterController*>(controller);
	if (object) {
		if (Hooks::Read(reinterpret_cast<std::uintptr_t>(&object->flags), &value, sizeof(value))) {
			return value;
		}
		if (error) {
			*error = GetLastError();
		}
	}
	return std::nullopt;
}

inline void WriteFlags(std::uintptr_t controller, std::uint32_t value) noexcept
{
	auto* object = reinterpret_cast<RE::bhkCharacterController*>(controller);
	std::memcpy(&object->flags, &value, sizeof(value));
}

inline std::optional<StateSnapshot> State(const RE::bhkCharacterController* controller) noexcept
{
	StateSnapshot state{};
	if (controller && Read(&controller->context.currentState, state.current) &&
	    Read(&controller->wantState, state.wanted) && Read(&controller->flags, state.flags)) {
		return state;
	}
	return std::nullopt;
}

inline bool ProxyPosition(const RE::bhkCharacterController* controller, std::array<float, 3>& output) noexcept
{
	const auto* object = static_cast<const RE::bhkCharProxyController*>(controller);
	RE::hkpCharacterProxy* proxy{};
	RE::hkpShapePhantom* phantom{};
	// Snapshot only the borrowed pointer; copying hkRefPtr would change its reference count.
	return object && Read(&object->proxy.referencedObject, proxy) && proxy &&
	       Read(&proxy->shapePhantom, phantom) && phantom &&
	       Read(&phantom->motionState.transform.translation, output) && std::isfinite(output[0]) &&
	       std::isfinite(output[1]) && std::isfinite(output[2]);
}

inline bool ReadDeltaTime(const RE::bhkCharacterController* controller, float& seconds) noexcept
{
	return controller && Read(&controller->stepInfo.deltaTime, seconds);
}

inline bool IsOutputVelocity(const RE::bhkCharacterController* controller, const void* velocity) noexcept
{
	return controller && velocity == &controller->outVelocity;
}

} // namespace SVT::Controller
