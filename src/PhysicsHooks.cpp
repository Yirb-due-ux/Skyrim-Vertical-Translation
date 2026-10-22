#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "RE/H/hkpCharacterRigidBody.h"
#include "PhysicsHooks.h"
#include "Hooks.h"
#include "RigidBodyMotion.h"
#include "RE/H/hkpRigidMotionUtil.h"
#include "RE/H/hkpSimpleCollisionResponse.h"
#include "RE/B/bhkCharRigidBodyController.h"
#include "RE/H/hkpWorld.h"
#include "RE/B/bhkWorld.h"
#include "RE/B/bhkCharacterController.h"

namespace SVT::PhysicsHooks
{
// Set only during installation; wrappers compare exact native return boundaries.
std::uintptr_t movementReturn{}, controllerPolicyReturn{}, solverPolicyReturnA{}, solverPolicyReturnB{};
using namespace SVT::Hooks;

namespace
{
// Entry and consumer boundaries for the fixed SE 1.5.97 physics path; see NATIVE-TYPES.md.
// clang-format off
constexpr std::array<std::uint8_t, 11> kSetMotionVelocityExpectedBytes{
    0x0F, 0x28, 0x02,                         // read XYZW
    0x0F, 0x29, 0x81, 0xE0, 0x00, 0x00, 0x00, // store linear velocity
    0xC3
};
constexpr std::array<std::uint8_t, 6> kSolverVelocityCallExpectedBytes{
    0xFF, 0x90, 0x80, 0x00, 0x00, 0x00
};
constexpr std::array<std::uint8_t, 3> kContactVelocityCallExpectedBytes{0xFF, 0x50, 0x38};
constexpr std::array<std::uint8_t, 5> kSaveRbxEntryExpected{0x48, 0x89, 0x5C, 0x24, 0x08};
constexpr std::array<std::uint8_t, 6> kSaveEdxEntryExpected{
    0x48, 0x8B, 0xC4,
    0x89, 0x50, 0x10
};
constexpr std::array<std::uint8_t, 5> kSaveR9dEntryExpected{0x44, 0x89, 0x4C, 0x24, 0x20};
constexpr std::array<std::uint8_t, 7> kSaveR8EntryExpected{
    0x48, 0x8B, 0xC4,
    0x4C, 0x89, 0x40, 0x18
};
constexpr std::array<std::uint8_t, 7> kSaveRdxEntryExpected{
    0x48, 0x8B, 0xC4,
    0x48, 0x89, 0x50, 0x10
};
// clang-format on
HookTransaction transaction;
bool movementInstalled{};
// Preserve raw R8B at the getter boundary; bool would normalize it.
using GetPositionFn = void (*)(const RE::bhkCharacterController*, RE::hkVector4*, std::uint8_t);
using MoveFinishSource = RE::bhkCharacterController::MoveFinishEventSource;
using SendMoveFinishFn = REL::Relocation<decltype(&MoveFinishSource::SendEvent)>;
REL::Relocation<decltype(&RE::hkpCharacterRigidBody::SetLinearVelocity)> setCharacterVelocityOriginal;
REL::Relocation<decltype(&RE::bhkWorld::Unk_32)> updateWorldOriginal;
REL::Relocation<decltype(&RE::hkpWorld::StepDeltaTime)> stepDeltaTimeOriginal;
REL::Relocation<decltype(&RE::hkpWorld::StepMultithreaded)> stepMultithreadedOriginal;
decltype(&RE::hkRigidMotionUtilApplyForcesAndBuildAccumulators) buildAccumulatorsOriginal{};
decltype(&RE::hkRigidMotionUtilApplyForcesAndStep) stepMotionOriginal{};
decltype(&RE::hkRigidMotionUtilApplyAccumulators) applyAccumulatorsOriginal{};
GetPositionFn getPositionOriginal{};
REL::Relocation<decltype(&RE::hkpCharacterRigidBodyListener::CharacterCallback)> characterCallbackOriginal;
REL::Relocation<decltype(&RE::hkpMotion::ApproxTransformAt)> approxTransformOriginal;
SendMoveFinishFn sendMoveFinishOriginal{};

void SetCharacterVelocity(RE::hkpCharacterRigidBody* character, const RE::hkVector4& velocity, float seconds)
{
	SVT::RigidBodyZ::Native::VelocityChange input(character, &velocity, seconds);
	setCharacterVelocityOriginal(character, velocity, seconds);
	input.Returned();
}

void UpdateWorld(RE::bhkWorld* world, std::uint32_t flags)
{
	SVT::RigidBodyZ::Native::WorldUpdate update;
	updateWorldOriginal(world, flags);
}

std::uint32_t StepDeltaTime(RE::hkpWorld* world, float seconds)
{
	SVT::RigidBodyZ::Native::WorldStep physics(world);
	const auto result = stepDeltaTimeOriginal(world, seconds);
	physics.Returned(result);
	return result;
}

std::uint32_t StepMultithreaded(RE::hkpWorld* world, RE::hkJobQueue* jobQueue,
                                RE::hkJobThreadPool* threadPool, float seconds)
{
	SVT::RigidBodyZ::Native::WorldStep physics(world);
	const auto result = stepMultithreadedOriginal(world, jobQueue, threadPool, seconds);
	physics.Returned(result);
	return result;
}

RE::hkpVelocityAccumulator* BuildAccumulators(const RE::hkStepInfo& step, RE::hkpMotion* const* entities,
                                              std::int32_t count, std::int32_t offset,
                                              RE::hkpVelocityAccumulator* accumulators)
{
	SVT::RigidBodyZ::Native::MotionIntegration input(&step, entities, count, offset, accumulators,
	                                                 SVT::RigidBodyZ::Native::MotionIntegration::Mode::Build);
	auto* result = buildAccumulatorsOriginal(step, entities, count, offset, accumulators);
	input.Built(result);
	return result;
}

std::int32_t StepMotion(const RE::hkpSolverInfo& info, const RE::hkStepInfo& step, const RE::hkVector4& delta,
                        RE::hkpMotion* const* entities, std::int32_t count, std::int32_t offset)
{
	SVT::RigidBodyZ::Native::MotionIntegration input(&step, entities, count, offset, nullptr,
	                                                 SVT::RigidBodyZ::Native::MotionIntegration::Mode::Free);
	const auto result = stepMotionOriginal(info, step, delta, entities, count, offset);
	input.Exported();
	return result;
}

std::int32_t ApplyAccumulators(const RE::hkpSolverInfo& info, const RE::hkStepInfo& step,
                               const RE::hkpVelocityAccumulator* accumulators, RE::hkpMotion* const* entities,
                               std::int32_t count, std::int32_t offset)
{
	SVT::RigidBodyZ::Native::MotionIntegration input(
	    &step, entities, count, offset, accumulators,
	    SVT::RigidBodyZ::Native::MotionIntegration::Mode::Export);
	const auto result = applyAccumulatorsOriginal(info, step, accumulators, entities, count, offset);
	input.Exported();
	return result;
}

void GetPosition(const RE::bhkCharacterController* controller, RE::hkVector4* position,
                 std::uint8_t centerOffset)
{
	SVT::RigidBodyZ::Native::PositionRead(controller, position, centerOffset,
	                                      SVT::RigidBodyZ::Native::PositionReadStage::Before);
	getPositionOriginal(controller, position, centerOffset);
	SVT::RigidBodyZ::Native::PositionRead(controller, position, centerOffset,
	                                      SVT::RigidBodyZ::Native::PositionReadStage::After);
}

void CharacterCallback(RE::hkpCharacterRigidBodyListener* listener, RE::hkpWorld* world,
                       RE::hkpCharacterRigidBody* character)
{
	SVT::RigidBodyZ::Native::PostSimulation publication(listener, world, character);
	characterCallbackOriginal(listener, world, character);
}

void ApproxTransformAt(RE::hkpMotion* motion, float time, RE::hkTransform& transform)
{
	SVT::RigidBodyZ::Native::RecordSampleTime(time);
	approxTransformOriginal(motion, time, transform);
}

void SendEvent(MoveFinishSource* source, const RE::bhkCharacterMoveFinishEvent* event)
{
	SVT::RigidBodyZ::Native::MoveFinishEvent publication(source, event);
	sendMoveFinishOriginal(source, event);
}

} // namespace

void Install(std::span<const Hooks::Change> sites)
{
	PreserveLastError preserve;
	if (movementInstalled) {
		return;
	}
	if (sites.empty()) {
		throw std::runtime_error("physics installation requires published motion hooks");
	}
	for (const auto& site : sites) {
		if (!site.written) {
			throw std::runtime_error(std::format("motion hook is not published at {:#x}", site.address));
		}
		RequireBytes(site.address, site.after, "motion hook before physics installation");
	}
	decltype(&SetCharacterVelocity) velocityContinuation{};
	StageEntry(transaction,
	           REL::Relocation<std::uintptr_t>{RE::hkpCharacterRigidBody::SetLinearVelocityID}.address(),
	           kSaveRbxEntryExpected, &SetCharacterVelocity, velocityContinuation);
	setCharacterVelocityOriginal = reinterpret_cast<std::uintptr_t>(velocityContinuation);
	StageEntry(transaction, REL::Relocation<std::uintptr_t>{RE::bhkWorld::Unk32ID}.address(),
	           kSaveEdxEntryExpected, &UpdateWorld, updateWorldOriginal);
	StageEntry(transaction, REL::Relocation<std::uintptr_t>{RE::hkpWorld::StepDeltaTimeID}.address(),
	           kSaveRbxEntryExpected, &StepDeltaTime, stepDeltaTimeOriginal);
	StageEntry(transaction, REL::Relocation<std::uintptr_t>{RE::hkpWorld::StepMultithreadedID}.address(),
	           kSaveRbxEntryExpected, &StepMultithreaded, stepMultithreadedOriginal);
	StageEntry(
	    transaction,
	    REL::Relocation<std::uintptr_t>{RE::hkRigidMotionUtilApplyForcesAndBuildAccumulatorsID}.address(),
	    kSaveR9dEntryExpected, &BuildAccumulators, buildAccumulatorsOriginal);
	StageEntry(transaction,
	           REL::Relocation<std::uintptr_t>{RE::hkRigidMotionUtilApplyForcesAndStepID}.address(),
	           kSaveR8EntryExpected, &StepMotion, stepMotionOriginal);
	StageEntry(transaction,
	           REL::Relocation<std::uintptr_t>{RE::hkRigidMotionUtilApplyAccumulatorsID}.address(),
	           kSaveRdxEntryExpected, &ApplyAccumulators, applyAccumulatorsOriginal);
	StageVtable(transaction,
	            REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharRigidBodyController[0]}.address() +
	                RE::bhkCharacterController::GetPositionSlot * sizeof(void*),
	            &GetPosition, getPositionOriginal,
	            REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::GetPositionImplID}.address());
	StageVtable(
	    transaction,
	    REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharRigidBodyController[1]}.address() +
	        RE::hkpCharacterRigidBodyListener::CharacterCallbackSlot * sizeof(void*),
	    &CharacterCallback, characterCallbackOriginal,
	    REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::CharacterCallbackID}.address());
	StageCall(transaction,
	          REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::GetPositionImplID}.address() +
	              0x115,
	          REL::Relocation<std::uintptr_t>{RE::hkpMotion::ApproxTransformAtID}.address(),
	          &ApproxTransformAt, approxTransformOriginal);
	StageCall(transaction,
	          REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::CharacterCallbackID}.address() +
	              0xAB,
	          REL::Relocation<std::uintptr_t>{RE::bhkCharacterController::MoveFinishEventSendID}.address(),
	          &SendEvent, sendMoveFinishOriginal);
	{
		using namespace SVT::RigidBodyZ::Native;
		const auto contactPolicyCall =
		    REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::UpdateContactsID}.address() +
		    0x12F;
		const auto solver =
		    REL::Relocation<std::uintptr_t>{RE::hkpSimpleCollisionResponse::SolveSingleContactID}.address();
		const std::array solverPolicyCalls{solver + 0x385, solver + 0x3AF};
		RequireBytes(REL::Relocation<std::uintptr_t>{RE::hkpMotion::SetLinearVelocityID}.address(),
		             kSetMotionVelocityExpectedBytes, "SetLinearVelocity");
		RequireBytes(solverPolicyCalls[0], kSolverVelocityCallExpectedBytes, "solver velocity call A");
		RequireBytes(solverPolicyCalls[1], kSolverVelocityCallExpectedBytes, "solver velocity call B");
		RequireBytes(contactPolicyCall, kContactVelocityCallExpectedBytes, "contact velocity call");
		const auto worldScaleBits = std::bit_cast<std::uint32_t>(RE::bhkWorld::GetWorldScale());
		if (worldScaleBits != 0x3C6A161E) {
			throw std::runtime_error(std::format(
			    "physics world scale mismatch: expectedBits=0x3C6A161E actualBits={:#x}", worldScaleBits));
		}
		const auto controllerVelocityTarget =
		    REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::SetLinearVelocityImplID}
		        .address();
		const auto controllerVelocityHook =
		    reinterpret_cast<std::uintptr_t>(&SVT::RigidBodyZ::Native::SetControllerVelocity);
		transaction.Add(REL::Relocation<std::uintptr_t>{RE::VTABLE_bhkCharRigidBodyController[0]}.address() +
		                    RE::bhkCharacterController::SetLinearVelocitySlot * sizeof(void*),
		                {reinterpret_cast<const std::uint8_t*>(&controllerVelocityTarget), 8},
		                {reinterpret_cast<const std::uint8_t*>(&controllerVelocityHook), 8});
		SetControllerVelocityOriginal(ControllerVelocity{controllerVelocityTarget});
		const auto motionVelocityTarget =
		    REL::Relocation<std::uintptr_t>{RE::hkpMotion::SetLinearVelocityID}.address();
		const auto motionVelocityHook = reinterpret_cast<std::uintptr_t>(&SetMotionVelocity);
		transaction.Add(REL::Relocation<std::uintptr_t>{RE::VTABLE_hkpCharacterMotion[0]}.address() +
		                    RE::hkpMotion::SetLinearVelocitySlot * sizeof(void*),
		                {reinterpret_cast<const std::uint8_t*>(&motionVelocityTarget), 8},
		                {reinterpret_cast<const std::uint8_t*>(&motionVelocityHook), 8});
		SetMotionVelocityOriginal(MotionVelocityFn{motionVelocityTarget});
		Configure(RE::bhkWorld::GetWorldScale());
		movementReturn =
		    REL::Relocation<std::uintptr_t>{RE::bhkCharRigidBodyController::Unk0BID}.address() + 0x12D;
		controllerPolicyReturn = contactPolicyCall + 3;
		solverPolicyReturnA = solverPolicyCalls[0] + kSolverVelocityCallExpectedBytes.size();
		solverPolicyReturnB = solverPolicyCalls[1] + kSolverVelocityCallExpectedBytes.size();
	}

	if (!transaction.Apply()) {
		throw std::runtime_error(std::string(transaction.Failure()));
	}
	for (const auto& site : sites) {
		RequireBytes(site.address, site.after, "motion hook after physics installation");
	}
	for (const auto& change : transaction.Changes()) {
		RequireBytes(change.address, change.after, "physics hook readback");
	}
	movementInstalled = true;
	SVT::RigidBodyZ::Native::Enable();
}

bool Restore(std::uintptr_t* failedAddress) noexcept
{
	SVT::RigidBodyZ::Native::Disable();
	movementInstalled = false;
	return transaction.Restore(failedAddress);
}

} // namespace SVT::PhysicsHooks

namespace SVT::RigidBodyZ::Native
{
void SetControllerVelocity(RE::bhkCharacterController* controller, const RE::hkVector4& velocity)
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	using namespace SVT::PhysicsHooks;
	const auto source = caller == movementReturn           ? VelocitySource::Movement
	                    : caller == controllerPolicyReturn ? VelocitySource::NativePolicy
	                                                       : VelocitySource::Other;
	SubmitVelocity(controller, velocity, source);
}

void SetMotionVelocity(RE::hkpMotion* motion, const RE::hkVector4& velocity)
{
	const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
	using namespace SVT::PhysicsHooks;
	SubmitMotionVelocity(motion, velocity, caller == solverPolicyReturnA || caller == solverPolicyReturnB);
}
} // namespace SVT::RigidBodyZ::Native
