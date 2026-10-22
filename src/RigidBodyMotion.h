#pragma once
#include "WindowsConfig.h"
#include "RigidBodyZ.h"
#include "SKSE/SKSE.h"
#include "RE/B/bhkCharacterController.h"
#include "RE/H/hkpMotion.h"
#include <memory>
#include <vector>

namespace RE
{
class bhkCharacterController;
class hkVector4;
class hkpCharacterRigidBody;
class hkpCharacterRigidBodyListener;
class hkpMotion;
class hkpWorld;
} // namespace RE

namespace SVT::RigidBodyZ::Native
{
struct Instance;
using Owner = std::shared_ptr<Instance>;
void Configure(float worldScale) noexcept;
void Enable() noexcept;
void Disable() noexcept;
// Revoke old load generations; WorldStep and callback owners retain borrowed storage.
void RetirePreviousLifetimes() noexcept;
void RetireActor(std::uintptr_t actor) noexcept;
void RetireController(std::uintptr_t controller) noexcept;
// Ending the native synchronized source stops new input; consumed physics may still publish.
void SetSource(std::uintptr_t actor, std::uint32_t form, std::uint32_t handle, bool active, float authoredDz,
               Id lifetime = ReferenceFrameZ::CurrentLifetime(), const std::atomic<bool>* cancelled = nullptr,
               Id sourceSerial = 0) noexcept;

// Movement shadows nested calls and restores the borrowed outer context on return.
class Movement
{
  public:
	Movement(const Movement&) = delete;
	Movement& operator=(const Movement&) = delete;
	Movement(std::uintptr_t actor, std::uint32_t form, std::uint32_t handle,
	         std::uintptr_t controller) noexcept;
	~Movement();

  private:
	Owner previousOwner{};
	Id previous{}, previousEpoch{};
};

class ControllerUpdate
{
  public:
	ControllerUpdate(const ControllerUpdate&) = delete;
	ControllerUpdate& operator=(const ControllerUpdate&) = delete;
	ControllerUpdate(std::uintptr_t controller, const void* info) noexcept;
	~ControllerUpdate();

  private:
	const void* previous{};
	bool previousEligible{};
	bool previousSubmitted{};
	Id previousEpoch{};
};

using ControllerVelocity = REL::Relocation<decltype(&RE::bhkCharacterController::SetLinearVelocityImpl)>;
using MotionVelocityFn = REL::Relocation<decltype(&RE::hkpMotion::SetLinearVelocity)>;
enum class VelocitySource
{
	Other,
	Movement,
	NativePolicy
};
__declspec(noinline) void SetControllerVelocity(RE::bhkCharacterController* controller,
                                                const RE::hkVector4& velocity);
void SubmitVelocity(RE::bhkCharacterController* controller, const RE::hkVector4& velocity,
                    VelocitySource source);
void SetControllerVelocityOriginal(ControllerVelocity original) noexcept;
void SetMotionVelocityOriginal(MotionVelocityFn original) noexcept;
__declspec(noinline) void SetMotionVelocity(RE::hkpMotion* motion, const RE::hkVector4& velocity);
void SubmitMotionVelocity(RE::hkpMotion* motion, const RE::hkVector4& velocity, bool nativePolicy);

// Match a controller submission to the actual native setter result; constructor alone grants no output.
class VelocityChange
{
  public:
	VelocityChange(const VelocityChange&) = delete;
	VelocityChange& operator=(const VelocityChange&) = delete;
	VelocityChange(void* character, const void* velocity, float dt) noexcept;
	~VelocityChange();
	void Returned() noexcept;

  private:
	Owner owner{};
	Id epoch{};
	Vector before{};
	Origin origin{};
	Instance* previousSetter{};
	bool policy{}, returned{};
};

// Own operation storage until the native world step returns, including joined worker callbacks.
class WorldStep
{
  public:
	WorldStep(const WorldStep&) = delete;
	WorldStep& operator=(const WorldStep&) = delete;
	WorldStep(void* world) noexcept;
	~WorldStep();
	void Returned(std::uint32_t result) noexcept;

  private:
	struct Item
	{
		Owner owner;
		Operation operation;
	};

	std::vector<Item> items;
	bool returned{};
};

class WorldUpdate
{
  public:
	WorldUpdate(const WorldUpdate&) = delete;
	WorldUpdate& operator=(const WorldUpdate&) = delete;
	WorldUpdate() noexcept;
	~WorldUpdate();
};

class MotionIntegration
{
  public:
	MotionIntegration(const MotionIntegration&) = delete;
	MotionIntegration& operator=(const MotionIntegration&) = delete;
	enum class Mode
	{
		Build,
		Export,
		Free
	};
	MotionIntegration(const void* step, const void* entities, std::int32_t count, std::int32_t offset,
	                  const void* accumulator, Mode mode) noexcept;
	void Built(void* returnedEnd) noexcept;
	void Exported() noexcept;

  private:
	struct Item
	{
		Owner owner;
		Input input;
		Id epoch{};
		Operation* operation{};
	};

	std::vector<Item> items;
	std::uintptr_t expectedEnd{};
};

struct PositionReadState
{
	Owner owner{};
	Id epoch{};
	Publication publication{};
	Vector getter{};
	bool reading{}, sampled{}, valid{};
	float sample{};
};

// Borrow only this callback's sampled position; source end does not erase already consumed physics.
class PostSimulation
{
  public:
	PostSimulation(const PostSimulation&) = delete;
	PostSimulation& operator=(const PostSimulation&) = delete;
	PostSimulation(RE::hkpCharacterRigidBodyListener* listener, RE::hkpWorld* world,
	               RE::hkpCharacterRigidBody* character) noexcept;
	~PostSimulation();

  private:
	PositionReadState state{};
	PositionReadState* previous{};
};

enum class PositionReadStage
{
	Before,
	After
};
void PositionRead(const RE::bhkCharacterController* controller, RE::hkVector4* position,
                  std::uint8_t centerOffset, PositionReadStage stage) noexcept;
void RecordSampleTime(float time) noexcept;

struct MoveFinishState
{
	Owner owner{};
	Id epoch{};
	Publication publication{};
	std::uintptr_t actor{};
	std::array<float, 3> position{};
	const void* finishPosition{};
	bool qualified{};
	unsigned finishDepth{};
};

class MoveFinishEvent
{
  public:
	MoveFinishEvent(const MoveFinishEvent&) = delete;
	MoveFinishEvent& operator=(const MoveFinishEvent&) = delete;
	MoveFinishEvent(void* source, const void* position) noexcept;
	~MoveFinishEvent();

  private:
	MoveFinishState state{};
	MoveFinishState* previous{};
};

class ActorWriteback
{
  public:
	ActorWriteback(const ActorWriteback&) = delete;
	ActorWriteback& operator=(const ActorWriteback&) = delete;
	ActorWriteback(std::uintptr_t actor, const void* position) noexcept;
	~ActorWriteback();

  private:
	MoveFinishState* owner{};
	std::uintptr_t actor{};
	const void* position{};
	unsigned previousDepth{};
};

bool CanOverrideSkipZ(std::uintptr_t actor, const void* position, bool skipZCaller, bool found) noexcept;
} // namespace SVT::RigidBodyZ::Native
