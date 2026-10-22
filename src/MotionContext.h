#pragma once

#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "ChaseBipLease.h"
#include "Controller.h"
#include "CorrectedZCommit.h"
#include "ReferenceFrameZ.h"
#include <memory>
#include <mutex>

namespace SVT
{
struct ProxyInstance
{
	std::uintptr_t actor{}, controller{};
	std::uint32_t handle{};
	bool busy{}, retirementReported{};
	std::atomic<bool> retired{};
	std::uint64_t lifetime{SVT::ReferenceFrameZ::CurrentLifetime()};
	std::uint64_t sourceSerial{}; // Last admitted consumer; an older callback cannot end a newer binding.
	SVT::ReferenceFrameZ::Phase source{};
	SVT::ChaseBip::Lease lease{Controller::kChaseBip, &Controller::ReadFlags, &Controller::WriteFlags};
	// Protected by the proxy lock; confirmation waits for the owning Move's final restoration exit.
	SVT::ChaseBip::NativeWriteSnapshot completedNativeWrite{};
};

using ProxyOwner = std::shared_ptr<ProxyInstance>;
using ReleaseProxy = void (*)(const ProxyOwner&) noexcept;

// Borrowed native calls never outlive their stack. Persistent phases stay with
// the Actor/controller instance; rigid physical results have their own owner.
class MotionContext
{
  public:
	class Consumer;
	class Dispatch;
	class Movement;
	class ControllerUpdate;
	class Finish;
	class ActorUnload;
	using EndSource = void (*)(RE::Actor*, std::uint64_t, std::uint64_t) noexcept;

  private:
	inline static thread_local Consumer* consumer{};
	inline static thread_local Dispatch* dispatch{};
	inline static thread_local Movement* movement{};
	inline static thread_local ControllerUpdate* controllerUpdate{};
	inline static thread_local Finish* finish{};
	// Stack-owned source registrations allow teardown to cancel another thread's
	// qualification without touching its TLS or keeping an engine object alive.
	inline static std::mutex sourcesLock;
	inline static Consumer* activeSources{};
	inline static ActorUnload* unloads{};

  public:
	class ActorUnload
	{
		friend class Consumer;
		RE::Actor* actor{};
		ActorUnload* next{};

	  public:
		explicit ActorUnload(RE::Actor* target) noexcept;
		~ActorUnload();
		ActorUnload(const ActorUnload&) = delete;
		ActorUnload& operator=(const ActorUnload&) = delete;
	};

	class Consumer
	{
		friend class Dispatch;
		friend class Movement;
		friend class ActorUnload;
		SVT::ReferenceFrameZ::Consumer state{};
		Consumer* previous{};
		EndSource end{};
		bool returned{};
		const std::uint64_t lifetime{SVT::ReferenceFrameZ::CurrentLifetime()};
		std::atomic<bool> cancelled{};
		Consumer* older{};
		Consumer* newer{};

	  public:
		Consumer(RE::Actor* actor, std::uintptr_t process, std::uint64_t serial, EndSource onEnd) noexcept
		    : previous(consumer), end(onEnd)
		{
			state.actor = reinterpret_cast<std::uintptr_t>(actor);
			state.process = process;
			state.serial = serial;
			{
				Hooks::PreserveLastError preserve;
				std::lock_guard lock(sourcesLock);
				for (auto* unload = unloads; unload; unload = unload->next) {
					if (unload->actor == actor) {
						cancelled = true;
					}
				}
				older = activeSources;
				if (older) {
					older->newer = this;
				}
				activeSources = this;
			}
			consumer = this;
		}

		Consumer(const Consumer&) = delete;
		Consumer& operator=(const Consumer&) = delete;

		~Consumer()
		{
			Hooks::PreserveLastError preserve;
			consumer = previous;
			std::lock_guard lock(sourcesLock);
			// End callbacks only mutate plugin indexes. Keep cancellation and source
			// end ordered against ActorUnload before unlinking this stack registration.
			if ((!returned || !state.dispatched) && !cancelled &&
			    SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime)) {
				end(Actor(), lifetime, state.serial);
			}
			if (newer) {
				newer->older = older;
			} else {
				activeSources = older;
			}
			if (older) {
				older->newer = newer;
			}
		}

		static Consumer* ForProcess(void* process) noexcept
		{
			return consumer && consumer->state.process == reinterpret_cast<std::uintptr_t>(process) ? consumer
			                                                                                        : nullptr;
		}

		RE::Actor* Actor() const noexcept
		{
			return reinterpret_cast<RE::Actor*>(state.actor);
		}

		void Accept(bool eligible, std::uint32_t handle, bool validPosition) noexcept
		{
			state.eligible = eligible && !cancelled && SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime);
			state.handle = handle;
			state.Read(state.process, validPosition);
		}

		void Returned() noexcept
		{
			returned = true;
		}
	};

	class Dispatch
	{
		friend class Movement;
		RE::Actor* actor{};
		std::uint32_t handle{};
		bool root{}, claimed{};
		std::uint64_t lifetime{};
		std::uint64_t sourceSerial{};
		Consumer* source{};
		Dispatch* previous{};
		EndSource end{};

	  public:
		Dispatch(RE::Actor* target, std::uintptr_t process, float seconds, const RE::NiPoint3& delta,
		         bool installed, EndSource onEnd) noexcept
		    : actor(target), previous(dispatch), end(onEnd)
		{
			lifetime = consumer ? consumer->lifetime : 0;
			source = consumer;
			sourceSerial = consumer ? consumer->state.serial : 0;
			root = installed && consumer && !consumer->cancelled && target &&
			       SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime) && consumer->state.process == process &&
			       consumer->state.CanDispatch(reinterpret_cast<std::uintptr_t>(target), seconds,
			                                   {delta.x, delta.y, delta.z});
			if (root) {
				handle = consumer->state.handle;
				consumer->state.dispatched = true;
			}
			dispatch = this;
		}

		Dispatch(const Dispatch&) = delete;
		Dispatch& operator=(const Dispatch&) = delete;

		~Dispatch()
		{
			Hooks::PreserveLastError preserve;
			dispatch = previous;
			std::lock_guard lock(sourcesLock);
			if (!claimed && (!source || !source->cancelled) &&
			    SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime)) {
				end(actor, lifetime, sourceSerial);
			}
		}
	};

	class Movement
	{
		friend class MotionContext;
		friend class ControllerUpdate;
		friend class Finish;
		Movement* previous{};
		RE::Actor* actor{};
		RE::bhkCharacterController* controller{};
		RE::NiPoint3 delta{};
		ProxyOwner owner{};
		std::mutex& proxyLock;
		ReleaseProxy release{};
		const std::uint64_t lifetime;
		Consumer* source{}; // Borrowed from the enclosing native consumer, never stored in an instance.
		std::uint64_t sourceSerial{};
		SVT::ChaseBip::Ticket lease{};
		SVT::CorrectedZ::Movement identity{};
		std::uint32_t handle{};
		bool root{};

	  public:
		Movement(RE::Actor* target, std::mutex& lock, ReleaseProxy onRelease = nullptr) noexcept
		    : previous(movement), actor(target), proxyLock(lock), release(onRelease),
		      lifetime(dispatch && dispatch->actor == target ? dispatch->lifetime
		                                                     : SVT::ReferenceFrameZ::CurrentLifetime())
		{
			source = dispatch && dispatch->actor == target ? dispatch->source : nullptr;
			sourceSerial = dispatch && dispatch->actor == target ? dispatch->sourceSerial : 0;
			movement = this; // Shadow before the native controller getter can reenter.
		}

		bool Claim(float seconds, const RE::NiPoint3& input, bool installed) noexcept
		{
			delta = input;
			root = installed && dispatch && dispatch->actor == actor && dispatch->root && CurrentSource() &&
			       dispatch->lifetime == lifetime && !dispatch->claimed && std::isfinite(seconds) &&
			       seconds > 0 && SVT::ReferenceFrameZ::FiniteXYZ({input.x, input.y, input.z});
			if (root) {
				dispatch->claimed = true;
				handle = dispatch->handle;
			}
			return root;
		}

		Movement(const Movement&) = delete;
		Movement& operator=(const Movement&) = delete;

		~Movement()
		{
			RestoreLease();
			if (owner && release) {
				release(owner);
			} else if (owner) {
				std::lock_guard lock(proxyLock);
				owner->busy = false;
			}
			movement = previous;
		}

		bool IsRoot() const noexcept
		{
			return root;
		}

		std::uint64_t Lifetime() const noexcept
		{
			return lifetime;
		}

		bool CurrentLifetime() const noexcept
		{
			return SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime);
		}

		std::uint64_t SourceSerial() const noexcept
		{
			return sourceSerial;
		}

		bool Available() const noexcept
		{
			return CurrentSource() && owner && owner->lifetime == lifetime && !owner->retired.load();
		}

		const std::atomic<bool>* Cancellation() const noexcept
		{
			return source ? &source->cancelled : nullptr;
		}

		bool CurrentSource() const noexcept
		{
			return CurrentLifetime() && (!source || !source->cancelled);
		}

		std::uint32_t Handle() const noexcept
		{
			return handle;
		}

		bool ProtectsParent() const noexcept
		{
			return previous && previous->owner && previous->actor == actor;
		}

		void Bind(const ProxyOwner& proxy, RE::bhkCharacterController* control, std::uint64_t serial) noexcept
		{
			owner = proxy;
			controller = control;
			identity = {reinterpret_cast<std::uintptr_t>(actor), reinterpret_cast<std::uintptr_t>(control),
			            proxy->source.episode, serial};
			bool ownsZ{};
			{
				std::lock_guard lock(proxyLock);
				ownsZ = Available() && owner->source.ClaimZ(delta.z);
				if (ownsZ) {
					lease = owner->lease.Begin(owner->controller);
				}
			}
		}

		void RestoreLease() noexcept
		{
			if (owner) {
				(void)owner->lease.Restore(
				    lease); // An unreadable flags field retains the ticket for later exits.
			}
		}
	};

	class ControllerUpdate
	{
		ControllerUpdate* previous{};
		Movement* move{};
		const RE::bhkCharacterControllerMovement* info{};
		SVT::CorrectedZ::Correction correction{};
		SVT::ChaseBip::Ticket lease{};
		bool closed{};

	  public:
		ControllerUpdate(RE::bhkCharacterController* controller,
		                 const RE::bhkCharacterControllerMovement* input, bool concreteProxy) noexcept
		    : previous(controllerUpdate), move(movement)
		{
			if (move) {
				std::lock_guard lock(move->proxyLock);
				if (move->Available() && move->owner->source.ownsZ && move->controller == controller &&
				    concreteProxy) {
					lease = move->owner->lease.Ensure(move->owner->controller);
					if (lease) {
						move->lease = lease;
						info = input;
						correction.movement = move->identity;
					}
				}
			}
			controllerUpdate = this;
		}

		ControllerUpdate(const ControllerUpdate&) = delete;
		ControllerUpdate& operator=(const ControllerUpdate&) = delete;

		~ControllerUpdate()
		{
			Close();
		}

		void Close() noexcept
		{
			if (!closed) {
				controllerUpdate = previous;
				if (move && move->owner) {
					(void)move->owner->lease.Restore(
					    lease); // Movement still owns the fallback restoration ticket.
				}
				closed = true;
			}
		}

		static ControllerUpdate* For(RE::bhkCharacterController* controller, bool concreteProxy) noexcept
		{
			return controllerUpdate && controllerUpdate->info && controllerUpdate->lease &&
			               controllerUpdate->move == movement && movement && movement->Available() &&
			               movement->controller == controller && concreteProxy
			           ? controllerUpdate
			           : nullptr;
		}

		const RE::bhkCharacterControllerMovement* Input() const noexcept
		{
			return info;
		}

		SVT::CorrectedZ::Correction& Result() noexcept
		{
			return correction;
		}

		bool CanCommit(RE::TESObjectREFR* actor, const RE::NiPoint3* position,
		               const std::array<float, 3>& proxyNow) const noexcept
		{
			return actor && position && move && move == movement && move->Available() &&
			       SVT::CorrectedZ::CanCommit(correction, move->identity, move->actor == actor, move->delta.z,
			                                  position->z, proxyNow);
		}
	};

	class Finish
	{
		Finish* previous{};
		RE::TESObjectREFR* actor{};
		const RE::NiPoint3* position{};
		bool eligible{};
		ProxyOwner owner{};
		const std::uint64_t lifetime{SVT::ReferenceFrameZ::CurrentLifetime()};

	  public:
		Finish(RE::TESObjectREFR* target, const RE::NiPoint3* point, bool proxyEligible) noexcept
		    : previous(finish), actor(target), position(point), eligible(proxyEligible)
		{
			if (eligible && movement) {
				owner = movement->owner;
			}
			finish = this;
		}

		Finish(const Finish&) = delete;
		Finish& operator=(const Finish&) = delete;

		~Finish()
		{
			finish = previous;
		}

		static Finish* Current() noexcept
		{
			return finish;
		}

		bool Owns(RE::TESObjectREFR* target) const noexcept
		{
			return actor == target && eligible && (!owner || !owner->retired) &&
			       SVT::ReferenceFrameZ::IsCurrentLifetime(lifetime);
		}

		const RE::NiPoint3* Position() const noexcept
		{
			return position;
		}
	};

	static bool IsMoving(RE::Actor* actor = nullptr) noexcept
	{
		return movement && movement->owner && (!actor || movement->actor == actor);
	}

	static RE::bhkCharacterController* Controller() noexcept
	{
		return movement ? movement->controller : nullptr;
	}

	static bool Idle() noexcept
	{
		return !consumer && !dispatch && !movement && !controllerUpdate && !finish;
	}
};

inline MotionContext::ActorUnload::ActorUnload(RE::Actor* target) noexcept : actor(target)
{
	Hooks::PreserveLastError preserve;
	std::lock_guard lock(sourcesLock);
	next = unloads;
	unloads = this;
	for (auto* source = activeSources; source; source = source->older) {
		if (source->Actor() == actor) {
			source->cancelled = true;
		}
	}
}

inline MotionContext::ActorUnload::~ActorUnload()
{
	Hooks::PreserveLastError preserve;
	std::lock_guard lock(sourcesLock);
	auto** link = &unloads;
	while (*link && *link != this) {
		link = &(*link)->next;
	}
	if (*link) {
		*link = next;
	}
}
} // namespace SVT
