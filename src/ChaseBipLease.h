#pragma once
#include "WindowsConfig.h"
#include <cstdint>
#include <optional>
#include <windows.h>

namespace SVT::ChaseBip
{
struct Ticket
{
	std::uint64_t serial{};
	std::uintptr_t controller{};

	explicit operator bool() const noexcept
	{
		return serial != 0;
	}

	bool operator==(const Ticket&) const = default;
};

// Any later SVT flags write invalidates this pre-native-write ordering snapshot.
struct NativeWriteSnapshot
{
	Ticket ticket{};
	std::uint64_t writeSequence{};

	explicit operator bool() const noexcept
	{
		return static_cast<bool>(ticket);
	}

	bool operator==(const NativeWriteSnapshot&) const = default;
};

// One owner-local lease; each Actor/controller instance keeps its own object.
// Calls serialize only metadata and flag access; no engine callback, allocation
// or logging runs under this lock.
class Lease
{
  public:
	using Read = std::optional<std::uint32_t> (*)(std::uintptr_t, DWORD*) noexcept;
	using Write = void (*)(std::uintptr_t, std::uint32_t) noexcept;

	Lease(std::uint32_t mask, Read read, Write write) noexcept : mask_(mask), read_(read), write_(write)
	{
	}

	Lease(const Lease&) = delete;
	Lease& operator=(const Lease&) = delete;

	// An existing lease is borrowed, so this movement gets no tail-restoration ticket.
	Ticket Begin(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		if (!controller || (current_ && current_.controller == controller)) {
			return {};
		}
		return Acquire(controller);
	}

	// Controller update captures the lease it used, even when borrowed from its outer Actor::Move.
	Ticket Ensure(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		if (!controller) {
			return {};
		}
		if (!current_ || current_.controller != controller) {
			return Acquire(controller);
		}
		const auto flags = read_(controller, nullptr);
		if (!flags) {
			return {};
		}
		UpdateRestoreBit(*flags);
		if ((*flags & mask_) == 0) {
			WriteLeasedFlags(controller, *flags | mask_);
		}
		return current_;
	}

	bool Restore(Ticket ticket) noexcept
	{
		if (!ticket) {
			return false;
		}
		Guard guard(lock_);
		if (ticket != current_) {
			return false;
		}
		return RestoreCurrent();
	}

	bool Active() noexcept
	{
		Guard guard(lock_);
		return static_cast<bool>(current_);
	}

	DWORD RestoreReadError() noexcept
	{
		Guard guard(lock_);
		return current_ ? restoreReadError_ : ERROR_SUCCESS;
	}

	Ticket PendingTicket(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		return current_.controller == controller ? current_ : Ticket{};
	}

	NativeWriteSnapshot PendingNativeWrite(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		return current_ && current_.controller == controller ? NativeWriteSnapshot{current_, writeSequence_}
		                                                     : NativeWriteSnapshot{};
	}

	// The native writer completed; the next successful read supplies the restoration value.
	bool ConfirmNativeWrite(NativeWriteSnapshot snapshot) noexcept
	{
		Guard guard(lock_);
		if (!snapshot || snapshot.ticket != current_ || snapshot.writeSequence != writeSequence_) {
			return false;
		}
		nativeValuePending_ = true;
		return true;
	}

	// After the owning Move exits, release only a completed write with no intervening SVT write.
	bool AcceptNativeWrite(NativeWriteSnapshot snapshot) noexcept
	{
		Guard guard(lock_);
		if (!snapshot || snapshot.ticket != current_ || snapshot.writeSequence != writeSequence_) {
			return false;
		}
		current_ = {};
		nativeValuePending_ = false;
		return true;
	}

	// After the owning Move ends, acknowledge only flags needing no old-value write.
	// A clear supersedes our forced set; a set baseline already matches a set current bit.
	bool ReleaseIfResolved(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		if (!current_) {
			return true;
		}
		if (current_.controller != controller) {
			return false;
		}
		const auto flags = read_(controller, nullptr);
		if (!flags || ((*flags & mask_) != 0 && restoreBits_ == 0)) {
			return false; // A remaining forced bit may now belong to native code.
		}
		current_ = {};
		nativeValuePending_ = false;
		return true;
	}

	// Native destruction ends any remaining restoration obligation without writing flags.
	// Serialize with an in-flight Restore; later stack exits then reject the old ticket.
	void ForgetDestroyedController(std::uintptr_t controller) noexcept
	{
		Guard guard(lock_);
		if (current_.controller == controller) {
			current_ = {};
			nativeValuePending_ = false;
		}
	}

  private:
	struct Guard
	{
		explicit Guard(SRWLOCK& lock) noexcept : lock_(lock)
		{
			AcquireSRWLockExclusive(&lock_);
		}

		~Guard()
		{
			ReleaseSRWLockExclusive(&lock_);
		}

		SRWLOCK& lock_;
	};

	Ticket Acquire(std::uintptr_t controller) noexcept
	{
		const auto flags = read_(controller, nullptr);
		if (!flags || !RestoreCurrent()) {
			return {};
		}
		restoreBits_ = *flags & mask_;
		restoreReadError_ = ERROR_SUCCESS;
		WriteLeasedFlags(controller, *flags | mask_);
		current_ = {++next_, controller};
		return current_;
	}

	bool RestoreCurrent() noexcept
	{
		if (current_) {
			restoreReadError_ = ERROR_SUCCESS;
			const auto flags = read_(current_.controller, &restoreReadError_);
			if (!flags) {
				return false; // Keep the ticket for the existing restoration exits to retry.
			}
			UpdateRestoreBit(*flags);
			WriteLeasedFlags(current_.controller, (*flags & ~mask_) | restoreBits_);
			current_ = {};
		}
		return true;
	}

	void UpdateRestoreBit(std::uint32_t flags) noexcept
	{
		// A clear cannot be SVT's forced set; a confirmed writer also establishes native ownership of 1.
		if (nativeValuePending_ || (flags & mask_) == 0) {
			restoreBits_ = flags & mask_;
		}
		nativeValuePending_ = false;
	}

	void WriteLeasedFlags(std::uintptr_t controller, std::uint32_t flags) noexcept
	{
		nativeValuePending_ = false;
		++writeSequence_; // Ensure can write again without changing the restoration ticket.
		write_(controller, flags);
	}

	SRWLOCK lock_ = SRWLOCK_INIT;
	const std::uint32_t mask_;
	const Read read_;
	const Write write_;
	std::uint64_t next_{};
	std::uint64_t writeSequence_{};
	Ticket current_{};
	std::uint32_t restoreBits_{};
	bool nativeValuePending_{};
	DWORD restoreReadError_{}; // Captured at the failed read, before caller LastError is restored.
};
} // namespace SVT::ChaseBip
