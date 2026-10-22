#pragma once

#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "Assembly.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <format>
#include <initializer_list>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <windows.h>

namespace SVT::Hooks
{
struct PreserveLastError
{
	DWORD value{GetLastError()};

	~PreserveLastError()
	{
		SetLastError(value);
	}
};

inline bool Read(std::uintptr_t address, void* out, std::size_t size) noexcept
{
	SIZE_T copied = 0;
	return address &&
	       ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(address), out, size, &copied) &&
	       copied == size;
}

template <class T> T ReadOrDefault(std::uintptr_t address) noexcept
{
	T output{};
	Read(address, &output, sizeof(output));
	return output;
}

bool WriteCode(std::uintptr_t address, std::span<const std::uint8_t> bytes) noexcept;
bool Executable(std::uintptr_t target) noexcept;
void RequireBytes(std::uintptr_t address, std::span<const std::uint8_t> expected, std::string_view operation);

struct Change
{
	std::uintptr_t address{};
	std::vector<std::uint8_t> before, after;
	bool written{};
	DWORD protection{};
};

class HookTransaction
{
  public:
	static constexpr std::size_t kMaxPatchSize = 32;
	void Add(std::uintptr_t address, std::span<const std::uint8_t> before,
	         std::span<const std::uint8_t> after);
	bool Validate() const noexcept;
	bool Apply(bool (*write)(std::uintptr_t, std::span<const std::uint8_t>) noexcept = WriteCode);
	bool Restore(std::uintptr_t* failedAddress = nullptr) noexcept;

	std::span<const Change> Changes() const noexcept
	{
		return changes;
	}

	std::string_view Failure() const noexcept
	{
		return failure;
	}

	static bool Equals(std::uintptr_t address, std::span<const std::uint8_t> expected) noexcept;
	static bool Protection(const Change& change) noexcept;
	static bool RestoreProtection(const Change& change) noexcept;

  private:
	const Change* FirstMismatch() const noexcept;
	std::vector<Change> changes;
	std::string failure;
};

template <class Original> void SaveOriginal(Original& original, std::uintptr_t target)
{
	if constexpr (std::is_pointer_v<Original>) {
		original = reinterpret_cast<Original>(target);
	} else {
		original = target;
	}
}

std::uintptr_t Publish(Xbyak::CodeGenerator& code);

template <class F, class Original>
void StageCall(HookTransaction& transaction, std::uintptr_t site, std::uintptr_t nativeTarget, F wrapper,
               Original& original, bool allowExistingChain = false)
{
	std::array<std::uint8_t, 5> before{};
	const auto expected = Assembly::Call(site, nativeTarget);
	if (!Read(site, before.data(), before.size())) {
		throw std::runtime_error(std::format("cannot read CALL instruction at {:#x}", site));
	}
	if (before[0] != 0xE8) {
		throw std::runtime_error(
		    std::format("CALL opcode mismatch at {:#x}: expected=0xE8 actual={:#x}", site, before[0]));
	}
	std::int32_t displacement{};
	std::memcpy(&displacement, before.data() + 1, sizeof(displacement));
	const auto target = site + 5 + displacement;
	if (!allowExistingChain && before != expected) {
		throw std::runtime_error(std::format("CALL target mismatch at {:#x}: expected={:#x} actual={:#x}",
		                                     site, nativeTarget, target));
	}
	if (!Executable(target)) {
		throw std::runtime_error(
		    std::format("call target {:#x} is not executable; site={:#x}", target, site));
	}
	SaveOriginal(original, target);
	Assembly::Relay relay(reinterpret_cast<std::uintptr_t>(wrapper));
	const auto after = Assembly::Call(site, Publish(relay));
	transaction.Add(site, before, after);
}

std::uintptr_t StageEntry(HookTransaction& transaction, std::uintptr_t site,
                          std::span<const std::uint8_t> prefix, std::uintptr_t wrapper);

template <class F, class Original>
void StageEntry(HookTransaction& transaction, std::uintptr_t site, std::span<const std::uint8_t> prefix,
                F wrapper, Original& original)
{
	SaveOriginal(original, StageEntry(transaction, site, prefix, reinterpret_cast<std::uintptr_t>(wrapper)));
}

template <class F, class Original>
void StageEntry(HookTransaction& transaction, std::uintptr_t site, std::initializer_list<std::uint8_t> prefix,
                F wrapper, Original& original)
{
	StageEntry(transaction, site, std::span<const std::uint8_t>{prefix.begin(), prefix.size()}, wrapper,
	           original);
}

template <class F, class Original>
void StageVtable(HookTransaction& transaction, std::uintptr_t slot, F wrapper, Original& original,
                 std::uintptr_t expected = 0)
{
	std::uintptr_t before{};
	if (!Read(slot, &before, sizeof(before))) {
		throw std::runtime_error(std::format("cannot read vtable slot at {:#x}", slot));
	}
	if (!Executable(before)) {
		throw std::runtime_error(
		    std::format("vtable target {:#x} is not executable; slot={:#x}", before, slot));
	}
	if (expected && before != expected) {
		throw std::runtime_error(std::format(
		    "vtable target check failed at {:#x}: expected={:#x} actual={:#x}", slot, expected, before));
	}
	const auto after = reinterpret_cast<std::uintptr_t>(wrapper);
	SaveOriginal(original, before);
	transaction.Add(slot, {reinterpret_cast<const std::uint8_t*>(&before), sizeof(before)},
	                {reinterpret_cast<const std::uint8_t*>(&after), sizeof(after)});
}
} // namespace SVT::Hooks
