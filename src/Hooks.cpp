#include "Hooks.h"
#include "SKSE/SKSE.h"
#include <safetyhook/inline_hook.hpp>
#include <algorithm>
#include <memory>

namespace SVT::Hooks
{

bool WriteCode(std::uintptr_t address, std::span<const std::uint8_t> bytes) noexcept
{
	DWORD old = 0, ignored = 0;
	if (!VirtualProtect(reinterpret_cast<void*>(address), bytes.size(), PAGE_EXECUTE_READWRITE, &old)) {
		return false;
	}
	std::memcpy(reinterpret_cast<void*>(address), bytes.data(), bytes.size());
	const auto protectedAgain = VirtualProtect(reinterpret_cast<void*>(address), bytes.size(), old, &ignored);
	const auto protectionError = protectedAgain ? ERROR_SUCCESS : GetLastError();
	const auto flushed =
	    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(address), bytes.size());
	if (!protectedAgain) {
		SetLastError(protectionError);
	}
	return protectedAgain && flushed;
}

bool Executable(std::uintptr_t target) noexcept
{
	MEMORY_BASIC_INFORMATION info{};
	return target && VirtualQuery(reinterpret_cast<void*>(target), &info, sizeof(info)) == sizeof(info) &&
	       info.State == MEM_COMMIT && (info.Protect & (PAGE_GUARD | PAGE_NOACCESS)) == 0 &&
	       (info.Protect &
	        (PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY)) != 0;
}

std::uintptr_t Publish(Xbyak::CodeGenerator& code)
{
	const auto block = SKSE::GetTrampoline().allocate(code);
	if (!FlushInstructionCache(GetCurrentProcess(), block, code.getSize())) {
		throw std::runtime_error(std::format("relay cache flush failed at {:#x}; win32={}",
		                                     reinterpret_cast<std::uintptr_t>(block), GetLastError()));
	}
	return reinterpret_cast<std::uintptr_t>(block);
}

void RequireBytes(std::uintptr_t address, std::span<const std::uint8_t> expected, std::string_view operation)
{
	std::array<std::uint8_t, HookTransaction::kMaxPatchSize> actual{};
	if (expected.empty() || expected.size() > actual.size()) {
		throw std::runtime_error(std::format("{}: unsupported verification length {} at {:#x}", operation,
		                                     expected.size(), address));
	}
	if (!address) {
		throw std::runtime_error(std::format("{}: null byte-check address", operation));
	}
	if (!Read(address, actual.data(), expected.size())) {
		const auto error = GetLastError();
		throw std::runtime_error(std::format("{}: cannot read {} bytes at {:#x}; win32={}", operation,
		                                     expected.size(), address, error));
	}
	if (!std::equal(expected.begin(), expected.end(), actual.begin())) {
		throw std::runtime_error(std::format("{}: byte mismatch at {:#x}", operation, address));
	}
}

void HookTransaction::Add(std::uintptr_t address, std::span<const std::uint8_t> before,
                          std::span<const std::uint8_t> after)
{
	if (before.empty() || before.size() > kMaxPatchSize || before.size() != after.size()) {
		throw std::runtime_error(
		    std::format("invalid patch length at {:#x}: before={} after={} supported=1..{}", address,
		                before.size(), after.size(), kMaxPatchSize));
	}
	MEMORY_BASIC_INFORMATION info{};
	if (VirtualQuery(reinterpret_cast<void*>(address), &info, sizeof(info)) != sizeof(info)) {
		const auto error = GetLastError();
		throw std::runtime_error(std::format("cannot query patch region at {:#x}; win32={}", address, error));
	}
	const auto region = reinterpret_cast<std::uintptr_t>(info.BaseAddress);
	if (before.size() > info.RegionSize || address - region > info.RegionSize - before.size()) {
		throw std::runtime_error(
		    std::format("patch at {:#x} crosses a memory region: length={}", address, before.size()));
	}
	changes.push_back(
	    {address, {before.begin(), before.end()}, {after.begin(), after.end()}, false, info.Protect});
}

const Change* HookTransaction::FirstMismatch() const noexcept
{
	for (const auto& change : changes) {
		if (!Equals(change.address, change.before)) {
			return &change;
		}
	}
	return nullptr;
}

bool HookTransaction::Validate() const noexcept
{
	return FirstMismatch() == nullptr;
}

bool HookTransaction::Apply(bool (*write)(std::uintptr_t, std::span<const std::uint8_t>) noexcept)
{
	failure.clear();
	if (const auto* change = FirstMismatch()) {
		failure = std::format("patch precondition check failed at {:#x}", change->address);
		return false;
	}
	for (auto& change : changes) {
		// A failed protection restore or cache flush can follow a completed write.
		change.written = true;
		const char* operation{};
		DWORD writeError{};
		if (!write(change.address, change.after)) {
			operation = "write";
			writeError = write == WriteCode ? GetLastError() : ERROR_SUCCESS;
		} else if (!Equals(change.address, change.after)) {
			operation = "readback";
		} else if (!Protection(change)) {
			operation = "protection check";
		}
		if (operation) {
			std::uintptr_t rollbackSite{};
			const auto restored = Restore(&rollbackSite);
			failure = std::format("patch {} failed at {:#x}; initialRollbackComplete={} rollbackSite={:#x}",
			                      operation, change.address, restored, rollbackSite);
			if (writeError) {
				failure += std::format(" win32={}", writeError);
			}
			return false;
		}
	}
	return true;
}

bool HookTransaction::Restore(std::uintptr_t* failedAddress) noexcept
{
	bool ok = true;
	if (failedAddress) {
		*failedAddress = 0;
	}
	for (auto it = changes.rbegin(); it != changes.rend(); ++it) {
		if (!it->written) {
			continue;
		}
		if (Equals(it->address, it->before)) {
			// A prior write may have restored the bytes but failed to flush the instruction cache.
			if (RestoreProtection(*it) &&
			    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(it->address),
			                          it->before.size())) {
				it->written = false;
			} else {
				ok = false;
				if (failedAddress) {
					*failedAddress = it->address;
				}
			}
			continue;
		}
		if (!Equals(it->address, it->after) || !WriteCode(it->address, it->before) ||
		    !Equals(it->address, it->before)) {
			ok = false;
			if (failedAddress) {
				*failedAddress = it->address;
			} // Do not overwrite another owner's later change.
		} else if (RestoreProtection(*it)) {
			it->written = false;
		} else {
			ok = false;
			if (failedAddress) {
				*failedAddress = it->address;
			}
		}
	}
	return ok;
}

bool HookTransaction::Equals(std::uintptr_t address, std::span<const std::uint8_t> expected) noexcept
{
	std::array<std::uint8_t, kMaxPatchSize> bytes{};
	return !expected.empty() && expected.size() <= bytes.size() &&
	       Read(address, bytes.data(), expected.size()) &&
	       std::equal(expected.begin(), expected.end(), bytes.begin());
}

bool HookTransaction::Protection(const Change& c) noexcept
{
	MEMORY_BASIC_INFORMATION info{};
	return VirtualQuery(reinterpret_cast<void*>(c.address), &info, sizeof(info)) == sizeof(info) &&
	       info.Protect == c.protection;
}

bool HookTransaction::RestoreProtection(const Change& c) noexcept
{
	if (Protection(c)) {
		return true;
	}
	DWORD old{};
	return VirtualProtect(reinterpret_cast<void*>(c.address), c.before.size(), c.protection, &old) &&
	       Protection(c);
}

namespace
{
std::string EntryError(std::uintptr_t site, const safetyhook::InlineHook::Error& error)
{
	using Error = safetyhook::InlineHook::Error;
	if (error.type == Error::BAD_ALLOCATION) {
		const auto reason = error.allocator_error == safetyhook::Allocator::Error::BAD_VIRTUAL_ALLOC
		                        ? "virtual allocation failed"
		                        : "no allocation within the required range";
		return std::format("SafetyHook trampoline creation failed at {:#x}: {}; allocatorError={}", site,
		                   reason, static_cast<unsigned>(error.allocator_error));
	}
	const char* reason = "unknown dependency error";
	switch (error.type) {
	case Error::FAILED_TO_DECODE_INSTRUCTION:
		reason = "instruction decode failed";
		break;
	case Error::SHORT_JUMP_IN_TRAMPOLINE:
		reason = "short jump in trampoline";
		break;
	case Error::IP_RELATIVE_INSTRUCTION_OUT_OF_RANGE:
		reason = "IP-relative instruction out of range";
		break;
	case Error::UNSUPPORTED_INSTRUCTION_IN_TRAMPOLINE:
		reason = "unsupported trampoline instruction";
		break;
	case Error::FAILED_TO_UNPROTECT:
		reason = "memory protection change failed";
		break;
	case Error::NOT_ENOUGH_SPACE:
		reason = "insufficient trampoline space";
		break;
	default:
		return std::format("SafetyHook trampoline creation failed at {:#x}: unknown error={}", site,
		                   static_cast<unsigned>(error.type));
	}
	return std::format("SafetyHook trampoline creation failed at {:#x}: {}; error={} address={:#x}", site,
	                   reason, static_cast<unsigned>(error.type), reinterpret_cast<std::uintptr_t>(error.ip));
}
} // namespace

std::uintptr_t StageEntry(HookTransaction& transaction, std::uintptr_t site,
                          std::span<const std::uint8_t> prefix, std::uintptr_t wrapper)
{
	if (prefix.size() < 5 || prefix.size() > HookTransaction::kMaxPatchSize) {
		throw std::runtime_error(std::format("invalid entry length at {:#x}: expected=5..{} actual={}", site,
		                                     HookTransaction::kMaxPatchSize, prefix.size()));
	}
	RequireBytes(site, prefix, "entry preparation");
	auto created =
	    safetyhook::InlineHook::create(reinterpret_cast<void*>(site), reinterpret_cast<void*>(wrapper),
	                                   safetyhook::InlineHook::StartDisabled);
	if (!created) {
		throw std::runtime_error(EntryError(site, created.error()));
	}
	if (created->original_bytes().size() != prefix.size()) {
		throw std::runtime_error(std::format("entry length mismatch at {:#x}: expected={} actual={}", site,
		                                     prefix.size(), created->original_bytes().size()));
	}
	if (!std::ranges::equal(created->original_bytes(), prefix)) {
		throw std::runtime_error(
		    std::format("entry bytes changed during trampoline creation at {:#x}", site));
	}
	auto retained = std::make_unique<safetyhook::InlineHook>(std::move(*created));
	if (!FlushInstructionCache(GetCurrentProcess(), retained->trampoline().data(),
	                           retained->trampoline().size())) {
		const auto error = GetLastError();
		throw std::runtime_error(
		    std::format("trampoline cache flush failed at {:#x}; win32={}", site, error));
	}
	Assembly::Relay relay(wrapper);
	std::vector<std::uint8_t> after(prefix.size());
	Assembly::Jump(after, site, Publish(relay));
	transaction.Add(site, prefix, after);
	// Saved originals and downstream hooks may remain reachable after installation rollback.
	return retained.release()->trampoline().address();
}
} // namespace SVT::Hooks
