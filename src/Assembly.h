#pragma once

#include <xbyak/xbyak.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <format>
#include <span>
#include <stdexcept>

namespace SVT::Assembly
{
class Code : public Xbyak::CodeGenerator
{
  public:
	using CodeGenerator::CodeGenerator;

	void AbsoluteJump(std::uintptr_t target)
	{
		jmp(ptr[rip]);
		dq(target);
	}

	void RequireSize(std::size_t expected) const
	{
		if (getSize() != expected) {
			throw std::runtime_error(
			    std::format("generated code length mismatch: expected={} actual={}", expected, getSize()));
		}
	}
};

inline std::span<const std::uint8_t> Bytes(const Xbyak::CodeGenerator& code)
{
	return {code.getCode(), code.getSize()};
}

class Relay : public Code
{
  public:
	explicit Relay(std::uintptr_t target) : Code(14)
	{
		AbsoluteJump(target);
		RequireSize(14);
	}
};

inline void CheckRelative(std::uintptr_t site, std::uintptr_t target, std::size_t length)
{
	if (site > UINTPTR_MAX - length) {
		throw std::runtime_error(
		    std::format("relative branch origin overflow: site={:#x} length={}", site, length));
	}
	const auto end = site + length;
	if (target >= end ? target - end > INT32_MAX : end - target > std::uint64_t(INT32_MAX) + 1) {
		throw std::runtime_error(
		    std::format("relative branch out of range: site={:#x} target={:#x}", site, target));
	}
}

// Translate the execution origin into the assembler's buffer. AutoGrow's fixup
// resolves the full signed rel32 range; the reserved buffer must never move.
inline void Relative(std::span<std::uint8_t> out, std::uintptr_t site, std::uintptr_t target, bool call)
{
	if (out.size() < 5 || out.size() > 32) {
		throw std::runtime_error(
		    std::format("invalid relative branch buffer length: actual={} supported=5..32", out.size()));
	}
	CheckRelative(site, target, 5);
	Code code(64, Xbyak::AutoGrow);
	const auto buffer = code.getCode<std::uintptr_t>();
	const auto relocated = reinterpret_cast<const void*>(buffer + (target - site));
	if (call) {
		code.call(relocated);
	} else {
		code.jmp(relocated, Xbyak::CodeGenerator::T_NEAR);
	}
	code.RequireSize(5);
	code.nop(out.size() - code.getSize());
	code.ready(Xbyak::CodeArray::PROTECT_RW);
	if (code.getCode<std::uintptr_t>() != buffer) {
		throw std::runtime_error(std::format("relative branch buffer moved: before={:#x} after={:#x}", buffer,
		                                     code.getCode<std::uintptr_t>()));
	}
	code.RequireSize(out.size());
	std::memcpy(out.data(), code.getCode(), out.size());
}

inline void Call(std::span<std::uint8_t> out, std::uintptr_t site, std::uintptr_t target)
{
	Relative(out, site, target, true);
}

inline std::array<std::uint8_t, 5> Call(std::uintptr_t site, std::uintptr_t target)
{
	std::array<std::uint8_t, 5> out{};
	Call(out, site, target);
	return out;
}

inline void Jump(std::span<std::uint8_t> out, std::uintptr_t site, std::uintptr_t target)
{
	Relative(out, site, target, false);
}
} // namespace SVT::Assembly
