#pragma once

#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "Assembly.h"
#include "RE/A/Actor.h"

namespace SVT::ReferenceFrameZ
{
inline constexpr std::array<std::uint8_t, 6> kMoveCallOriginal{0xFF, 0x90, 0x40, 0x06, 0x00, 0x00};

// 5D93FD's original RAX is the caller-loaded vptr. Capture its current target
// in private argument 4 (R9), then tail-jump to typed C++. This is a leaf:
// no stack adjustment or nonvolatile register change and no synthetic unwind.
inline std::array<std::uint8_t, 21> MoveShim(std::uintptr_t wrapper)
{
	std::array<std::uint8_t, 21> out{};
	SVT::Assembly::Code code(out.size(), out.data());
	code.mov(code.r9, code.ptr[code.rax + RE::Actor::MoveSlot * sizeof(void*)]);
	code.AbsoluteJump(wrapper);
	code.RequireSize(out.size());
	return out;
}

// A six-byte call retains native return 5D9403. E8 + NOP would return one byte
// earlier and is deliberately not used for the original FF90 instruction.
inline std::array<std::uint8_t, 6> MoveCall(std::uintptr_t site, std::uintptr_t pointerSlot)
{
	SVT::Assembly::CheckRelative(site, pointerSlot, 6);
	std::array<std::uint8_t, 6> out{};
	SVT::Assembly::Code code(out.size(), out.data());
	const auto relocated = code.getCode<std::uintptr_t>() + (pointerSlot - site);
	code.call(code.ptr[code.rip + reinterpret_cast<const void*>(relocated)]);
	code.RequireSize(out.size());
	return out;
}
} // namespace SVT::ReferenceFrameZ
