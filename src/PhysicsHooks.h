#pragma once
#include <cstdint>
#include <span>
#include "Hooks.h"

namespace SVT::PhysicsHooks
{
// The source hooks must already be published; installation verifies their bytes before and after patching.
void Install(std::span<const Hooks::Change> sites);
bool Restore(std::uintptr_t* failedAddress = nullptr) noexcept;
} // namespace SVT::PhysicsHooks
