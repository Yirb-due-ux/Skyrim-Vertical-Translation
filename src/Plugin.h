#pragma once
#include <string_view>

namespace SKSE
{
class LoadInterface;
}

namespace SVT
{
inline constexpr std::string_view kPluginName{"Skyrim Vertical Translation"};
bool Load(const SKSE::LoadInterface* skse);
void InstallMotion();
} // namespace SVT
