#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace SVT::CorrectedZ
{
struct Movement
{
	std::uintptr_t actor{};
	std::uintptr_t controller{};
	std::uint64_t episode{};
	std::uint64_t serial{};

	bool operator==(const Movement&) const = default;
};

struct Correction
{
	Movement movement{};
	bool authoredVelocitySubmitted{};
	bool integrated{};
	std::array<float, 3> proxyPosition{};
};

// Admit only a synchronous event from the movement whose native integration
// supplied this position. The caller keeps the native event Z unchanged.
[[nodiscard]] inline bool CanCommit(const Correction& correction, const Movement& event, bool sameActor,
                                    float deltaZ, float eventZ, const std::array<float, 3>& proxyNow) noexcept
{
	if (!sameActor || !event.actor || !event.controller || !event.episode || !event.serial ||
	    event != correction.movement || !correction.authoredVelocitySubmitted || !correction.integrated ||
	    !std::isfinite(deltaZ) || !std::isfinite(eventZ)) {
		return false;
	}
	for (std::size_t i = 0; i < proxyNow.size(); ++i) {
		if (!std::isfinite(proxyNow[i]) || proxyNow[i] != correction.proxyPosition[i]) {
			return false;
		}
	}
	return true;
}
} // namespace SVT::CorrectedZ
