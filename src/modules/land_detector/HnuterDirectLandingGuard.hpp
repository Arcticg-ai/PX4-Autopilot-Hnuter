#pragma once

#include <cmath>
#include <cstdint>

namespace land_detector
{
// Direct Hnuter publishes ||F_requested|| / (2*m*g) as a landing hint.
// Low post-allocation motor mean alone is ambiguous during tilt/reallocation.
inline bool hnuter_direct_low_demand(bool direct, uint64_t now, uint64_t timestamp,
				    float demand, float threshold)
{
	return !direct || (timestamp > 0 && timestamp <= now && now - timestamp < 500000
			  && std::isfinite(demand) && demand >= 0.f && demand <= threshold);
}
}
