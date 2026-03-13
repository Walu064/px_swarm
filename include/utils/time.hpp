#pragma once

#include <cstdint>
#include <chrono>

namespace px_swarm::utils {

/// Monotonic timestamp in microseconds since steady_clock epoch.
uint64_t now_us();

/// Sleep to maintain loop frequency.
void sleep_for_hz(double hz);

} // namespace px_swarm::utils
