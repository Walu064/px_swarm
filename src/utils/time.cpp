#include "utils/time.hpp"

#include <chrono>
#include <thread>

namespace px_swarm::utils {

uint64_t now_us()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

void sleep_for_hz(double hz)
{
    if (hz <= 0.0) return;
    const auto dt = std::chrono::duration<double>(1.0 / hz);
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(dt));
}

} // namespace px_swarm::utils
