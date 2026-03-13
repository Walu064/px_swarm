#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "control/px4_interface.hpp"

namespace px_swarm::control {

struct LeaderParams {
    float v_xy_m_s = 2.0f;
    float v_z_m_s = 1.0f;
    float yaw_rate_deg_s = 45.0f;
};

/// Leader: maps keyboard input to body-velocity setpoints.
class LeaderController {
public:
    explicit LeaderController(LeaderParams p);

    /// Call each control tick (e.g., 20 Hz).
    /// Uses hold-to-move behaviour: if no key, command hover (zeros).
    void tick(Px4Interface& px4, const std::optional<std::string>& key);

private:
    LeaderParams _p{};
};

} // namespace px_swarm::control
