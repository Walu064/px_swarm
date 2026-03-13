#include "control/leader_controller.hpp"

namespace px_swarm::control {

LeaderController::LeaderController(LeaderParams p) : _p(p) {}

void LeaderController::tick(Px4Interface& px4, const std::optional<std::string>& key)
{
    float fwd = 0.f, right = 0.f, down = 0.f, yaw_rate = 0.f;

    if (key) {
        // Arrow keys: forward/back + yaw
        if (*key == "UP") fwd = +_p.v_xy_m_s;
        else if (*key == "DOWN") fwd = -_p.v_xy_m_s;
        else if (*key == "LEFT") yaw_rate = -_p.yaw_rate_deg_s;
        else if (*key == "RIGHT") yaw_rate = +_p.yaw_rate_deg_s;

        // WASD: strafe + altitude (optional)
        else if (*key == "A") right = -_p.v_xy_m_s;
        else if (*key == "D") right = +_p.v_xy_m_s;
        else if (*key == "W") down = -_p.v_z_m_s; // up
        else if (*key == "S") down = +_p.v_z_m_s; // down

        // Q/E: yaw as well
        else if (*key == "Q") yaw_rate = -_p.yaw_rate_deg_s;
        else if (*key == "E") yaw_rate = +_p.yaw_rate_deg_s;

        // SPACE: hover
        else if (*key == "SPACE") { fwd = right = down = 0.f; yaw_rate = 0.f; }
    }

    px4.set_velocity_body(fwd, right, down, yaw_rate);
}

} // namespace px_swarm::control
