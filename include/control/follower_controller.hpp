#pragma once

#include <chrono>

#include "control/px4_interface.hpp"
#include "swarm/pso.hpp"
#include "swarm/swarm_state.hpp"

namespace px_swarm::control {

struct FollowerParams {
    float update_hz = 5.0f;     // PSO solve rate
    float control_hz = 20.0f;   // offboard command rate

    float max_speed_m_s = 3.0f; // speed limit to reduce MAVLink stress + stable flight
    float kp = 0.8f;            // simple proportional gain for velocity towards target

    // Formation/safety
    float safe_radius_m = 3.0f;
    float formation_spacing_m = 4.5f;
    float cohesion_radius_m = 6.0f;

    // Search bounds (PSO box around desired point)
    float search_xy_m = 6.0f;
    float search_z_m  = 2.0f;
};

/// Follower: periodically runs PSO to compute best target position,
/// then uses a simple controller to fly toward it.
class FollowerController {
public:
    struct Diagnostics {
        bool solved_this_tick{false};
        bool used_cached_target{false};
        float best_fitness{0.0f};
        int neighbor_count{0};
        swarm::Vec3 desired{};
        swarm::Vec3 target{};
        float desired_to_target_m{0.0f};
        float current_to_target_m{0.0f};
    };

    FollowerController(int self_id, FollowerParams p, swarm::PsoParams pso);

    /// desired_anchor is typically leader pose (or formation center).
    /// The "desired position" for this follower is computed from formation policy inside.
    void tick(Px4Interface& px4, const swarm::SwarmState& swarm_state,
              const std::chrono::steady_clock::time_point now);

    const Diagnostics& diagnostics() const { return _diag; }
    const swarm::PsoParams& pso_params() const { return _pso_params; }

private:
    int _self_id{};
    FollowerParams _p{};
    swarm::PsoParams _pso_params{};
    swarm::PsoOptimizer _pso{};

    // timing
    std::chrono::steady_clock::time_point _last_solve{};
    std::optional<swarm::Vec3> _last_target{};
    float _last_best_fitness{0.0f};
    Diagnostics _diag{};

    // formation: compute desired position for this follower based on leader pose etc.
    swarm::Vec3 _desired_position(const swarm::SwarmState& swarm_state,
                                  const swarm::Vec3& current) const;

    // control: command velocity towards target (bounded)
    void _fly_towards(Px4Interface& px4, const swarm::Vec3& current, const swarm::Vec3& target) const;
};

} // namespace px_swarm::control
