#include "control/follower_controller.hpp"

#include <algorithm>
#include <cmath>

namespace px_swarm::control {

FollowerController::FollowerController(int self_id, FollowerParams p, swarm::PsoParams pso)
    : _self_id(self_id), _p(p), _pso_params(pso)
{
    _last_solve = std::chrono::steady_clock::time_point{};
}

swarm::Vec3 FollowerController::_desired_position(const swarm::SwarmState& swarm_state,
                                                  const swarm::Vec3& current) const
{
    (void)current;
    const auto leader = swarm_state.leader();
    if (!leader) {
        return current;
    }

    const float min_distance_m = std::max(0.1f, _p.safe_radius_m);
    const float max_distance_m = std::max(min_distance_m, _p.cohesion_radius_m);
    const float target_distance_m = std::clamp(_p.formation_spacing_m, min_distance_m, max_distance_m);
    const float heading = static_cast<float>(std::max(1, _self_id)) * 1.0471975512f;

    const swarm::Vec3 offset{
        std::cos(heading) * target_distance_m,
        std::sin(heading) * target_distance_m,
        0.f
    };

    return swarm::Vec3{
        leader->pos_ned.x + offset.x,
        leader->pos_ned.y + offset.y,
        leader->pos_ned.z + offset.z
    };
}

void FollowerController::_fly_towards(Px4Interface& px4, const swarm::Vec3& /*current*/, const swarm::Vec3& target) const
{
    // P-controller to velocity in NED (we'll use body velocity? easiest is position setpoint;
    // but velocity gives smoother behaviour for frequent target changes).
    // We'll command BODY velocity is harder (needs heading transform), so use POSITION setpoint.
    // For stability and simplicity: send position setpoint to target, yaw fixed to 0.
    px4.set_position_ned(target.x, target.y, target.z, 0.f);
}

void FollowerController::tick(Px4Interface& px4, const swarm::SwarmState& swarm_state,
                             const std::chrono::steady_clock::time_point now)
{
    const auto pose = px4.pose_ned();
    if (!pose) return;

    const swarm::Vec3 current{pose->north_m, pose->east_m, pose->down_m};

    // compute desired base (formation)
    const swarm::Vec3 desired = _desired_position(swarm_state, current);

    // collect neighbors (exclude self)
    std::vector<swarm::Vec3> neighbors;
    for (const auto& a : swarm_state.all(_self_id)) {
        neighbors.push_back(a.pos_ned);
    }
    _diag.neighbor_count = static_cast<int>(neighbors.size());
    _diag.desired = desired;
    _diag.solved_this_tick = false;
    _diag.used_cached_target = false;

    // Solve PSO at update_hz, otherwise keep last target
    const auto min_dt = std::chrono::duration<double>(1.0 / std::max(0.1f, _p.update_hz));
    const bool should_solve = (_last_solve.time_since_epoch().count() == 0)
        || (now - _last_solve >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(min_dt));

    if (should_solve) {
        swarm::PsoProblem prob{};
        prob.desired = desired;
        prob.neighbors = neighbors;
        prob.safe_radius_m = _p.safe_radius_m;
        prob.cohesion_radius_m = _p.cohesion_radius_m;
        prob.prev_target = _last_target;
        prob.w = swarm::FitnessWeights{1.0f, 3.0f, 0.2f};

        // bounds: around desired point
        prob.bounds.min = swarm::Vec3{desired.x - _p.search_xy_m, desired.y - _p.search_xy_m, desired.z - _p.search_z_m};
        prob.bounds.max = swarm::Vec3{desired.x + _p.search_xy_m, desired.y + _p.search_xy_m, desired.z + _p.search_z_m};

        const auto best = _pso.solve(_pso_params, prob, &_last_best_fitness);
        _last_target = best;
        _last_solve = now;
        _diag.solved_this_tick = true;
    } else {
        _diag.used_cached_target = _last_target.has_value();
    }

    if (_last_target) {
        _diag.target = *_last_target;
        const float dx_dt = desired.x - _last_target->x;
        const float dy_dt = desired.y - _last_target->y;
        const float dz_dt = desired.z - _last_target->z;
        _diag.desired_to_target_m = std::sqrt(dx_dt * dx_dt + dy_dt * dy_dt + dz_dt * dz_dt);

        const float dx_ct = current.x - _last_target->x;
        const float dy_ct = current.y - _last_target->y;
        const float dz_ct = current.z - _last_target->z;
        _diag.current_to_target_m = std::sqrt(dx_ct * dx_ct + dy_ct * dy_ct + dz_ct * dz_ct);
        _diag.best_fitness = _last_best_fitness;
        _fly_towards(px4, current, *_last_target);
    } else {
        _diag.target = current;
        _diag.desired_to_target_m = 0.0f;
        _diag.current_to_target_m = 0.0f;
        _diag.best_fitness = _last_best_fitness;
        // hover-ish: keep current position
        px4.set_position_ned(current.x, current.y, current.z, 0.f);
    }
}

} // namespace px_swarm::control
