#include "control/px4_interface.hpp"

#include <iostream>
#include <thread>

namespace px_swarm::control {

Px4Interface::Px4Interface()
    : _mavsdk(std::make_unique<mavsdk::Mavsdk>(
          mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}))
{
}

Px4Interface::~Px4Interface()
{
    // best effort shutdown
    try {
        stop_offboard();
    } catch (...) {}
}

bool Px4Interface::connect_udp(int udp_port)
{
    if (!_mavsdk) return false;

    const std::string url = "udpin://0.0.0.0:" + std::to_string(udp_port);
    const auto res = _mavsdk->add_any_connection(url);
    if (res != mavsdk::ConnectionResult::Success) {
        std::cerr << "MAVSDK connect failed: " << int(res) << "\n";
        return false;
    }

    _system = _wait_for_system(*_mavsdk, std::chrono::milliseconds(8000));
    if (!_system) {
        std::cerr << "No system detected (timeout)\n";
        return false;
    }

    _action = std::make_unique<mavsdk::Action>(_system);
    _offboard = std::make_unique<mavsdk::Offboard>(_system);
    _telemetry = std::make_unique<mavsdk::Telemetry>(_system);

    _setup_telemetry_rates();

    // cache pose via callbacks
    _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed pv) {
        if (!_last_pose) _last_pose = PoseNed{};
        _last_pose->north_m = pv.position.north_m;
        _last_pose->east_m  = pv.position.east_m;
        _last_pose->down_m  = pv.position.down_m;
        _last_pose->vel_north_m_s = pv.velocity.north_m_s;
        _last_pose->vel_east_m_s  = pv.velocity.east_m_s;
        _last_pose->vel_down_m_s  = pv.velocity.down_m_s;
        _last_pose->stamp   = std::chrono::steady_clock::now();
    });

    _telemetry->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle e) {
        if (!_last_pose) _last_pose = PoseNed{};
        _last_pose->yaw_deg = e.yaw_deg;
        _last_pose->stamp   = std::chrono::steady_clock::now();
    });

    return true;
}

bool Px4Interface::wait_until_connected(std::chrono::milliseconds timeout)
{
    if (!_system) {
        if (!_mavsdk) return false;
        _system = _wait_for_system(*_mavsdk, timeout);
    }
    return static_cast<bool>(_system);
}

bool Px4Interface::arm_and_takeoff(float takeoff_alt_m)
{
    if (!_action) return false;

    if (_action->set_takeoff_altitude(takeoff_alt_m) != mavsdk::Action::Result::Success) {
        std::cerr << "Set takeoff altitude failed\n";
        return false;
    }

    // Give SITL time to become armable before the first arm request.
    if (_telemetry) {
        const auto armable_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (std::chrono::steady_clock::now() < armable_deadline) {
            if (_telemetry->health().is_armable) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    mavsdk::Action::Result arm_result = mavsdk::Action::Result::Unknown;
    const auto arm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    do {
        arm_result = _action->arm();
        if (arm_result == mavsdk::Action::Result::Success) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } while (std::chrono::steady_clock::now() < arm_deadline);

    if (arm_result != mavsdk::Action::Result::Success) {
        std::cerr << "Arm failed (" << int(arm_result) << "), trying force-arm fallback\n";
        arm_result = _action->arm_force();
    }

    if (arm_result != mavsdk::Action::Result::Success) {
        std::cerr << "Arm failed\n";
        return false;
    }

    // Takeoff
    if (_action->takeoff() != mavsdk::Action::Result::Success) {
        std::cerr << "Takeoff failed\n";
        return false;
    }

    return true;
}

void Px4Interface::land_and_disarm_best_effort()
{
    if (_offboard) {
        try { _offboard->stop(); } catch (...) {}
    }
    if (_action) {
        try { _action->land(); } catch (...) {}
    }
}

bool Px4Interface::start_offboard()
{
    if (!_offboard) return false;
    const auto r = _offboard->start();
    if (r != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << int(r) << "\n";
        return false;
    }
    return true;
}

void Px4Interface::stop_offboard()
{
    if (!_offboard) return;
    _offboard->stop();
}

void Px4Interface::set_velocity_body(float forward_m_s, float right_m_s, float down_m_s, float yaw_rate_deg_s)
{
    if (!_offboard) return;
    mavsdk::Offboard::VelocityBodyYawspeed v{};
    v.forward_m_s = forward_m_s;
    v.right_m_s = right_m_s;
    v.down_m_s = down_m_s;
    v.yawspeed_deg_s = yaw_rate_deg_s;
    _offboard->set_velocity_body(v);

    _last_command = LastCommand{
        CommandMode::VelocityBody,
        forward_m_s,
        right_m_s,
        down_m_s,
        yaw_rate_deg_s,
        std::chrono::steady_clock::now()
    };
}

void Px4Interface::set_velocity_ned(float north_m_s, float east_m_s, float down_m_s, float yaw_rate_deg_s)
{
    if (!_offboard) return;
    mavsdk::Offboard::VelocityNedYaw v{};
    v.north_m_s = north_m_s;
    v.east_m_s = east_m_s;
    v.down_m_s = down_m_s;
    v.yaw_deg = yaw_rate_deg_s; // NOTE: MAVSDK uses yaw angle here; for yaw rate use body mode
    _offboard->set_velocity_ned(v);

    _last_command = LastCommand{
        CommandMode::VelocityNed,
        north_m_s,
        east_m_s,
        down_m_s,
        yaw_rate_deg_s,
        std::chrono::steady_clock::now()
    };
}

void Px4Interface::set_position_ned(float north_m, float east_m, float down_m, float yaw_deg)
{
    if (!_offboard) return;
    mavsdk::Offboard::PositionNedYaw p{};
    p.north_m = north_m;
    p.east_m = east_m;
    p.down_m = down_m;
    p.yaw_deg = yaw_deg;
    _offboard->set_position_ned(p);

    _last_command = LastCommand{
        CommandMode::PositionNed,
        north_m,
        east_m,
        down_m,
        yaw_deg,
        std::chrono::steady_clock::now()
    };
}

std::optional<PoseNed> Px4Interface::pose_ned() const
{
    return _last_pose;
}

std::optional<LastCommand> Px4Interface::last_command() const
{
    return _last_command;
}

bool Px4Interface::wait_until_takeoff_altitude(float takeoff_alt_m,
                                               std::chrono::milliseconds timeout,
                                               float tolerance_m) const
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const float target_down_m = -(takeoff_alt_m - tolerance_m);

    while (std::chrono::steady_clock::now() < deadline) {
        if (_last_pose && _last_pose->down_m <= target_down_m) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}

void Px4Interface::_setup_telemetry_rates()
{
    if (!_telemetry) return;
    _telemetry->set_rate_position_velocity_ned(20.0);
    _telemetry->set_rate_attitude_euler(20.0);
}

std::shared_ptr<mavsdk::System> Px4Interface::_wait_for_system(mavsdk::Mavsdk& sdk,
                                                              std::chrono::milliseconds timeout)
{
    const auto system = sdk.first_autopilot(timeout.count() / 1000.0);
    if (!system) return {};
    return *system;
}

} // namespace px_swarm::control
