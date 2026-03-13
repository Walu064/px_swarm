#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace px_swarm::control {

struct PoseNed {
    float north_m{};
    float east_m{};
    float down_m{};     // NOTE: Down is positive downward, negative means "up"
    float vel_north_m_s{};
    float vel_east_m_s{};
    float vel_down_m_s{};
    float yaw_deg{};
    std::chrono::steady_clock::time_point stamp{};
};

enum class CommandMode {
    None,
    VelocityBody,
    VelocityNed,
    PositionNed,
};

struct LastCommand {
    CommandMode mode{CommandMode::None};
    float a{};
    float b{};
    float c{};
    float d{};
    std::chrono::steady_clock::time_point stamp{};
};

class Px4Interface {
public:
    Px4Interface();
    ~Px4Interface();

    Px4Interface(const Px4Interface&) = delete;
    Px4Interface& operator=(const Px4Interface&) = delete;

    /// Connect to PX4 SITL/vehicle via MAVSDK.
    /// Example: udp_port=14540 => connection string "udpin://0.0.0.0:14540"
    bool connect_udp(int udp_port);

    /// Arm + takeoff (simple). Returns false on failure.
    bool arm_and_takeoff(float takeoff_alt_m = 5.0f);

    /// Land and stop offboard (best-effort).
    void land_and_disarm_best_effort();

    /// Start OFFBOARD mode. Requires sending an initial setpoint first.
    bool start_offboard();

    /// Stop OFFBOARD mode.
    void stop_offboard();

    /// Offboard setpoints
    void set_velocity_body(float forward_m_s, float right_m_s, float down_m_s, float yaw_rate_deg_s);
    void set_velocity_ned(float north_m_s, float east_m_s, float down_m_s, float yaw_rate_deg_s);
    void set_position_ned(float north_m, float east_m, float down_m, float yaw_deg);

    /// Telemetry snapshot (last known).
    std::optional<PoseNed> pose_ned() const;
    std::optional<LastCommand> last_command() const;

    /// Convenience: block until system connected (with timeout).
    bool wait_until_connected(std::chrono::milliseconds timeout);

    /// Convenience: wait until the vehicle reaches the commanded takeoff altitude.
    bool wait_until_takeoff_altitude(float takeoff_alt_m,
                                     std::chrono::milliseconds timeout,
                                     float tolerance_m = 0.5f) const;

private:
    std::unique_ptr<mavsdk::Mavsdk> _mavsdk;
    std::shared_ptr<mavsdk::System> _system;

    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::Offboard> _offboard;
    std::unique_ptr<mavsdk::Telemetry> _telemetry;

    mutable std::optional<PoseNed> _last_pose{};
    mutable std::optional<LastCommand> _last_command{};
    void _setup_telemetry_rates();

    // helper
    static std::shared_ptr<mavsdk::System> _wait_for_system(mavsdk::Mavsdk& sdk,
                                                           std::chrono::milliseconds timeout);
};

} // namespace px_swarm::control
