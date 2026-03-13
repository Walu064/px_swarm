#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace px_swarm::app {

enum class Role { Leader, Follower };

struct CliArgs {
    int udp_port = 14540;               // local listen port for MAVSDK (PX4 offboard link)
    Role role = Role::Leader;
    int vehicle_id = 0;                 // logical ID in swarm messages

    std::string swarm_ip = "239.255.0.1"; // multicast group for swarm state
    uint16_t swarm_port = 9000;         // UDP port for swarm bus

    // Leader tuning
    float leader_v_xy = 2.0f;           // m/s
    float leader_v_z  = 1.0f;           // m/s (up/down)
    float leader_yaw_rate = 45.0f;      // deg/s

    // Follower tuning
    float follow_update_hz = 5.0f;      // PSO update rate
    float control_hz = 20.0f;           // offboard loop rate
    float takeoff_alt_m = 10.0f;        // startup altitude for every vehicle

    // Formation / safety
    float safe_radius_m = 3.0f;         // min separation
    float formation_spacing_m = 4.5f;   // typical spacing
    float cohesion_radius_m = 6.0f;     // max desired separation

    bool auto_takeoff = false;          // start and enter offboard automatically

    // Logging
    bool csv_log_enabled = true;
    std::string log_dir = "./logs";
    std::string flight_tag{};

    // Leader mission (optional)
    std::string mission_file{};
    int swarm_size = 1;                // expected number of drones (leader + followers)
    float mission_leader_max_speed_m_s = 0.0f; // <=0 means no extra mission speed cap
};

/// Parse args like:
///   --port 14540 --role leader --id 0 --swarm-ip 127.0.0.1 --swarm-port 9000
/// Optional tuning flags described in implementation (.cpp).
CliArgs parse_args(int argc, char** argv);

std::string role_to_string(Role r);

} // namespace px_swarm::app
