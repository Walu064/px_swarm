#include "app/cli.hpp"

#include <stdexcept>

namespace px_swarm::app {

static Role parse_role(const std::string& s)
{
    if (s == "leader") return Role::Leader;
    if (s == "follower") return Role::Follower;
    throw std::runtime_error("Invalid --role (use leader|follower)");
}

std::string role_to_string(Role r)
{
    return (r == Role::Leader) ? "leader" : "follower";
}

CliArgs parse_args(int argc, char** argv)
{
    CliArgs a{};

    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc) throw std::runtime_error("Missing value for argument");
        return std::string(argv[++i]);
    };

    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];

        if (k == "--port") a.udp_port = std::stoi(next(i));
        else if (k == "--role") a.role = parse_role(next(i));
        else if (k == "--id") a.vehicle_id = std::stoi(next(i));
        else if (k == "--swarm-ip") a.swarm_ip = next(i);
        else if (k == "--swarm-port") a.swarm_port = static_cast<uint16_t>(std::stoi(next(i)));

        else if (k == "--leader-vxy") a.leader_v_xy = std::stof(next(i));
        else if (k == "--leader-vz") a.leader_v_z = std::stof(next(i));
        else if (k == "--leader-yaw-rate") a.leader_yaw_rate = std::stof(next(i));

        else if (k == "--follow-update-hz") a.follow_update_hz = std::stof(next(i));
        else if (k == "--control-hz") a.control_hz = std::stof(next(i));
        else if (k == "--takeoff-alt") a.takeoff_alt_m = std::stof(next(i));

        else if (k == "--safe-radius") a.safe_radius_m = std::stof(next(i));
        else if (k == "--spacing") a.formation_spacing_m = std::stof(next(i));
        else if (k == "--max-spacing") a.cohesion_radius_m = std::stof(next(i));
        else if (k == "--auto-takeoff") a.auto_takeoff = true;
        else if (k == "--log-dir") a.log_dir = next(i);
        else if (k == "--flight-tag") a.flight_tag = next(i);
        else if (k == "--no-csv-log") a.csv_log_enabled = false;
        else if (k == "--mission") a.mission_file = next(i);
        else if (k == "--swarm-size") a.swarm_size = std::stoi(next(i));
        else if (k == "--mission-leader-max-speed") a.mission_leader_max_speed_m_s = std::stof(next(i));
        // unknown args ignored for now (or throw if you prefer)
    }

    if (a.swarm_size < 1) a.swarm_size = 1;
    return a;
}

} // namespace px_swarm::app
