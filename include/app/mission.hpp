#pragma once

#include <string>
#include <vector>

namespace px_swarm::app {

struct MissionWaypoint {
    std::string id{};
    float north_m{};
    float east_m{};
    float down_m{};
    float speed_m_s{0.0f}; // 0 => use default mission speed
    float hold_s{0.0f};
};

struct MissionPlan {
    std::string name{};
    float default_speed_m_s{2.5f};
    std::vector<MissionWaypoint> waypoints{};
};

MissionPlan load_mission_yaml(const std::string& path);

} // namespace px_swarm::app
