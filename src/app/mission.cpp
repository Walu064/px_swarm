#include "app/mission.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

namespace px_swarm::app {

static std::string trim(std::string s)
{
    const auto is_ws = [](const char c) {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r';
    };

    while (!s.empty() && is_ws(s.front())) s.erase(s.begin());
    while (!s.empty() && is_ws(s.back())) s.pop_back();
    return s;
}

static bool starts_with(const std::string& s, const std::string& p)
{
    return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

static std::string value_after_colon(const std::string& s)
{
    const auto pos = s.find(':');
    if (pos == std::string::npos) return {};
    return trim(s.substr(pos + 1));
}

static float parse_float(const std::string& s, const char* field)
{
    try {
        return std::stof(trim(s));
    } catch (...) {
        throw std::runtime_error(std::string("Invalid float for field: ") + field);
    }
}

MissionPlan load_mission_yaml(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open mission file: " + path);
    }

    MissionPlan plan{};
    bool in_waypoints = false;

    std::string line;
    while (std::getline(in, line)) {
        // Strip comments.
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        const std::string t = trim(line);
        if (t.empty()) continue;

        if (starts_with(t, "name:")) {
            plan.name = value_after_colon(t);
            continue;
        }

        if (starts_with(t, "default_speed_m_s:")) {
            plan.default_speed_m_s = parse_float(value_after_colon(t), "default_speed_m_s");
            continue;
        }

        if (t == "waypoints:") {
            in_waypoints = true;
            continue;
        }

        if (!in_waypoints) continue;

        if (starts_with(t, "- ")) {
            MissionWaypoint wp{};
            plan.waypoints.push_back(wp);

            const std::string rest = trim(t.substr(2));
            if (starts_with(rest, "id:")) {
                plan.waypoints.back().id = value_after_colon(rest);
            }
            continue;
        }

        if (plan.waypoints.empty()) continue;
        MissionWaypoint& wp = plan.waypoints.back();

        if (starts_with(t, "id:")) wp.id = value_after_colon(t);
        else if (starts_with(t, "north_m:")) wp.north_m = parse_float(value_after_colon(t), "north_m");
        else if (starts_with(t, "east_m:")) wp.east_m = parse_float(value_after_colon(t), "east_m");
        else if (starts_with(t, "down_m:")) wp.down_m = parse_float(value_after_colon(t), "down_m");
        else if (starts_with(t, "speed_m_s:")) wp.speed_m_s = parse_float(value_after_colon(t), "speed_m_s");
        else if (starts_with(t, "hold_s:")) wp.hold_s = parse_float(value_after_colon(t), "hold_s");
    }

    if (plan.waypoints.empty()) {
        throw std::runtime_error("Mission has no waypoints: " + path);
    }
    if (plan.name.empty()) {
        plan.name = "unnamed_mission";
    }

    return plan;
}

} // namespace px_swarm::app
