#pragma once

#include <chrono>
#include <optional>
#include <unordered_map>
#include <vector>

#include "swarm/swarm_bus.hpp"

namespace px_swarm::swarm {

struct Vec3 {
    float x{}, y{}, z{}; // we'll use x=N, y=E, z=D for NED
};

struct AgentState {
    int id{};
    Vec3 pos_ned{};
    float yaw_deg{};
    int reported_leader_id{-1};
    std::chrono::steady_clock::time_point stamp{};
};

/// Maintains a cache of latest agent states (id -> state), with staleness handling.
class SwarmState {
public:
    void ingest(const SwarmMsg& msg, std::chrono::steady_clock::time_point now);
    void prune(std::chrono::steady_clock::time_point now);

    std::optional<AgentState> get(int id) const;

    /// Returns all known agents (optionally excluding self).
    std::vector<AgentState> all(std::optional<int> exclude_id = std::nullopt) const;

    /// Consider state stale if older than this threshold.
    void set_stale_after(std::chrono::milliseconds ms);

    /// Leader ID convention (default 0)
    void set_leader_id(int id);
    int leader_id() const;

    std::optional<AgentState> leader() const;
    int highest_active_id() const;

private:
    std::unordered_map<int, AgentState> _agents;
    std::chrono::milliseconds _stale_after{1000};
    int _leader_id{0};
};

} // namespace px_swarm::swarm
