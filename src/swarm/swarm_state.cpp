#include "swarm/swarm_state.hpp"

namespace px_swarm::swarm {

void SwarmState::ingest(const SwarmMsg& msg, std::chrono::steady_clock::time_point now)
{
    if (msg.kind != SwarmMsgKind::State) {
        prune(now);
        return;
    }

    AgentState st{};
    st.id = msg.id;
    st.pos_ned = Vec3{msg.north_m, msg.east_m, msg.down_m};
    st.yaw_deg = msg.yaw_deg;
    st.reported_leader_id = msg.leader_id;
    st.stamp = now;

    _agents[msg.id] = st;

    prune(now);
}

void SwarmState::prune(std::chrono::steady_clock::time_point now)
{
    for (auto it = _agents.begin(); it != _agents.end(); ) {
        if (now - it->second.stamp > _stale_after) it = _agents.erase(it);
        else ++it;
    }
}

std::optional<AgentState> SwarmState::get(int id) const
{
    auto it = _agents.find(id);
    if (it == _agents.end()) return std::nullopt;
    return it->second;
}

std::vector<AgentState> SwarmState::all(std::optional<int> exclude_id) const
{
    std::vector<AgentState> out;
    out.reserve(_agents.size());
    for (const auto& [id, st] : _agents) {
        if (exclude_id && id == *exclude_id) continue;
        out.push_back(st);
    }
    return out;
}

void SwarmState::set_stale_after(std::chrono::milliseconds ms) { _stale_after = ms; }

void SwarmState::set_leader_id(int id) { _leader_id = id; }

int SwarmState::leader_id() const { return _leader_id; }

std::optional<AgentState> SwarmState::leader() const { return get(_leader_id); }

int SwarmState::highest_active_id() const
{
    int best = _leader_id;
    for (const auto& [id, _] : _agents) {
        if (id > best) best = id;
    }
    return best;
}

} // namespace px_swarm::swarm
