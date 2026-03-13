#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <netinet/in.h> // sockaddr_in

namespace px_swarm::swarm {

enum class SwarmMsgKind : char {
    State = 'S',
    Election = 'E',
    Ok = 'O',
    Coordinator = 'C',
};

struct SwarmMsg {
    static constexpr uint32_t FlagMissionLanding = 1u << 0;

    SwarmMsgKind kind = SwarmMsgKind::State;
    int id = 0;
    int target_id = -1;
    int leader_id = -1;
    float north_m = 0.f;
    float east_m = 0.f;
    float down_m = 0.f;
    float yaw_deg = 0.f;
    uint64_t t_us = 0;
    uint32_t flags = 0;
};

class SwarmBus {
public:
    SwarmBus();
    ~SwarmBus();

    SwarmBus(const SwarmBus&) = delete;
    SwarmBus& operator=(const SwarmBus&) = delete;

    bool open_sender(const std::string& ip, uint16_t port);
    bool open_receiver(const std::string& ip, uint16_t port);

    bool send(const SwarmMsg& msg);
    std::optional<SwarmMsg> try_recv();

private:
    int _send_sock{-1};
    int _recv_sock{-1};
    sockaddr_in _dst_addr{};
    bool _has_dst{false};
};

} // namespace px_swarm::swarm
