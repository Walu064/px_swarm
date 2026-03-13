#include "swarm/swarm_bus.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace px_swarm::swarm {

static void close_if_open(int& s)
{
    if (s >= 0) {
        ::close(s);
        s = -1;
    }
}

static bool is_multicast_ip(const std::string& ip)
{
    in_addr addr{};
    if (::inet_pton(AF_INET, ip.c_str(), &addr) != 1) return false;
    const uint32_t host = ntohl(addr.s_addr);
    return host >= 0xE0000000u && host <= 0xEFFFFFFFu;
}

SwarmBus::SwarmBus() = default;

SwarmBus::~SwarmBus()
{
    close_if_open(_send_sock);
    close_if_open(_recv_sock);
}

bool SwarmBus::open_sender(const std::string& ip, uint16_t port)
{
    close_if_open(_send_sock);
    _send_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (_send_sock < 0) {
        std::cerr << "SwarmBus sender socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }

    std::memset(&_dst_addr, 0, sizeof(_dst_addr));
    _dst_addr.sin_family = AF_INET;
    _dst_addr.sin_port = htons(port);

    if (::inet_pton(AF_INET, ip.c_str(), &_dst_addr.sin_addr) != 1) {
        std::cerr << "SwarmBus inet_pton failed for ip=" << ip << "\n";
        close_if_open(_send_sock);
        return false;
    }

    if (is_multicast_ip(ip)) {
        const unsigned char loop = 1;
        const unsigned char ttl = 1;
        ::setsockopt(_send_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
        ::setsockopt(_send_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    }

    _has_dst = true;
    return true;
}

bool SwarmBus::open_receiver(const std::string& ip, uint16_t port)
{
    close_if_open(_recv_sock);
    _recv_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (_recv_sock < 0) {
        std::cerr << "SwarmBus receiver socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }

    int reuse = 1;
    ::setsockopt(_recv_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    ::setsockopt(_recv_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(port);

    if (::bind(_recv_sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::cerr << "SwarmBus bind() failed: " << std::strerror(errno) << "\n";
        close_if_open(_recv_sock);
        return false;
    }

    const int flags = fcntl(_recv_sock, F_GETFL, 0);
    if (flags >= 0) fcntl(_recv_sock, F_SETFL, flags | O_NONBLOCK);

    if (is_multicast_ip(ip)) {
        ip_mreq mreq{};
        if (::inet_pton(AF_INET, ip.c_str(), &mreq.imr_multiaddr) != 1) {
            std::cerr << "SwarmBus multicast inet_pton failed for ip=" << ip << "\n";
            close_if_open(_recv_sock);
            return false;
        }
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);

        if (::setsockopt(_recv_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0) {
            std::cerr << "SwarmBus IP_ADD_MEMBERSHIP failed: " << std::strerror(errno) << "\n";
            close_if_open(_recv_sock);
            return false;
        }
    }

    return true;
}

bool SwarmBus::send(const SwarmMsg& msg)
{
    if (_send_sock < 0 || !_has_dst) return false;

    // Simple text message for easy debugging:
    // "kind id target leader north east down yaw t_us flags\n"
    char buf[192];
    const int len = std::snprintf(buf, sizeof(buf),
                                  "%c %d %d %d %.3f %.3f %.3f %.2f %llu %u\n",
                                  static_cast<char>(msg.kind),
                                  msg.id, msg.target_id, msg.leader_id,
                                  msg.north_m, msg.east_m, msg.down_m, msg.yaw_deg,
                                  static_cast<unsigned long long>(msg.t_us),
                                  msg.flags);

    const ssize_t n = ::sendto(_send_sock, buf, len, 0,
                              reinterpret_cast<const sockaddr*>(&_dst_addr),
                              sizeof(_dst_addr));
    return n == len;
}

std::optional<SwarmMsg> SwarmBus::try_recv()
{
    if (_recv_sock < 0) return std::nullopt;

    char buf[256];
    const ssize_t n = ::recv(_recv_sock, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return std::nullopt;

    buf[n] = '\0';

    SwarmMsg m{};
    char kind = '\0';
    unsigned long long t_us = 0;
    unsigned int flags = 0;

    const int parsed = std::sscanf(buf, " %c %d %d %d %f %f %f %f %llu %u",
                                   &kind, &m.id, &m.target_id, &m.leader_id,
                                   &m.north_m, &m.east_m, &m.down_m, &m.yaw_deg, &t_us, &flags);
    if (parsed == 10 || parsed == 9) {
        switch (kind) {
            case 'S': m.kind = SwarmMsgKind::State; break;
            case 'E': m.kind = SwarmMsgKind::Election; break;
            case 'O': m.kind = SwarmMsgKind::Ok; break;
            case 'C': m.kind = SwarmMsgKind::Coordinator; break;
            default: return std::nullopt;
        }
        m.t_us = static_cast<uint64_t>(t_us);
        m.flags = (parsed == 10) ? static_cast<uint32_t>(flags) : 0u;
        return m;
    }

    return std::nullopt;
}

} // namespace px_swarm::swarm
