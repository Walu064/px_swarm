#include "app/cli.hpp"
#include "app/mission.hpp"
#include "control/follower_controller.hpp"
#include "control/leader_controller.hpp"
#include "control/px4_interface.hpp"
#include "swarm/swarm_bus.hpp"
#include "swarm/swarm_state.hpp"
#include "utils/keyboard.hpp"
#include "utils/time.hpp"

#include <atomic>
#include <algorithm>
#include <csignal>
#include <cmath>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>

using namespace px_swarm;

static std::atomic<bool> g_running{true};
static void on_sigint(int) { g_running = false; }
static const char* command_mode_to_string(const control::CommandMode mode)
{
    switch (mode) {
        case control::CommandMode::VelocityBody: return "vel_body";
        case control::CommandMode::VelocityNed: return "vel_ned";
        case control::CommandMode::PositionNed: return "pos_ned";
        default: return "none";
    }
}

static std::string trim_copy(std::string s)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    const auto first = std::find_if(s.begin(), s.end(), not_space);
    if (first == s.end()) return {};
    const auto last = std::find_if(s.rbegin(), s.rend(), not_space).base();
    return std::string(first, last);
}

static std::string sanitize_tag(std::string s)
{
    for (char& c : s) {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (std::isalnum(uc)) continue;
        c = '_';
    }
    while (!s.empty() && s.front() == '_') s.erase(s.begin());
    while (!s.empty() && s.back() == '_') s.pop_back();
    return s.empty() ? std::string("flight") : s;
}

static std::string timestamp_tag_now()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    app::CliArgs args{};
    try {
        args = app::parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Arg parse error: " << e.what() << "\n";
        return 2;
    }

    std::cout << "px_swarm_node: role=" << app::role_to_string(args.role)
              << " id=" << args.vehicle_id
              << " udp_port=" << args.udp_port
              << " swarm=" << args.swarm_ip << ":" << args.swarm_port << "\n";

    std::optional<app::MissionPlan> mission{};
    if (args.role == app::Role::Leader && !args.mission_file.empty()) {
        try {
            mission = app::load_mission_yaml(args.mission_file);
            std::cout << "Loaded mission: " << mission->name
                      << " waypoints=" << mission->waypoints.size() << "\n";
            if (args.mission_leader_max_speed_m_s > 0.0f) {
                std::cout << "Mission leader speed cap: " << args.mission_leader_max_speed_m_s << " m/s\n";
            } else {
                std::cout << "Mission leader speed cap: disabled\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Mission load failed: " << e.what() << "\n";
            return 2;
        }
    }

    // PX4/MAVSDK
    control::Px4Interface px4;
    if (!px4.connect_udp(args.udp_port)) return 1;

    const auto takeoff_to_offboard = [&px4, &args]() -> bool {
        if (!px4.arm_and_takeoff(args.takeoff_alt_m)) return false;

        if (!px4.wait_until_takeoff_altitude(args.takeoff_alt_m, std::chrono::milliseconds(15000))) {
            std::cerr << "Timed out waiting to reach takeoff altitude, continuing to offboard\n";
        }

        // Offboard requires a stream of initial setpoints before start.
        for (int i = 0; i < 20; ++i) {
            px4.set_velocity_body(0.f, 0.f, 0.f, 0.f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        for (int attempt = 0; attempt < 10; ++attempt) {
            if (px4.start_offboard()) return true;
            px4.set_velocity_body(0.f, 0.f, 0.f, 0.f);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    };

    if (!takeoff_to_offboard()) return 1;

    // swarm comms
    swarm::SwarmBus bus;
    if (!bus.open_sender(args.swarm_ip, args.swarm_port)) {
        std::cerr << "Failed to open swarm sender\n";
        return 1;
    }
    if (!bus.open_receiver(args.swarm_ip, args.swarm_port)) {
        std::cerr << "Failed to open swarm receiver\n";
        return 1;
    }

    swarm::SwarmState swarm_state;
    bool is_leader = (args.role == app::Role::Leader);
    swarm_state.set_leader_id(is_leader ? args.vehicle_id : 0);
    swarm_state.set_stale_after(std::chrono::milliseconds(6000));

    utils::Keyboard kb;
    const bool keyboard_enabled = (args.role == app::Role::Leader);
    bool keyboard_active = false;

    control::LeaderController leader(control::LeaderParams{
        args.leader_v_xy, args.leader_v_z, args.leader_yaw_rate
    });

    control::FollowerController follower(
        args.vehicle_id,
        control::FollowerParams{
            args.follow_update_hz,
            args.control_hz,
            3.0f,     // max_speed (unused in position-mode)
            0.8f,     // kp
            args.safe_radius_m,
            args.formation_spacing_m,
            args.cohesion_radius_m,
            6.0f,     // search_xy
            2.0f      // search_z
        },
        swarm::PsoParams{
            20, 15,   // particles, iters
            0.6f, 1.4f, 1.4f,
            42u + static_cast<unsigned>(args.vehicle_id) // different seed per agent
        }
    );

    if (keyboard_enabled) {
        keyboard_active = kb.start();
        if (!keyboard_active) std::cerr << "Keyboard start failed (run in a real terminal)\n";
        std::cout << "Leader controls: L land, arrows (fwd/back + yaw), "
                     "WASD (strafe/alt), Q/E yaw, SPACE hover, Ctrl+C exit\n";
    }

    if (!is_leader) std::cout << "Follower running PSO target selection.\n";

    std::ofstream csv_log;
    if (args.csv_log_enabled) {
        try {
            const auto base_log_dir = std::filesystem::path(args.log_dir);
            std::filesystem::create_directories(base_log_dir);

            const auto active_flight_file = base_log_dir / ".active_flight";
            std::string flight_tag = trim_copy(args.flight_tag);
            if (flight_tag.empty() && is_leader) {
                std::string mission_tag = "manual";
                if (mission && !mission->name.empty()) {
                    mission_tag = sanitize_tag(mission->name);
                } else if (!args.mission_file.empty()) {
                    mission_tag = sanitize_tag(std::filesystem::path(args.mission_file).stem().string());
                }
                flight_tag = mission_tag + "_" + timestamp_tag_now();
            } else if (flight_tag.empty()) {
                std::ifstream in(active_flight_file);
                if (in) {
                    std::string line;
                    std::getline(in, line);
                    flight_tag = trim_copy(line);
                }
                if (flight_tag.empty()) {
                    flight_tag = "adhoc_" + timestamp_tag_now();
                }
            }

            if (is_leader) {
                std::ofstream out(active_flight_file, std::ios::out | std::ios::trunc);
                if (out) out << flight_tag << "\n";
            }

            const auto run_log_dir = base_log_dir / flight_tag;
            std::filesystem::create_directories(run_log_dir);
            const auto log_path = run_log_dir / ("px_swarm_node_" + std::to_string(args.vehicle_id) + ".csv");
            csv_log.open(log_path, std::ios::out | std::ios::trunc);
            if (!csv_log) {
                std::cerr << "Failed to open csv log file: " << log_path << "\n";
            } else {
                csv_log
                    << "t_us,node_id,is_leader,leader_id,leader_changed,leader_visible,"
                    << "north_m,east_m,down_m,yaw_deg,vel_n_m_s,vel_e_m_s,vel_d_m_s,"
                    << "neighbor_count,min_dist_m,avg_dist_m,max_dist_m,below_min_count,above_max_count,"
                    << "avg_rx_delay_ms,max_rx_delay_ms,"
                    << "cmd_mode,cmd_a,cmd_b,cmd_c,cmd_d,cmd_delta_norm,"
                    << "viol_below_ratio,viol_above_ratio,"
                    << "pso_particles,pso_iterations,pso_w,pso_c1,pso_c2,"
                    << "pso_solved,pso_cached,pso_best_fitness,pso_neighbors,"
                    << "pso_desired_n,pso_desired_e,pso_desired_d,"
                    << "pso_target_n,pso_target_e,pso_target_d,"
                    << "pso_desired_target_err_m,pso_current_target_err_m,"
                    << "key\n";
                std::cout << "CSV logging enabled: " << log_path << "\n";
                std::cout << "Flight log directory: " << run_log_dir << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "CSV log init failed: " << e.what() << "\n";
        }
    }

    int previous_leader_id = swarm_state.leader_id();
    std::optional<control::LastCommand> previous_command{};
    std::unordered_map<int, double> last_rx_delay_ms_by_id{};
    auto last_tx_print = std::chrono::steady_clock::time_point{};
    std::unordered_map<int, std::chrono::steady_clock::time_point> last_rx_print_by_id{};
    std::size_t mission_wp_index = 0;
    bool mission_hold_active = false;
    auto mission_hold_deadline = std::chrono::steady_clock::time_point{};
    bool mission_completed = false;
    bool mission_started = false;
    bool landing_requested = false;
    bool landing_command_sent = false;
    int landing_stable_ticks = 0;
    int mission_ready_stable_ticks = 0;
    auto last_mission_wait_print = std::chrono::steady_clock::time_point{};

    const double hz = std::max(1.0f, args.control_hz);
    const auto dt = std::chrono::duration<double>(1.0 / hz);

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t now_us = utils::now_us();
        swarm_state.prune(now);

        // publish our pose
        if (auto pose = px4.pose_ned()) {
            swarm::SwarmMsg m{};
            m.kind = swarm::SwarmMsgKind::State;
            m.id = args.vehicle_id;
            m.target_id = -1;
            m.leader_id = swarm_state.leader_id();
            m.north_m = pose->north_m;
            m.east_m = pose->east_m;
            m.down_m = pose->down_m;
            m.yaw_deg = pose->yaw_deg;
            m.t_us = utils::now_us();
            m.flags = (mission && landing_requested) ? swarm::SwarmMsg::FlagMissionLanding : 0u;
            bus.send(m);

            if (last_tx_print.time_since_epoch().count() == 0
                || now - last_tx_print >= std::chrono::milliseconds(500)) {
                std::cout << "[swarm] sent state from drone " << m.id
                          << " leader=" << m.leader_id
                          << " pos=(" << m.north_m << ", " << m.east_m << ", " << m.down_m << ")"
                          << ((m.flags & swarm::SwarmMsg::FlagMissionLanding) ? " [mission_landing]" : "")
                          << "\n";
                last_tx_print = now;
            }
        }

        // ingest incoming swarm messages
        while (auto msg = bus.try_recv()) {
            if (msg->id == args.vehicle_id) continue;

            if (msg->kind == swarm::SwarmMsgKind::State) {
                if (msg->t_us > 0 && now_us >= msg->t_us) {
                    const double delay_ms = static_cast<double>(now_us - msg->t_us) / 1000.0;
                    last_rx_delay_ms_by_id[msg->id] = delay_ms;
                }
                swarm_state.ingest(*msg, now);
                if (msg->id == msg->leader_id && msg->leader_id >= 0) {
                    swarm_state.set_leader_id(msg->leader_id);
                    if (!is_leader && (msg->flags & swarm::SwarmMsg::FlagMissionLanding) && !landing_requested) {
                        landing_requested = true;
                        std::cout << "[swarm] Leader requested mission landing, follower landing...\n";
                    }
                }

                const auto it = last_rx_print_by_id.find(msg->id);
                const bool should_print = (it == last_rx_print_by_id.end())
                    || (now - it->second >= std::chrono::milliseconds(500));
                if (should_print) {
                    std::cout << "[swarm] received state from drone " << msg->id
                              << " leader=" << msg->leader_id
                              << " pos=(" << msg->north_m << ", " << msg->east_m << ", " << msg->down_m << ")"
                              << " delay_ms=" << last_rx_delay_ms_by_id[msg->id]
                              << ((msg->flags & swarm::SwarmMsg::FlagMissionLanding) ? " [mission_landing]" : "")
                              << "\n";
                    last_rx_print_by_id[msg->id] = now;
                }
                continue;
            }
        }

        std::string key_for_log{};
        // role tick
        if (is_leader) {
            const auto key = keyboard_active ? kb.poll() : std::nullopt;
            if (key) {
                key_for_log = *key;
                if (*key == "ESC") {
                    g_running = false;
                } else if (*key == "L") {
                    landing_requested = true;
                    mission_completed = true;
                }
            }

            const bool manual_control = key.has_value();
            if (mission && !manual_control && !mission_completed && !landing_requested) {
                if (auto pose = px4.pose_ned()) {
                    auto at_takeoff_alt = [&](float down_m) {
                        return std::fabs(down_m + args.takeoff_alt_m) <= 1.0f;
                    };

                    const auto neighbors = swarm_state.all(args.vehicle_id);
                    int ready_count = at_takeoff_alt(pose->down_m) ? 1 : 0; // self
                    for (const auto& nb : neighbors) {
                        if (at_takeoff_alt(nb.pos_ned.z)) ++ready_count;
                    }

                    const int required_count = std::max(1, args.swarm_size);
                    const bool ready_now = (ready_count >= required_count);
                    if (!mission_started) {
                        if (ready_now) {
                            ++mission_ready_stable_ticks;
                        } else {
                            mission_ready_stable_ticks = 0;
                        }

                        if (last_mission_wait_print.time_since_epoch().count() == 0
                            || now - last_mission_wait_print >= std::chrono::seconds(1)) {
                            std::cout << "[mission] waiting for takeoff readiness: "
                                      << ready_count << "/" << required_count
                                      << " at " << args.takeoff_alt_m << "m\n";
                            last_mission_wait_print = now;
                        }

                        if (mission_ready_stable_ticks >= 20) {
                            mission_started = true;
                            std::cout << "[mission] all drones ready, starting mission.\n";
                        } else {
                            px4.set_position_ned(pose->north_m, pose->east_m, -args.takeoff_alt_m, pose->yaw_deg);
                            continue;
                        }
                    }

                    const auto& wp = mission->waypoints[mission_wp_index];
                    const double dx = static_cast<double>(pose->north_m - wp.north_m);
                    const double dy = static_cast<double>(pose->east_m - wp.east_m);
                    const double dz = static_cast<double>(pose->down_m - wp.down_m);
                    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const bool reached_wp = (dist <= 1.5);

                    if (args.mission_leader_max_speed_m_s > 0.0f) {
                        if (!reached_wp && dist > 1e-6) {
                            const double slow_radius_m = std::max(4.0, static_cast<double>(args.mission_leader_max_speed_m_s) * 4.0);
                            const float min_approach_speed_m_s = 0.4f;
                            float desired_speed_m_s = args.mission_leader_max_speed_m_s;
                            if (dist < slow_radius_m) {
                                const double alpha = std::clamp(dist / slow_radius_m, 0.0, 1.0);
                                desired_speed_m_s = std::max(
                                    min_approach_speed_m_s,
                                    static_cast<float>(args.mission_leader_max_speed_m_s * alpha));
                            }

                            const double inv_dist = 1.0 / dist;
                            const float vn = static_cast<float>((wp.north_m - pose->north_m) * inv_dist * desired_speed_m_s);
                            const float ve = static_cast<float>((wp.east_m - pose->east_m) * inv_dist * desired_speed_m_s);
                            const float vd = static_cast<float>((wp.down_m - pose->down_m) * inv_dist * desired_speed_m_s);
                            px4.set_velocity_ned(vn, ve, vd, pose->yaw_deg);
                        } else {
                            // Hold at waypoint for hold_s handling.
                            px4.set_velocity_ned(0.0f, 0.0f, 0.0f, pose->yaw_deg);
                        }
                    } else {
                        px4.set_position_ned(wp.north_m, wp.east_m, wp.down_m, 0.0f);
                    }

                    if (reached_wp) {
                        if (!mission_hold_active) {
                            mission_hold_deadline = now + std::chrono::milliseconds(static_cast<int>(wp.hold_s * 1000.0f));
                            mission_hold_active = true;
                        }

                        if (now >= mission_hold_deadline) {
                            mission_hold_active = false;
                            if (mission_wp_index + 1 < mission->waypoints.size()) {
                                ++mission_wp_index;
                                std::cout << "Mission waypoint reached, next index=" << mission_wp_index << "\n";
                            } else {
                                mission_completed = true;
                                std::cout << "Mission completed, landing...\n";
                                landing_requested = true;
                            }
                        }
                    } else {
                        mission_hold_active = false;
                    }
                }
            } else if (landing_requested) {
                if (!landing_command_sent) {
                    px4.stop_offboard();
                    px4.land_and_disarm_best_effort();
                    landing_command_sent = true;
                }
                if (auto pose = px4.pose_ned()) {
                    const bool near_ground = pose->down_m > -0.35f;
                    const bool vertical_slow = std::fabs(pose->vel_down_m_s) < 0.25f;
                    if (near_ground && vertical_slow) {
                        ++landing_stable_ticks;
                    } else {
                        landing_stable_ticks = 0;
                    }

                    if (landing_stable_ticks >= 20) {
                        std::cout << "Landing confirmed, mission finished.\n";
                        g_running = false;
                    }
                }
            } else {
                leader.tick(px4, key);
            }
        } else {
            if (landing_requested) {
                if (!landing_command_sent) {
                    px4.stop_offboard();
                    px4.land_and_disarm_best_effort();
                    landing_command_sent = true;
                }
                if (auto pose = px4.pose_ned()) {
                    const bool near_ground = pose->down_m > -0.35f;
                    const bool vertical_slow = std::fabs(pose->vel_down_m_s) < 0.25f;
                    if (near_ground && vertical_slow) {
                        ++landing_stable_ticks;
                    } else {
                        landing_stable_ticks = 0;
                    }

                    if (landing_stable_ticks >= 20) {
                        std::cout << "Follower landing confirmed, exiting.\n";
                        g_running = false;
                    }
                }
            } else {
                follower.tick(px4, swarm_state, now);
            }
        }

        if (csv_log) {
            const auto pose = px4.pose_ned();
            const auto cmd = px4.last_command();
            const auto follower_diag = follower.diagnostics();
            const auto neighbors = swarm_state.all(args.vehicle_id);

            const bool leader_visible = static_cast<bool>(swarm_state.leader());
            const int current_leader_id = swarm_state.leader_id();
            const bool leader_changed = (current_leader_id != previous_leader_id);
            previous_leader_id = current_leader_id;

            double min_dist = std::numeric_limits<double>::quiet_NaN();
            double avg_dist = std::numeric_limits<double>::quiet_NaN();
            double max_dist = std::numeric_limits<double>::quiet_NaN();
            int below_min_count = 0;
            int above_max_count = 0;
            if (pose && !neighbors.empty()) {
                double sum = 0.0;
                min_dist = std::numeric_limits<double>::infinity();
                max_dist = 0.0;
                for (const auto& nb : neighbors) {
                    const double dx = static_cast<double>(pose->north_m - nb.pos_ned.x);
                    const double dy = static_cast<double>(pose->east_m - nb.pos_ned.y);
                    const double dz = static_cast<double>(pose->down_m - nb.pos_ned.z);
                    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    sum += d;
                    if (d < min_dist) min_dist = d;
                    if (d > max_dist) max_dist = d;
                    if (d < args.safe_radius_m) ++below_min_count;
                    if (d > args.cohesion_radius_m) ++above_max_count;
                }
                avg_dist = sum / static_cast<double>(neighbors.size());
            }
            const double viol_below_ratio = neighbors.empty()
                ? 0.0
                : static_cast<double>(below_min_count) / static_cast<double>(neighbors.size());
            const double viol_above_ratio = neighbors.empty()
                ? 0.0
                : static_cast<double>(above_max_count) / static_cast<double>(neighbors.size());

            double avg_rx_delay_ms = std::numeric_limits<double>::quiet_NaN();
            double max_rx_delay_ms = std::numeric_limits<double>::quiet_NaN();
            {
                double sum = 0.0;
                double max_v = 0.0;
                int count = 0;
                for (const auto& nb : neighbors) {
                    const auto it = last_rx_delay_ms_by_id.find(nb.id);
                    if (it == last_rx_delay_ms_by_id.end()) continue;
                    sum += it->second;
                    if (it->second > max_v) max_v = it->second;
                    ++count;
                }
                if (count > 0) {
                    avg_rx_delay_ms = sum / static_cast<double>(count);
                    max_rx_delay_ms = max_v;
                }
            }

            double cmd_delta_norm = std::numeric_limits<double>::quiet_NaN();
            if (cmd && previous_command && cmd->mode == previous_command->mode) {
                const double da = static_cast<double>(cmd->a - previous_command->a);
                const double db = static_cast<double>(cmd->b - previous_command->b);
                const double dc = static_cast<double>(cmd->c - previous_command->c);
                const double dd = static_cast<double>(cmd->d - previous_command->d);
                cmd_delta_norm = std::sqrt(da * da + db * db + dc * dc + dd * dd);
            }
            previous_command = cmd;

            const auto nan = std::numeric_limits<double>::quiet_NaN();
            csv_log
                << now_us << ","
                << args.vehicle_id << ","
                << (is_leader ? 1 : 0) << ","
                << current_leader_id << ","
                << (leader_changed ? 1 : 0) << ","
                << (leader_visible ? 1 : 0) << ","
                << (pose ? pose->north_m : nan) << ","
                << (pose ? pose->east_m : nan) << ","
                << (pose ? pose->down_m : nan) << ","
                << (pose ? pose->yaw_deg : nan) << ","
                << (pose ? pose->vel_north_m_s : nan) << ","
                << (pose ? pose->vel_east_m_s : nan) << ","
                << (pose ? pose->vel_down_m_s : nan) << ","
                << neighbors.size() << ","
                << min_dist << ","
                << avg_dist << ","
                << max_dist << ","
                << below_min_count << ","
                << above_max_count << ","
                << avg_rx_delay_ms << ","
                << max_rx_delay_ms << ","
                << (cmd ? command_mode_to_string(cmd->mode) : "none") << ","
                << (cmd ? cmd->a : nan) << ","
                << (cmd ? cmd->b : nan) << ","
                << (cmd ? cmd->c : nan) << ","
                << (cmd ? cmd->d : nan) << ","
                << cmd_delta_norm << ","
                << viol_below_ratio << ","
                << viol_above_ratio << ","
                << (is_leader ? 0 : follower.pso_params().particles) << ","
                << (is_leader ? 0 : follower.pso_params().iterations) << ","
                << (is_leader ? nan : follower.pso_params().w_inertia) << ","
                << (is_leader ? nan : follower.pso_params().c_cognitive) << ","
                << (is_leader ? nan : follower.pso_params().c_social) << ","
                << (is_leader ? 0 : (follower_diag.solved_this_tick ? 1 : 0)) << ","
                << (is_leader ? 0 : (follower_diag.used_cached_target ? 1 : 0)) << ","
                << (is_leader ? nan : follower_diag.best_fitness) << ","
                << (is_leader ? 0 : follower_diag.neighbor_count) << ","
                << (is_leader ? nan : follower_diag.desired.x) << ","
                << (is_leader ? nan : follower_diag.desired.y) << ","
                << (is_leader ? nan : follower_diag.desired.z) << ","
                << (is_leader ? nan : follower_diag.target.x) << ","
                << (is_leader ? nan : follower_diag.target.y) << ","
                << (is_leader ? nan : follower_diag.target.z) << ","
                << (is_leader ? nan : follower_diag.desired_to_target_m) << ","
                << (is_leader ? nan : follower_diag.current_to_target_m) << ","
                << key_for_log
                << "\n";
        }

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(dt));
    }

    if (keyboard_active) kb.stop();

    px4.stop_offboard();
    px4.land_and_disarm_best_effort();
    return 0;
}
