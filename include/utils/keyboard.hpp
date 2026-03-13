#pragma once

#include <optional>
#include <string>

namespace px_swarm::utils {

/// Non-blocking keyboard for Linux terminal.
/// Supports arrow keys + WASD + Q/E + T/L + SPACE + ESC.
class Keyboard {
public:
    Keyboard();
    ~Keyboard();

    Keyboard(const Keyboard&) = delete;
    Keyboard& operator=(const Keyboard&) = delete;

    bool start();
    void stop();

    /// Returns "UP","DOWN","LEFT","RIGHT","W","A","S","D","Q","E","T","L","SPACE","ESC"
    /// or std::nullopt if no input.
    std::optional<std::string> poll();

private:
    bool _active{false};

    // opaque storage to avoid pulling termios into header if you prefer;
    // for now kept simple; implementation will include termios.
    struct Impl;
    Impl* _impl{nullptr};
};

} // namespace px_swarm::utils
