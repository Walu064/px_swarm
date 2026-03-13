#include "utils/keyboard.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace px_swarm::utils {

struct Keyboard::Impl {
    termios orig{};
    bool ok{false};
};

Keyboard::Keyboard() : _impl(new Impl()) {}
Keyboard::~Keyboard()
{
    stop();
    delete _impl;
    _impl = nullptr;
}

bool Keyboard::start()
{
    if (!_impl) return false;
    if (_active) return true;

    if (tcgetattr(STDIN_FILENO, &_impl->orig) != 0) return false;

    termios raw = _impl->orig;
    raw.c_lflag &= ~(ICANON | ECHO);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;

    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    _impl->ok = true;
    _active = true;
    return true;
}

void Keyboard::stop()
{
    if (!_impl || !_active) return;
    if (_impl->ok) tcsetattr(STDIN_FILENO, TCSANOW, &_impl->orig);
    _active = false;
}

std::optional<std::string> Keyboard::poll()
{
    if (!_active) return std::nullopt;

    unsigned char c{};
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) return std::nullopt;

    if (c == 27) { // ESC or ANSI sequence
        unsigned char seq[2]{};
        const ssize_t n1 = ::read(STDIN_FILENO, &seq[0], 1);
        const ssize_t n2 = ::read(STDIN_FILENO, &seq[1], 1);
        if (n1 == 1 && n2 == 1 && seq[0] == '[') {
            switch (seq[1]) {
                case 'A': return "UP";
                case 'B': return "DOWN";
                case 'C': return "RIGHT";
                case 'D': return "LEFT";
            }
        }
        return "ESC";
    }

    if (c == ' ') return "SPACE";
    if (c == 'w' || c == 'W') return "W";
    if (c == 's' || c == 'S') return "S";
    if (c == 'a' || c == 'A') return "A";
    if (c == 'd' || c == 'D') return "D";
    if (c == 'q' || c == 'Q') return "Q";
    if (c == 'e' || c == 'E') return "E";
    if (c == 't' || c == 'T') return "T";
    if (c == 'l' || c == 'L') return "L";

    return std::nullopt;
}

} // namespace px_swarm::utils
