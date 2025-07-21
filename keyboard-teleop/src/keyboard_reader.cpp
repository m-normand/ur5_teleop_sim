#include "keyboard_teleop/keyboard_reader.hh"

namespace keyboard_teleop
{
KeyboardReader::KeyboardReader()
{
    // Save original terminal settings
    tcgetattr(STDIN_FILENO, &orig_termios_);
    orig_fl_ = fcntl(STDIN_FILENO, F_GETFL);
    // Set new terminal settings
    struct termios new_termios = orig_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, orig_fl_ | O_NONBLOCK);
}

KeyboardReader::~KeyboardReader()
{
    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    fcntl(STDIN_FILENO, F_SETFL, orig_fl_);
}

KeyboardReader& KeyboardReader::getInstance()
{
    static KeyboardReader instance;
    return instance;
}

int KeyboardReader::readChar() const
{
    int c;
    int n = read(STDIN_FILENO, &c, 1);
    if (n == 1)
        return c;
    else
        return -1;  // Indicate no character was read
}

}  // namespace keyboard_teleop
