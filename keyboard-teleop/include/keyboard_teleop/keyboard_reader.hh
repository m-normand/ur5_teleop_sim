#ifndef KEYBOARD_READER_HH
#define KEYBOARD_READER_HH

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace keyboard_teleop
{

/**
 * @class KeyboardReader
 * @brief A singleton to read keyboard input without echoing to the terminal.
 *
 * This class sets the terminal to raw mode to read single characters
 * without waiting for a newline and without displaying them on the screen.
 */
class KeyboardReader
{
   public:
    // Rule of Five, we need to define a destructor
    KeyboardReader(const KeyboardReader&)            = delete;
    KeyboardReader(KeyboardReader&&)                 = delete;
    KeyboardReader& operator=(const KeyboardReader&) = delete;
    KeyboardReader& operator=(KeyboardReader&&)      = delete;

    /**
     * @brief Get the singleton instance of KeyboardReader.
     * @return Reference to the singleton instance.
     */
    static KeyboardReader& getInstance();

    /**
     * @brief Read a single character from the keyboard.
     * @return The character read from the keyboard.
     */
    int readChar() const;

   private:
    termios orig_termios_;
    int     orig_fl_;

    KeyboardReader();
    ~KeyboardReader();
};

}  // namespace keyboard_teleop

#endif  // KEYBOARD_READER_HH
