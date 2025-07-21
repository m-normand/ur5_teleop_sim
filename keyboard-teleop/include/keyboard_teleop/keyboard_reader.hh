#ifndef KEYBOARD_READER_HH
#define KEYBOARD_READER_HH

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace keyboard_teleop
{

class KeyboardReader
{
   public:
    // Rule of Five
    KeyboardReader(const KeyboardReader&)            = delete;
    KeyboardReader(KeyboardReader&&)                 = delete;
    KeyboardReader& operator=(const KeyboardReader&) = delete;
    KeyboardReader& operator=(KeyboardReader&&)      = delete;

    static KeyboardReader& getInstance();

    int readChar() const;

   private:
    termios orig_termios_;
    int     orig_fl_;

    KeyboardReader();
    ~KeyboardReader();
};

}  // namespace keyboard_teleop

#endif  // KEYBOARD_READER_HH
