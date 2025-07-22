/**
 * @file keyboard_teleop.cpp
 * @brief Keyboard teleoperation node for UR5 robot.
 */

#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "keyboard_teleop/eigen_twist.hh"
#include "keyboard_teleop/keybinds.hh"
#include "keyboard_teleop/keyboard_reader.hh"
#include "tomlplusplus/toml.hpp"

void printInstructions()
{
    std::cout << "===============================================\n"
              << "Keyboard Teleoperation Instructions:\n"
              << "Use the following keys to control the robot:\n"
              << "  - 'w'/'s' for forward/backward movement\n"
              << "  - 'a'/'d' for left/right movement\n"
              << "  - 'q'/'e' for up/down movement\n"
              << "  - 'W'/'S' for angular rotation around x-axis\n"
              << "  - 'A'/'D' for angular rotation around y-axis\n"
              << "  - 'Q'/'E' for angular rotation around z-axis\n"
              << "Press 'space' to stop the robot.\n"
              << "Press [ctrl+C] to exit.\n"
              << "===============================================\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;
    ros::Rate       rate_hz(60);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(
        "/servo_server/delta_twist_cmds", 10);

    ROS_INFO_STREAM("Loading Keybinds...");
    const auto twistKeybinds =
        keyboard_teleop::makeTwistKeybinds(toml::parse_file(
            ros::package::getPath("keyboard_teleop") + "/config/keybinds.toml"));
    auto isRegisteredKey = [&twistKeybinds](char key)
    { return twistKeybinds.find(key) != twistKeybinds.end(); };

    ROS_INFO_STREAM("Starting Terminal UI...");
    keyboard_teleop::KeyboardReader &keyboardReader =
        keyboard_teleop::KeyboardReader::getInstance();

    printInstructions();
    int                         key;
    char                        keyChar;
    bool                        staleKeys = false;
    geometry_msgs::TwistStamped twistMsg;
    while (ros::ok())
    {
        twistMsg.header.stamp = ros::Time::now();

        key = keyboardReader.readChar();
        if (key == -1)
        {
            if (!staleKeys)
            {
                twistMsg.twist = geometry_msgs::Twist();
                cmd_vel_pub.publish(twistMsg);
            }
            staleKeys = true;
            rate_hz.sleep();
            continue;
        }

        keyChar = char(key);

        twistMsg.twist = (isRegisteredKey(keyChar))
                             ? twistKeybinds.at(keyChar)->asMsg()
                             : geometry_msgs::Twist();
        cmd_vel_pub.publish(twistMsg);

        staleKeys = false;
        rate_hz.sleep();
    }
}
