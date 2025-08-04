/**
 * @file keyboard_teleop.cpp
 * @brief Keyboard teleoperation node for UR5 robot.
 */

#include <mutex>
#include <thread>

#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "keyboard_teleop/eigen_twist.hh"
#include "keyboard_teleop/keybinds.hh"
#include "keyboard_teleop/keyboard_reader.hh"

static keyboard_teleop::EigenTwist twistCmd;
static std::mutex                  twistCmdMutex;

void printInstructions()
{
    std::cout << R"(
===============================================
Keyboard Teleoperation Instructions:
Use the following keys to control the robot:
  - 'w'/'s' for forward/backward movement
  - 'a'/'d' for left/right movement
  - 'q'/'e' for up/down movement
  - 'W'/'S' for angular rotation around x-axis
  - 'A'/'D' for angular rotation around y-axis
  - 'Q'/'E' for angular rotation around z-axis
Press 'space' to stop the robot.
Press [ctrl+C] to exit.
===============================================
")";
}

void publishTwistCmd()
{
    ros::Publisher cmd_vel_pub =
        ros::NodeHandle().advertise<geometry_msgs::TwistStamped>(
            "/servo_server/delta_twist_cmds", 10);
    ros::Rate rate_hz(60);

    geometry_msgs::TwistStamped twistStampedMsg;
    while (ros::ok())
    {
        twistCmdMutex.lock();
        twistStampedMsg.twist = twistCmd.asMsg();
        twistCmd = keyboard_teleop::EigenTwist();  // Reset after publishing
        twistCmdMutex.unlock();

        twistStampedMsg.header.stamp = ros::Time::now();
        cmd_vel_pub.publish(twistStampedMsg);

        rate_hz.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");

    std::thread twistPubThread(publishTwistCmd);

    ROS_INFO_STREAM("Loading Keybinds...");
    const keyboard_teleop::Keybinds keybinds(toml::parse_file(
        ros::package::getPath("keyboard_teleop") + "/config/keybinds.toml"));

    printInstructions();
    keyboard_teleop::KeyboardReader &keyboardReader =
        keyboard_teleop::KeyboardReader::getInstance();

    int key;
    while (ros::ok())
    {
        key = keyboardReader.readChar();
        if (keybinds.isRegisteredKey(key))
        {
            twistCmdMutex.lock();
            twistCmd += *keybinds.twistKeybinds.at(char(key));
            twistCmdMutex.unlock();
        }
    }

    twistPubThread.join();
    return 0;
}
