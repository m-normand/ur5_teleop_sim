/**
 * @file keyboard_teleop.cpp
 * @brief Keyboard teleoperation node for UR5 robot.
 */

#include <mutex>
#include <thread>

#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "keyboard_teleop/eigen_twist.hh"
#include "keyboard_teleop/keybinds.hh"
#include "keyboard_teleop/keyboard_reader.hh"

#define PUB_RATE_HZ 60

static keyboard_teleop::EigenTwist twistCmd;
static std::mutex twistCmdMutex;
static keyboard_teleop::JointCommand jointCmd;
static std::mutex jointCmdMutex;

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
)";
}

void publishTwistCmd(ros::NodeHandle &nh)
{
    ros::Publisher cmd_vel_pub =
        nh.advertise<geometry_msgs::TwistStamped>(
            "/servo_server/delta_twist_cmds", 10);
    ros::Rate rate_hz(PUB_RATE_HZ);

    geometry_msgs::TwistStamped twistStampedMsg;
    while (ros::ok())
    {
        twistCmdMutex.lock();
        twistStampedMsg.twist = twistCmd.asMsg();
        twistCmd = keyboard_teleop::EigenTwist(); // Reset after publishing
        twistCmdMutex.unlock();

        twistStampedMsg.header.stamp = ros::Time::now();
        cmd_vel_pub.publish(twistStampedMsg);

        rate_hz.sleep();
    }
}

void publishJointCmd(ros::NodeHandle &nh)
{
    ros::Publisher joint_cmd_pub =
        nh.advertise<control_msgs::JointJog>(
            "/servo_server/delta_joint_cmds", 10);
    ros::Rate rate_hz(PUB_RATE_HZ);

    control_msgs::JointJog jointJogMsg;
    jointJogMsg.joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    };
    jointJogMsg.velocities.resize(6, 0.0);
    jointJogMsg.duration = rate_hz.expectedCycleTime().toSec();

    while (ros::ok())
    {
        jointCmdMutex.lock();
        for (size_t i = 0; i < jointCmd.jointVelocities.size(); i++)
        {
            jointJogMsg.velocities[i] = jointCmd.jointVelocities(i);
            jointCmd.jointVelocities(i) = 0.0; // Reset after publishing
        }
        jointCmdMutex.unlock();

        jointJogMsg.header.stamp = ros::Time::now();
        joint_cmd_pub.publish(jointJogMsg);

        rate_hz.sleep();
    }
}

void handleTwistKey(const char &key, const keyboard_teleop::Keybinds &keybinds)
{
    if (!keybinds.isTwistKey(key))
    {
        return;
    }

    twistCmdMutex.lock();
    twistCmd += *keybinds.twistKeybinds.at(key);
    twistCmdMutex.unlock();
}

void handleJointKey(const char &key, const keyboard_teleop::Keybinds &keybinds)
{
    if (!keybinds.isJointKey(key))
    {
        return;
    }

    jointCmdMutex.lock();
    jointCmd += *keybinds.jointKeybinds.at(key);
    jointCmdMutex.unlock();
}

void waitForTime()
{
    while (ros::Time::now().toSec() == 0.0)
    {
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;

    std::thread twistPubThread(publishTwistCmd, std::ref(nh));
    std::thread jointPubThread(publishJointCmd, std::ref(nh));

    ROS_INFO_STREAM("Loading Keybinds...");
    const keyboard_teleop::Keybinds keybinds(toml::parse_file(
        ros::package::getPath("keyboard_teleop") + "/config/keybinds.toml"));

    printInstructions();
    keyboard_teleop::KeyboardReader &keyboardReader =
        keyboard_teleop::KeyboardReader::getInstance();

    int key;
    int cnt = 0;
    while (ros::ok())
    {
        key = keyboardReader.readChar();
        handleTwistKey(char(key), keybinds);
        handleJointKey(char(key), keybinds);
    }

    twistPubThread.join();
    jointPubThread.join();

    return 0;
}
