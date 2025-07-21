/**
 * @file keyboard_teleop.cpp
 * @brief Keyboard teleoperation node for UR5 robot.
 */

#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "keyboard_teleop/eigen_twist.hh"
#include "keyboard_teleop/keybinds.hh"
#include "tomlplusplus/toml.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;

    toml::table tbl;
    try
    {
        tbl = toml::parse_file(ros::package::getPath("keyboard_teleop") +
                               "/config/keybinds.toml");
    }
    catch (const toml::parse_error &err)
    {
        ROS_ERROR_STREAM("Parsing failed:\n" << err << "\n");
        return 1;
    }

    ROS_INFO_STREAM("Keybinds are makin...");
    const auto twistKeybinds = keyboard_teleop::makeTwistKeybinds(tbl);

    // Print the unordered map
    std::cout << "Twist Keybinds:" << std::endl;
    geometry_msgs::Twist twistMsg;
    for (const auto &pair : twistKeybinds)
    {
        twistMsg = pair.second->asMsg();
        std::cout << pair.first << " -> [" << twistMsg.linear.x << ", "
                  << twistMsg.linear.y << ", " << twistMsg.linear.z << ", "
                  << twistMsg.angular.x << ", " << twistMsg.angular.y << ", "
                  << twistMsg.angular.z << "]" << std::endl;
    }
}
