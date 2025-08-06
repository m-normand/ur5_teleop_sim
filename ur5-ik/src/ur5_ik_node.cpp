#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include "ur5_ik/screw_axes_cfg.hh"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_ik_node");
    ros::NodeHandle nh;

    std::vector<ur5_ik::ScrewAxis> screwAxes = ur5_ik::loadScrewAxes(
        ros::package::getPath("ur5_ik") + "/config/ur5_kinematics.toml");

    ur5_ik::printScrewAxes(screwAxes);
}
