#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include "ur5_ik/kinematic_model.hh"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_ik_node");
    ros::NodeHandle nh;

    toml::table cfg = toml::parse_file(ros::package::getPath("ur5_ik") +
                                       "/config/ur5_kinematics.toml");

    ur5_ik::KinematicModel model = ur5_ik::loadKinematicModel(cfg);
    ur5_ik::printKinematicModel(model);
}
