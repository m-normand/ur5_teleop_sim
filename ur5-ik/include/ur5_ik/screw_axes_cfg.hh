/**
 * @file kinematics_config.hh
 * @brief Defines methods to load kinematics parameters for the UR5 robot.
 */

#ifndef SCREW_AXES_CFG_HH
#define SCREW_AXES_CFG_HH

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "tomlplusplus/toml.hpp"

namespace ur5_ik
{
struct ScrewAxis
{
    std::string     name;
    Eigen::Vector3f angular;
    Eigen::Vector3f linear;
};

Eigen::Vector3f loadVector(const toml::node_view<const toml::node>& node);

std::vector<ScrewAxis> loadScrewAxes(const std::string_view& file_path);

void printScrewAxes(const std::vector<ScrewAxis>& screwAxes);

}  // namespace ur5_ik

#endif  // SCREW_AXES_CFG_HH
