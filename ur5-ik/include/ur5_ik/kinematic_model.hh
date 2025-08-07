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
/**
 * @brief Represents a screw axis with a name, angular and linear components.
 */
struct ScrewAxis
{
    std::string     name;
    Eigen::Vector3f angular;
    Eigen::Vector3f linear;
};

/**
 * @brief Represents the kinematic model using Product of Exponentials
 */
struct KinematicModel
{
    std::vector<ScrewAxis> screwAxes;
    Eigen::Matrix4f        homeTransform;
};

/**
 * @brief Loads screw axes from a TOML configuration file.
 * @param cfg The TOML table containing the screw axes configuration.
 * @return A vector of ScrewAxis objects.
 */
std::vector<ScrewAxis> loadScrewAxes(const toml::table& cfg);

/**
 * @brief Loads a 3D vector from a TOML node.
 * @param node The TOML node containing the vector data.
 * @return An Eigen::Vector3f initialized with the values from the node.
 */
Eigen::Vector3f loadVector(const toml::node_view<const toml::node>& node);

/**
 * @brief Loads a quaternion from a TOML node.
 * @param node The TOML node containing the quaternion data.
 * @return An Eigen::Quaternionf initialized with the values from the node.
 */
Eigen::Quaternionf loadQuaternion(const toml::node_view<const toml::node>& node);

/**
 * @brief Loads the home transformation matrix from a TOML configuration.
 * @param cfg The TOML table containing the home pose configuration.
 * @return An Eigen::Matrix4f representing the home transformation.
 */
Eigen::Matrix4f loadHomeTransform(const toml::table& cfg);

/**
 * @brief Loads the kinematic model from a TOML configuration.
 * @param cfg The TOML table containing the kinematic model configuration.
 * @return A KinematicModel object containing screw axes and home transformation.
 */
KinematicModel loadKinematicModel(const toml::table& cfg);

/**
 * @brief Prints the screw axes to the standard output.
 * @param screwAxes A vector of ScrewAxis objects to print.
 */
void printScrewAxes(const std::vector<ScrewAxis>& screwAxes);

/**
 * @brief Prints a transformation matrix to the standard output.
 * @param transform The Eigen::Matrix4f transformation matrix to print.
 */
void printTransform(const Eigen::Matrix4f& transform);

/**
 * @brief Prints the kinematic model to the standard output.
 * @param model The KinematicModel object to print.
 */
void printKinematicModel(const KinematicModel& model);

}  // namespace ur5_ik

#endif  // SCREW_AXES_CFG_HH
