#ifndef KEYBINDS_HH
#define KEYBINDS_HH

#include <Eigen/Geometry>

#include "tomlplusplus/toml.hpp"

#include <keyboard_teleop/eigen_twist.hh>

namespace keyboard_teleop
{

const std::unordered_map<std::string, EigenTwist> TWIST_KEYS = {
    {"twist.stop", EigenTwist()},
    {"twist.linear.x[0]", EigenTwist::LinearTwist(Eigen::Vector3f::UnitX())},
    {"twist.linear.x[1]", EigenTwist::LinearTwist(-Eigen::Vector3f::UnitX())},
    {"twist.linear.y[0]", EigenTwist::LinearTwist(Eigen::Vector3f::UnitY())},
    {"twist.linear.y[1]", EigenTwist::LinearTwist(-Eigen::Vector3f::UnitY())},
    {"twist.linear.z[0]", EigenTwist::LinearTwist(Eigen::Vector3f::UnitZ())},
    {"twist.linear.z[1]", EigenTwist::LinearTwist(-Eigen::Vector3f::UnitZ())},
    {"twist.angular.x[0]", EigenTwist::AngularTwist(Eigen::Vector3f::UnitX())},
    {"twist.angular.x[1]", EigenTwist::AngularTwist(-Eigen::Vector3f::UnitX())},
    {"twist.angular.y[0]", EigenTwist::AngularTwist(Eigen::Vector3f::UnitY())},
    {"twist.angular.y[1]", EigenTwist::AngularTwist(-Eigen::Vector3f::UnitY())},
    {"twist.angular.z[0]", EigenTwist::AngularTwist(Eigen::Vector3f::UnitZ())},
    {"twist.angular.z[1]", EigenTwist::AngularTwist(-Eigen::Vector3f::UnitZ())}};

/** * @brief Retrieves a keybind from the TOML table.
 * * @param table The TOML table containing keybinds.
 * @param tableKey The key to look up in the table.
 * @return The keybind as a string.
 * @throws toml::parse_error if the key is not found in the table.
 */
char getKeybind(const toml::table &table, const std::string_view tableKey);

/** * @brief Creates a map of keybinds from the TOML table.
 * * @param table The TOML table containing keybinds.
 * @return A map of keys to unique pointers of EigenTwist objects.
 */
std::unordered_map<char, std::unique_ptr<EigenTwist>> makeTwistKeybinds(
    const toml::table &table);

}  // namespace keyboard_teleop
#endif  // KEYBINDS_HH
