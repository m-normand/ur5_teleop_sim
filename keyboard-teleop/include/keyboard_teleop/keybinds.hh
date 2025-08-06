#ifndef KEYBINDS_HH
#define KEYBINDS_HH

#include <Eigen/Geometry>

#include "tomlplusplus/toml.hpp"

#include <keyboard_teleop/eigen_twist.hh>
#include <keyboard_teleop/joint_command.hh>

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

const std::unordered_map<std::string, JointCommand> JOINT_KEYS = {
    {"joint.stop", JointCommand()},
    {"joint.0[0]", JointCommand(0, 1)},
    {"joint.0[1]", JointCommand(0, -1)},
    {"joint.1[0]", JointCommand(1, 1)},
    {"joint.1[1]", JointCommand(1, -1)},
    {"joint.2[0]", JointCommand(2, 1)},
    {"joint.2[1]", JointCommand(2, -1)},
    {"joint.3[0]", JointCommand(3, 1)},
    {"joint.3[1]", JointCommand(3, -1)},
    {"joint.4[0]", JointCommand(4, 1)},
    {"joint.4[1]", JointCommand(4, -1)},
    {"joint.5[0]", JointCommand(5, 1)},
    {"joint.5[1]", JointCommand(5, -1)}
};

/** @brief Retrieves a keybind from the TOML table.
 * @param table The TOML table containing keybinds.
 * @param tableKey The key to look up in the table.
 * @return The keybind as a string.
 * @throws toml::parse_error if the key is not found in the table.
 */
char getKeybind(const toml::table &table, const std::string_view tableKey);

/** @brief Creates a map of twist keybinds from the TOML table.
 * @param table The TOML table containing keybinds.
 * @return A map of keys to unique pointers of EigenTwist objects.
 */
std::unordered_map<char, std::unique_ptr<EigenTwist>> makeTwistKeybinds(
    const toml::table &table);

/** @brief Creates a map of joint keybinds from the TOML table.
 * @param table The TOML table containing keybinds.
 * @return A map of keys to unique pointers of JointCommand objects.
 */
std::unordered_map<char, std::unique_ptr<JointCommand>> makeJointKeybinds(
    const toml::table &table);

struct Keybinds
{
    std::unordered_map<char, std::unique_ptr<EigenTwist>> twistKeybinds;
    std::unordered_map<char, std::unique_ptr<JointCommand>> jointKeybinds;

    bool isTwistKey(const int &key) const
    {
        return twistKeybinds.find(char(key)) != twistKeybinds.end();
    };

    bool isJointKey(const int &key) const
    {
        return jointKeybinds.find(char(key)) != jointKeybinds.end();
    };

    Keybinds(const toml::table &table)
        : twistKeybinds(makeTwistKeybinds(table)),
          jointKeybinds(makeJointKeybinds(table))
    {
    }
};

}  // namespace keyboard_teleop
#endif  // KEYBINDS_HH
