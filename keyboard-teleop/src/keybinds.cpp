#include "keyboard_teleop/keybinds.hh"

namespace keyboard_teleop
{
char getKeybind(const toml::table &table, const std::string_view tableKey)
{
    auto tableVal = table.at_path(tableKey).value<std::string>();
    if (!tableVal.has_value())
    {
        throw toml::parse_error("Could not find key",
                                table[tableKey].node()->source());
    }

    auto keybind = tableVal.value();

    if (keybind == "space")
    {
        keybind = " ";
    }
    if (keybind.length() > 1)
    {
        throw std::runtime_error("Keybind " + std::string(tableKey) +
                                 " has more than one char: " + keybind +
                                 ". Please adjust keybinds.toml");
    }
    return keybind[0];
};

std::unordered_map<char, std::unique_ptr<EigenTwist>> makeTwistKeybinds(
    const toml::table &table)
{
    auto getKey = [&table](const std::string_view key)
    { return getKeybind(table, key); };

    std::unordered_map<char, std::unique_ptr<EigenTwist>> keybinds;
    for (const auto &[key, twist] : TWIST_KEYS)
    {
        keybinds[getKey(key)] = std::make_unique<EigenTwist>(twist);
    }

    return keybinds;
};

std::unordered_map<char, std::unique_ptr<JointCommand>> makeJointKeybinds(
    const toml::table &table)
{
    auto getKey = [&table](const std::string_view key)
    { return getKeybind(table, key); };

    std::unordered_map<char, std::unique_ptr<JointCommand>> keybinds;
    for (const auto &[key, command] : JOINT_KEYS)
    {
        keybinds[getKey(key)] = std::make_unique<JointCommand>(command);
    }

    return keybinds;
};


}  // namespace keyboard_teleop
