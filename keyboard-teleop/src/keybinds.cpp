#include "keyboard_teleop/keybinds.hh"

namespace keyboard_teleop
{
std::string getKeybind(const toml::table &table, const std::string_view tableKey)
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
    else if (keybind.length() > 1)
    {
        throw std::runtime_error("Keybind " + std::string(tableKey) +
                                 " has more than one char: " + keybind +
                                 ". Please adjust keybinds.toml");
    }
    return keybind;
};

std::unordered_map<std::string, std::unique_ptr<EigenTwist>> makeTwistKeybinds(
    const toml::table &table)
{
    auto getKey = [&table](const std::string_view key)
    { return getKeybind(table, key); };

    std::unordered_map<std::string, std::unique_ptr<EigenTwist>> keybinds;
    for (const auto &[key, twist] : TWIST_KEYS)
    {
        keybinds[key] = std::make_unique<EigenTwist>(twist);
    }

    return keybinds;
};
}  // namespace keyboard_teleop
