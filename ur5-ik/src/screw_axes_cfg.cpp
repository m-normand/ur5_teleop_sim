#include <ur5_ik/screw_axes_cfg.hh>

namespace ur5_ik
{
Eigen::Vector3f loadVector(const toml::node_view<const toml::node>& node)
{
    if (!node.is_array() || node.as_array()->size() != 3)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f    vec;
    const toml::array& arr = *node.as_array();
    for (size_t i = 0; i < 3; i++)
    {
        vec[i] = arr.at(i).value_or(0.0f);
    }
    return vec;
}

std::vector<ScrewAxis> loadScrewAxes(const std::string_view& file_path)
{
    toml::table cfg = toml::parse_file(file_path);

    std::vector<ScrewAxis> screwAxes;

    for (const auto& axes : *cfg["screw_axes"].as_array())
    {
        screwAxes.push_back(ScrewAxis{*axes.at_path("name").value<std::string>(),
                                      loadVector(axes.at_path("angular")),
                                      loadVector(axes.at_path("linear"))});
    }

    return screwAxes;
}

void printScrewAxes(const std::vector<ScrewAxis>& screwAxes)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Joint Screw Axes:" << std::endl;
    for (const auto& axis : screwAxes)
    {
        std::cout << " - " << axis.name << "\n\tAngular: [" << axis.angular.x()
                  << ", " << axis.angular.y() << ", " << axis.angular.z() << "]"
                  << "\n\tLinear:  [" << axis.linear.x() << ", "
                  << axis.linear.y() << ", " << axis.linear.z() << "]"
                  << std::endl;
    }
}

}  // namespace ur5_ik
