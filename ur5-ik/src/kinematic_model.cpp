#include <ur5_ik/kinematic_model.hh>

namespace ur5_ik
{
Eigen::Vector3f loadVector(const toml::node_view<const toml::node> &node)
{
    if (!node.is_array() || node.as_array()->size() != 3)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f    vec;
    const toml::array &arr = *node.as_array();
    for (size_t i = 0; i < 3; i++)
    {
        vec[i] = arr.at(i).value_or(0.0f);
    }
    return vec;
}

std::vector<ScrewAxis> loadScrewAxes(const toml::table &cfg)
{
    std::vector<ScrewAxis> screwAxes;
    for (const auto &axes : *cfg["screw_axes"].as_array())
    {
        screwAxes.push_back(ScrewAxis{*axes.at_path("name").value<std::string>(),
                                      loadVector(axes.at_path("angular")),
                                      loadVector(axes.at_path("linear"))});
    }

    return screwAxes;
}

Eigen::Quaternionf loadQuaternion(const toml::node_view<const toml::node> &node)
{
    if (!node.is_array() || node.as_array()->size() != 4)
    {
        return Eigen::Quaternionf::Identity();
    }

    Eigen::Quaternionf quat;
    const toml::array &arr = *node.as_array();
    for (size_t i = 0; i < 4; i++)
    {
        quat.coeffs()[i] = arr.at(i).value_or(0.0f);
    }
    return quat;
}

Eigen::Matrix4f loadHomeTransform(const toml::table &cfg)
{
    Eigen::Matrix4f home = Eigen::Matrix4f::Identity();

    if (const auto &homePose = cfg["home_pose"]; homePose)
    {
        home.block<3, 1>(0, 3) = loadVector(homePose.at_path("position"));
        home.block<3, 3>(0, 0) =
            loadQuaternion(homePose.at_path("orientation")).toRotationMatrix();
    }
    return home;
}

KinematicModel loadKinematicModel(const toml::table &cfg)
{
    return {loadScrewAxes(cfg), loadHomeTransform(cfg)};
}

void printScrewAxes(const std::vector<ScrewAxis> &screwAxes)
{
    std::cout << std::fixed << std::setprecision(3);
    for (const auto &axis : screwAxes)
    {
        std::cout << " - " << axis.name << "\n\tAngular: [" << axis.angular.x()
                  << ", " << axis.angular.y() << ", " << axis.angular.z() << "]"
                  << "\n\tLinear:  [" << axis.linear.x() << ", "
                  << axis.linear.y() << ", " << axis.linear.z() << "]"
                  << std::endl;
    }
}

void printTransform(const Eigen::Matrix4f &transform)
{
    for (int i = 0; i < 4; ++i)
    {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  [";

        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::setw(10) << transform(i, j) << " ";
        }

        std::cout << "] " << std::endl;
    }
}

void printKinematicModel(const KinematicModel &model)
{
    std::cout << "Screw Axes:" << std::endl;
    printScrewAxes(model.screwAxes);
    std::cout << "Home Transform:" << std::endl;
    printTransform(model.homeTransform);
}

}  // namespace ur5_ik
