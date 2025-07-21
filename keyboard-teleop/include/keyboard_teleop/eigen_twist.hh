/**
 * @file eigen_twist.hh
 * @brief Header file for Eigen twist representation.
 */

#ifndef EIGEN_TWIST_HH
#define EIGEN_TWIST_HH

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/Twist.h>

namespace keyboard_teleop
{

/**
 * @class EigenTwist
 * @brief A class representing a twist in Eigen format.
 *
 * This consists of a linear and angular component
 */
class EigenTwist
{
   public:
    EigenTwist()
        : linear_(Eigen::Vector3i::Zero()), angular_(Eigen::Vector3i::Zero())
    {
    }
    EigenTwist(const Eigen::Vector3i &lin, const Eigen::Vector3i &ang)
        : linear_(lin), angular_(ang)
    {
    }

    static EigenTwist LinearTwist(const Eigen::Vector3i &lin)
    {
        return {lin, Eigen::Vector3i::Zero()};
    }

    static EigenTwist AngularTwist(const Eigen::Vector3i &ang)
    {
        return {Eigen::Vector3i::Zero(), ang};
    }

    EigenTwist operator+(const EigenTwist &other) const
    {
        EigenTwist result;
        result.linear_  = this->linear_ + other.linear_;
        result.angular_ = this->angular_ + other.angular_;
        return result;
    }

    geometry_msgs::Twist asMsg()
    {
        geometry_msgs::Twist msg;
        msg.linear.x  = linear_.x();
        msg.linear.y  = linear_.y();
        msg.linear.z  = linear_.z();
        msg.angular.x = angular_.x();
        msg.angular.y = angular_.y();
        msg.angular.z = angular_.z();
        return msg;
    }

   protected:
    Eigen::Vector3i linear_;
    Eigen::Vector3i angular_;
};

}  // namespace keyboard_teleop

#endif  // EIGEN_TWIST_HH
