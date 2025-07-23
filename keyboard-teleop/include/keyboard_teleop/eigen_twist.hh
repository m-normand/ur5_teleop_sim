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
        : linear_(Eigen::Vector3f::Zero()), angular_(Eigen::Vector3f::Zero())
    {
    }
    EigenTwist(const Eigen::Vector3f &lin, const Eigen::Vector3f &ang)
        : linear_(lin), angular_(ang)
    {
    }

    static EigenTwist LinearTwist(const Eigen::Vector3f &lin)
    {
        return {lin, Eigen::Vector3f::Zero()};
    }

    static EigenTwist AngularTwist(const Eigen::Vector3f &ang)
    {
        return {Eigen::Vector3f::Zero(), ang};
    }

    EigenTwist operator+(const EigenTwist &other) const
    {
        EigenTwist result;
        result.linear_  = this->linear_ + other.linear_;
        result.angular_ = this->angular_ + other.angular_;
        return result;
    }

    EigenTwist operator*(const float &scale) const
    {
        EigenTwist result;
        result.linear_  = this->linear_ * scale;
        result.angular_ = this->angular_ * scale;
        return result;
    }

    EigenTwist operator+=(const EigenTwist &other)
    {
        this->linear_ += other.linear_;
        this->angular_ += other.angular_;
        return *this;
    }

    geometry_msgs::Twist asMsg() const
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
    Eigen::Vector3f linear_;
    Eigen::Vector3f angular_;
};

}  // namespace keyboard_teleop

#endif  // EIGEN_TWIST_HH
