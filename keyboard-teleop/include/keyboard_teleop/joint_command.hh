#ifndef JOINT_COMMAND_HH
#define JOINT_COMMAND_HH

#include <Eigen/Dense>

namespace keyboard_teleop
{

#define N_JOINTS 6

/**
 * @brief Type alias for static vector of N_JOINTS floats.
 * This is used to represent joint velocities in a fixed-size vector.
 */
using VectorNJf = Eigen::Matrix<float, N_JOINTS, 1>;

struct JointCommand
{
    VectorNJf jointVelocities;

    JointCommand() : jointVelocities(VectorNJf::Zero()) {}
    JointCommand(const VectorNJf &velocities) : jointVelocities(velocities) {}
    JointCommand(const size_t &index, const float &velocity)
        : jointVelocities(VectorNJf::Zero())
    {
        if (index < 0 || index > N_JOINTS - 1){
            throw std::out_of_range("Index out of range for JointCommand");
        }

        jointVelocities(index) = velocity;
    }

    JointCommand operator+(const JointCommand &other) const
    {
        return JointCommand(jointVelocities + other.jointVelocities);
    }

    JointCommand operator+=(const JointCommand &other)
    {
        jointVelocities += other.jointVelocities;
        return *this;
    }
};

}  // namespace keyboard_teleop

#endif  // JOINT_COMMAND_HH
