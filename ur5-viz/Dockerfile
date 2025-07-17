FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    ros-noetic-ur5-moveit-config \
    ros-noetic-ur-gazebo \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m rviz-user
USER rviz-user
