FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-moveit \
    ros-noetic-moveit-servo \
    ros-noetic-ur-gazebo \
    ros-noetic-ur5-moveit-config \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/ur5_sim/
WORKDIR /ws
RUN . /opt/ros/noetic/setup.sh && catkin build --no-status

RUN useradd -m rviz-user
USER rviz-user
