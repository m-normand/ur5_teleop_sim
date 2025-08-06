FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    libeigen3-dev \
    ros-noetic-control-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/ur5_ik/
WORKDIR /ws
RUN . /opt/ros/noetic/setup.sh && catkin build --no-status
