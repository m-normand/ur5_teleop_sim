FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    wget \
    libeigen3-dev \
    ros-noetic-control-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/keyboard_teleop/
RUN wget -q -O /ws/src/keyboard_teleop/include/tomlplusplus/toml.hpp \
    https://raw.githubusercontent.com/marzer/tomlplusplus/v3.4.0/toml.hpp

WORKDIR /ws
RUN . /opt/ros/noetic/setup.sh && catkin build --no-status
