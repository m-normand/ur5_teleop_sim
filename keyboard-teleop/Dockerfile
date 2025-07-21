FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    libeignen3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/keyboard_teleop/
WORKDIR /ws
RUN . /opt/ros/noetic/setup.sh && catkin build --no-status
