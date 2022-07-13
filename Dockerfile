ARG ROS_DISTRO=humble

FROM ros:$ROS_DISTRO-ros-base

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws

# Update Ubuntu Software repository
RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-joy && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Create and initialise ROS workspace

COPY ./joy2twist ./src/joy2twist

RUN mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
