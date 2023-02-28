ARG ROS_DISTRO=humble
ARG PREFIX=

FROM ros:$ROS_DISTRO-ros-base AS pkg-builder

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY ./joy2twist ./src/joy2twist

# Update Ubuntu Software repository and initialise ROS workspace
RUN apt update -y && apt upgrade -y &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

# select bash as default shell
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-joy-linux && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY --from=pkg-builder /ros2_ws /ros2_ws

RUN echo $(cat /ros2_ws/src/joy2twist/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt

