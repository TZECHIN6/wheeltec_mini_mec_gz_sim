ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID > /dev/null 2>&1; then \
        userdel -rf $(id -un $USER_UID); fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

ENV GZ_VERSION=harmonic
ENV ROS_DISTRO=$ROS_DISTRO
ENV USER=$USERNAME

RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

COPY src ./src
RUN sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build \
    && sudo rm -rf /var/lib/apt/lists/*

# Config ROS 2 to use cyclonedds as middleware
COPY src/wheeltec_mini_mec_gz_sim/cyclonedds_config_file.xml /opt/ros/cyclonedds_config_file.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///opt/ros/cyclonedds_config_file.xml

ENV DEBIAN_FRONTEND=

COPY src/wheeltec_mini_mec_gz_sim/ws_entrypoint.sh /ws_entrypoint.sh
ENTRYPOINT ["/ws_entrypoint.sh"]
CMD ["bash"]
