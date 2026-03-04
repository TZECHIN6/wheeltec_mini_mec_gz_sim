# ===================================
# Stage: base
# ===================================
ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ENV GZ_VERSION=harmonic

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user with UID 1000 if it exists in container
RUN if id -u $USER_UID > /dev/null 2>&1; then userdel -rf $(id -un $USER_UID); fi
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ros-dev-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-rmf-dev \
    && rm -rf /var/lib/apt/lists/*

# Config ROS 2 to use cyclonedds as middleware
COPY --chown=$USERNAME src/wheeltec_mini_mec_gz_sim/cyclonedds_config_file.xml /opt/ros/cyclonedds_config_file.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///opt/ros/cyclonedds_config_file.xml

ENV USER=$USERNAME
ENV HOME=/home/$USERNAME

USER $USERNAME
RUN mkdir -p $HOME/ros2_ws/src
WORKDIR $HOME/ros2_ws

# ===================================
# Stage: development
# ===================================
FROM base AS development

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# ===================================
# Stage: deploy
# ===================================
FROM base AS deploy

COPY --chown=$USERNAME src/wheeltec_mini_mec_gz_sim ./src/wheeltec_mini_mec_gz_sim
RUN sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf build log

ENV DEBIAN_FRONTEND=

COPY --chown=$USERNAME src/wheeltec_mini_mec_gz_sim/ws_entrypoint.sh /ws_entrypoint.sh
ENTRYPOINT ["/ws_entrypoint.sh"]
CMD ["bash"]
