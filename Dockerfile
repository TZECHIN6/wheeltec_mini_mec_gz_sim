# ===================================
# Stage: base
# ===================================
ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV GZ_VERSION=harmonic
ENV ROS_DISTRO=$ROS_DISTRO
ENV USER=$USERNAME
ENV HOME=/home/$USERNAME
ENV TERM=xterm-256color

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID > /dev/null 2>&1 ; then userdel -rf $(id -un $USER_UID) ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-image-pipeline \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN mkdir -p $HOME/ros2_ws/src
WORKDIR $HOME/ros2_ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc

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
COPY src $HOME/ros2_ws/src
RUN sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build \
    && sudo rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/bin/bash", "-c", "exec $HOME/ros2_ws/src/wheeltec_mini_mec_gz_sim/ws_entrypoint.sh"]
CMD ["bash"]
