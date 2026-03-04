#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"

WS_SETUP="/home/${USER:-user}/ros2_ws/install/setup.bash"

if [ -f "$WS_SETUP" ]; then
  source "$WS_SETUP"
  echo "Workspace sourced: $WS_SETUP"
else
  echo "Workspace not built yet, skipping source."
fi

exec "$@"
