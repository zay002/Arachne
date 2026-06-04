#!/usr/bin/env bash
set -euo pipefail

if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
if [[ -f "install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "install/setup.bash"
fi

ros2 run tf2_tools view_frames
