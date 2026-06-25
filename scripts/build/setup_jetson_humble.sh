#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export ROS_DISTRO=humble
export ARACHNE_SETUP_WITH_GAZEBO="${ARACHNE_SETUP_WITH_GAZEBO:-false}"
export ARACHNE_SETUP_ROS_DESKTOP="${ARACHNE_SETUP_ROS_DESKTOP:-false}"

echo "Setting up Arachne for Jetson / Ubuntu 22.04 / ROS 2 Humble."
echo "Using a lean RViz/MoveIt/Nav2 install; set ARACHNE_SETUP_WITH_GAZEBO=true to add Gazebo."

exec "${ROOT_DIR}/scripts/build/setup_ubuntu.sh" "$@"
