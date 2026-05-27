#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
USE_GUI="${USE_GUI:-true}"
WITH_GRIPPER_SIM="${WITH_GRIPPER_SIM:-true}"
WITH_GRIPPER_GUI="${WITH_GRIPPER_GUI:-true}"
GRIPPER_SIM_PROFILE="${GRIPPER_SIM_PROFILE:-${GRIPPER_TYPE}}"
GRIPPER_CLOSED_POSITION="${GRIPPER_CLOSED_POSITION:--1.0}"
GRIPPER_OPEN_POSITION="${GRIPPER_OPEN_POSITION:--1.0}"
GRIPPER_MAX_VELOCITY="${GRIPPER_MAX_VELOCITY:--1.0}"

pkill -f '/opt/ros/.*/rviz2' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher_gui' 2>/dev/null || true
pkill -f 'joint_state_mux' 2>/dev/null || true
pkill -f 'gripper_sim_controller' 2>/dev/null || true
pkill -f 'gripper_state_gui' 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run the README build command first." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u

exec ros2 launch arachne_description display.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  use_gui:="${USE_GUI}" \
  with_gripper_sim:="${WITH_GRIPPER_SIM}" \
  with_gripper_gui:="${WITH_GRIPPER_GUI}" \
  gripper_sim_profile:="${GRIPPER_SIM_PROFILE}" \
  gripper_closed_position:="${GRIPPER_CLOSED_POSITION}" \
  gripper_open_position:="${GRIPPER_OPEN_POSITION}" \
  gripper_max_velocity:="${GRIPPER_MAX_VELOCITY}"
