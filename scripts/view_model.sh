#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/ros_env.sh"
arachne_require_ros_distro

GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
USE_GUI="${USE_GUI:-true}"
WITH_RVIZ="${WITH_RVIZ:-true}"
WITH_BASE_SIM="${WITH_BASE_SIM:-true}"
WITH_BASE_GUI="${WITH_BASE_GUI:-true}"
WITH_GRIPPER_SIM="${WITH_GRIPPER_SIM:-true}"
WITH_GRIPPER_GUI="${WITH_GRIPPER_GUI:-true}"
GRIPPER_SIM_PROFILE="${GRIPPER_SIM_PROFILE:-${GRIPPER_TYPE}}"
BASE_LINEAR_SPEED="${BASE_LINEAR_SPEED:-0.25}"
BASE_ANGULAR_SPEED="${BASE_ANGULAR_SPEED:-0.65}"
BASE_MAX_LINEAR_VELOCITY="${BASE_MAX_LINEAR_VELOCITY:-0.8}"
BASE_MAX_ANGULAR_VELOCITY="${BASE_MAX_ANGULAR_VELOCITY:-1.4}"
GRIPPER_CLOSED_POSITION="${GRIPPER_CLOSED_POSITION:--1.0}"
GRIPPER_OPEN_POSITION="${GRIPPER_OPEN_POSITION:--1.0}"
GRIPPER_MAX_VELOCITY="${GRIPPER_MAX_VELOCITY:--1.0}"

pkill -f '/opt/ros/.*/rviz2' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher_gui' 2>/dev/null || true
pkill -f 'joint_state_mux' 2>/dev/null || true
pkill -f 'base_sim_controller' 2>/dev/null || true
pkill -f 'base_teleop_gui' 2>/dev/null || true
pkill -f 'gripper_sim_controller' 2>/dev/null || true
pkill -f 'gripper_state_gui' 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true

arachne_source_ros_setup
arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Run ./scripts/build_workspace.sh first."

exec ros2 launch arachne_description display.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  use_gui:="${USE_GUI}" \
  with_rviz:="${WITH_RVIZ}" \
  with_base_sim:="${WITH_BASE_SIM}" \
  with_base_gui:="${WITH_BASE_GUI}" \
  base_linear_speed:="${BASE_LINEAR_SPEED}" \
  base_angular_speed:="${BASE_ANGULAR_SPEED}" \
  base_max_linear_velocity:="${BASE_MAX_LINEAR_VELOCITY}" \
  base_max_angular_velocity:="${BASE_MAX_ANGULAR_VELOCITY}" \
  with_gripper_sim:="${WITH_GRIPPER_SIM}" \
  with_gripper_gui:="${WITH_GRIPPER_GUI}" \
  gripper_sim_profile:="${GRIPPER_SIM_PROFILE}" \
  gripper_closed_position:="${GRIPPER_CLOSED_POSITION}" \
  gripper_open_position:="${GRIPPER_OPEN_POSITION}" \
  gripper_max_velocity:="${GRIPPER_MAX_VELOCITY}"
