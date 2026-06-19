#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
WITH_RVIZ="${WITH_RVIZ:-true}"
USE_GUI="${USE_GUI:-false}"
WITH_BASE_SIM="${WITH_BASE_SIM:-true}"
WITH_BASE_GUI="${WITH_BASE_GUI:-false}"
WITH_GRIPPER_SIM="${WITH_GRIPPER_SIM:-false}"
WITH_GRIPPER_GUI="${WITH_GRIPPER_GUI:-false}"
WITH_LIDAR="${WITH_LIDAR:-true}"
WITH_EE_CAMERA="${WITH_EE_CAMERA:-true}"
WITH_REAR_RACK="${WITH_REAR_RACK:-true}"
WITH_FRONT_BASKET="${WITH_FRONT_BASKET:-true}"

ARM_MOUNT_XYZ="${ARM_MOUNT_XYZ:-0.22 0.0 0.105}"
ARM_MOUNT_RPY="${ARM_MOUNT_RPY:-0.0 0.0 1.57079632679}"
EE_SUPPORT_XYZ="${EE_SUPPORT_XYZ:-0.0 0.0 0.0}"
EE_SUPPORT_RPY="${EE_SUPPORT_RPY:-0.0 0.0 2.35619449019}"
TOOL_ADAPTER_XYZ="${TOOL_ADAPTER_XYZ:--0.049334103 0.049874070 0.021816675}"
TOOL_ADAPTER_RPY="${TOOL_ADAPTER_RPY:-1.570796327 0.0 0.0}"

FRONT_BASKET_XYZ="${FRONT_BASKET_XYZ:-0.4655 0.0 -0.0715}"
FRONT_BASKET_RPY="${FRONT_BASKET_RPY:-0.0 0.0 0.0}"
REAR_RACK_XYZ="${REAR_RACK_XYZ:--0.16 0.0 0.105}"
REAR_RACK_RPY="${REAR_RACK_RPY:-0.0 0.0 1.57079632679}"
LIDAR_XYZ="${LIDAR_XYZ:-0.0 0.035 0.6223}"
LIDAR_RPY="${LIDAR_RPY:-0.0 0.0 0.0}"
EE_CAMERA_XYZ="${EE_CAMERA_XYZ:-0.0 -0.0741 0.005}"
EE_CAMERA_RPY="${EE_CAMERA_RPY:-0.0 0.0 1.57079632679}"

GRIPPER_SIM_PROFILE="${GRIPPER_SIM_PROFILE:-${GRIPPER_TYPE}}"
GRIPPER_CLOSED_POSITION="${GRIPPER_CLOSED_POSITION:--1.0}"
GRIPPER_OPEN_POSITION="${GRIPPER_OPEN_POSITION:--1.0}"
GRIPPER_MAX_VELOCITY="${GRIPPER_MAX_VELOCITY:--1.0}"
BASE_LINEAR_SPEED="${BASE_LINEAR_SPEED:-0.25}"
BASE_ANGULAR_SPEED="${BASE_ANGULAR_SPEED:-0.65}"
BASE_MAX_LINEAR_VELOCITY="${BASE_MAX_LINEAR_VELOCITY:-0.8}"
BASE_MAX_ANGULAR_VELOCITY="${BASE_MAX_ANGULAR_VELOCITY:-1.4}"

usage() {
  cat <<'EOF'
Usage: ./scripts/model/view_model.sh [--help]

Primary model-inspection entrypoint. Starts the Arachne display launch with the
configured gripper, optional RViz, base simulation, sensor links, and front basket.

Common environment overrides:
  GRIPPER_TYPE=ms42dc|ag95
  WITH_RVIZ=true|false
  USE_GUI=true|false
  WITH_BASE_SIM=true|false
  WITH_GRIPPER_SIM=true|false
  WITH_LIDAR=true|false
  WITH_EE_CAMERA=true|false
  WITH_FRONT_BASKET=true|false

This is a mock/sim visualization path and does not command real hardware.
EOF
}

case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac

cleanup_model_view() {
  local patterns=(
    "[r]os2 launch arachne_description display.launch.py"
    "[r]viz2.*arachne_model\\.rviz"
    "[_]_node:=arachne_display_robot_state_publisher"
    "[_]_node:=default_joint_state_publisher"
    "[_]_node:=joint_state_publisher_gui"
    "[a]rachne_gripper/lib/arachne_gripper/joint_state_mux"
    "[a]rachne_sim/lib/arachne_sim/base_sim_controller"
    "[a]rachne_sim/lib/arachne_sim/base_teleop_gui"
    "[a]rachne_sim/lib/arachne_sim/end_effector_direction_markers"
    "[s]tatic_transform_publisher.*camera_depth_optical_frame"
    "[a]rachne_gripper/lib/arachne_gripper/gripper_sim_controller"
    "[a]rachne_gripper/lib/arachne_gripper/gripper_state_gui"
    "[u]rban_trash_map_to_odom"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" >/dev/null 2>&1 || true
  done
}

cleanup_model_view
ros2 daemon stop >/dev/null 2>&1 || true

arachne_source_ros_setup
arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first."

exec ros2 launch arachne_description display.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  use_gui:="${USE_GUI}" \
  with_rviz:="${WITH_RVIZ}" \
  with_base_sim:="${WITH_BASE_SIM}" \
  with_base_gui:="${WITH_BASE_GUI}" \
  with_gripper_sim:="${WITH_GRIPPER_SIM}" \
  with_gripper_gui:="${WITH_GRIPPER_GUI}" \
  with_lidar:="${WITH_LIDAR}" \
  with_ee_camera:="${WITH_EE_CAMERA}" \
  with_rear_rack:="${WITH_REAR_RACK}" \
  with_front_basket:="${WITH_FRONT_BASKET}" \
  display_robot_description_topic:=/arachne/display/robot_description \
  display_joint_states_topic:=/arachne/display/joint_states \
  arm_mount_xyz:="${ARM_MOUNT_XYZ}" \
  arm_mount_rpy:="${ARM_MOUNT_RPY}" \
  ee_support_xyz:="${EE_SUPPORT_XYZ}" \
  ee_support_rpy:="${EE_SUPPORT_RPY}" \
  tool_adapter_xyz:="${TOOL_ADAPTER_XYZ}" \
  tool_adapter_rpy:="${TOOL_ADAPTER_RPY}" \
  front_basket_xyz:="${FRONT_BASKET_XYZ}" \
  front_basket_rpy:="${FRONT_BASKET_RPY}" \
  rear_rack_xyz:="${REAR_RACK_XYZ}" \
  rear_rack_rpy:="${REAR_RACK_RPY}" \
  lidar_xyz:="${LIDAR_XYZ}" \
  lidar_rpy:="${LIDAR_RPY}" \
  ee_camera_xyz:="${EE_CAMERA_XYZ}" \
  ee_camera_rpy:="${EE_CAMERA_RPY}" \
  gripper_sim_profile:="${GRIPPER_SIM_PROFILE}" \
  gripper_closed_position:="${GRIPPER_CLOSED_POSITION}" \
  gripper_open_position:="${GRIPPER_OPEN_POSITION}" \
  gripper_max_velocity:="${GRIPPER_MAX_VELOCITY}" \
  base_linear_speed:="${BASE_LINEAR_SPEED}" \
  base_angular_speed:="${BASE_ANGULAR_SPEED}" \
  base_max_linear_velocity:="${BASE_MAX_LINEAR_VELOCITY}" \
  base_max_angular_velocity:="${BASE_MAX_ANGULAR_VELOCITY}"
