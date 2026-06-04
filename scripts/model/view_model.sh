#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
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
WITH_LIDAR="${WITH_LIDAR:-true}"
WITH_EE_CAMERA="${WITH_EE_CAMERA:-true}"
WITH_REAR_RACK="${WITH_REAR_RACK:-true}"
WITH_FRONT_BASKET="${WITH_FRONT_BASKET:-true}"
FRONT_BASKET_XYZ="${FRONT_BASKET_XYZ:-0.4655 0.0 -0.0735}"
FRONT_BASKET_RPY="${FRONT_BASKET_RPY:-0.0 0.0 0.0}"
REAR_RACK_XYZ="${REAR_RACK_XYZ:--0.16 0.0 0.105}"
REAR_RACK_RPY="${REAR_RACK_RPY:-0.0 0.0 1.57079632679}"
LIDAR_XYZ="${LIDAR_XYZ:-0.0 0.035 0.6223}"
LIDAR_RPY="${LIDAR_RPY:-0.0 0.0 0.0}"
EE_CAMERA_XYZ="${EE_CAMERA_XYZ:-0.0 0.0 0.0}"
EE_CAMERA_RPY="${EE_CAMERA_RPY:-0.0 0.0 0.0}"

python3 - <<'PY'
import os
import signal
import subprocess
import time

patterns = (
    "/opt/ros/",
    "robot_state_publisher",
    "joint_state_publisher",
    "joint_state_publisher_gui",
    "joint_state_mux",
    "base_sim_controller",
    "base_teleop_gui",
    "gripper_sim_controller",
    "gripper_state_gui",
)
must_match_one = (
    "rviz2",
    "robot_state_publisher",
    "joint_state_publisher",
    "joint_state_publisher_gui",
    "joint_state_mux",
    "base_sim_controller",
    "base_teleop_gui",
    "gripper_sim_controller",
    "gripper_state_gui",
)

own = {os.getpid(), os.getppid()}

def matching_pids():
    try:
        output = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
    except subprocess.CalledProcessError:
        return []
    pids = []
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, _, args = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid in own:
            continue
        if "scripts/model/view_model.sh" in args:
            continue
        if any(token in args for token in must_match_one):
            pids.append(pid)
    return pids

for sig in (signal.SIGTERM, signal.SIGKILL):
    pids = matching_pids()
    if not pids:
        break
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    time.sleep(0.5)
PY
ros2 daemon stop 2>/dev/null || true

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
  base_linear_speed:="${BASE_LINEAR_SPEED}" \
  base_angular_speed:="${BASE_ANGULAR_SPEED}" \
  base_max_linear_velocity:="${BASE_MAX_LINEAR_VELOCITY}" \
  base_max_angular_velocity:="${BASE_MAX_ANGULAR_VELOCITY}" \
  with_gripper_sim:="${WITH_GRIPPER_SIM}" \
  with_gripper_gui:="${WITH_GRIPPER_GUI}" \
  with_lidar:="${WITH_LIDAR}" \
  with_ee_camera:="${WITH_EE_CAMERA}" \
  with_rear_rack:="${WITH_REAR_RACK}" \
  with_front_basket:="${WITH_FRONT_BASKET}" \
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
  gripper_max_velocity:="${GRIPPER_MAX_VELOCITY}"
