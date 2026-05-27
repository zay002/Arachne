#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEMO_MODE="${DEMO_MODE:-rviz}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
INPUT_BACKEND="${INPUT_BACKEND:-auto}"
WEB_GAMEPAD_HOST="${WEB_GAMEPAD_HOST:-127.0.0.1}"
WEB_GAMEPAD_PORT="${WEB_GAMEPAD_PORT:-8787}"

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

GZ_RESOURCE_DIRS=()
for package_name in arachne_description aubo_description scout_description dh_ag95_description; do
  package_share_parent="${ROOT_DIR}/install/${package_name}/share"
  if [[ -d "${package_share_parent}" ]]; then
    GZ_RESOURCE_DIRS+=("${package_share_parent}")
  fi
done
if [[ ${#GZ_RESOURCE_DIRS[@]} -gt 0 ]]; then
  GZ_RESOURCE_PATH="$(IFS=:; echo "${GZ_RESOURCE_DIRS[*]}")${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
  export GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_PATH}"
fi

IS_WSL=false
if [[ -r /proc/sys/kernel/osrelease ]] && grep -qi "microsoft\\|wsl" /proc/sys/kernel/osrelease; then
  IS_WSL=true
fi

case "${INPUT_BACKEND}" in
  auto)
    if [[ "${IS_WSL}" == "true" || ! -e "${JOY_DEV}" ]]; then
      WITH_JOY=false
      WITH_WEB_GAMEPAD=true
    else
      WITH_JOY=true
      WITH_WEB_GAMEPAD=false
    fi
    ;;
  joy)
    WITH_JOY=true
    WITH_WEB_GAMEPAD=false
    ;;
  web)
    WITH_JOY=false
    WITH_WEB_GAMEPAD=true
    ;;
  both)
    WITH_JOY=true
    WITH_WEB_GAMEPAD=true
    ;;
  *)
    echo "INPUT_BACKEND must be auto, joy, web, or both. Current value: ${INPUT_BACKEND}" >&2
    exit 1
    ;;
esac

if [[ "${WITH_WEB_GAMEPAD}" == "true" ]]; then
  echo "Open http://${WEB_GAMEPAD_HOST}:${WEB_GAMEPAD_PORT} in a browser and press any gamepad button."
fi

case "${DEMO_MODE}" in
  rviz)
    exec ros2 launch arachne_demo switch_rviz_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}" \
      with_joy:="${WITH_JOY}" \
      with_web_gamepad:="${WITH_WEB_GAMEPAD}" \
      web_gamepad_host:="${WEB_GAMEPAD_HOST}" \
      web_gamepad_port:="${WEB_GAMEPAD_PORT}"
    ;;
  gazebo)
    exec ros2 launch arachne_demo switch_gazebo_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}" \
      with_joy:="${WITH_JOY}" \
      with_web_gamepad:="${WITH_WEB_GAMEPAD}" \
      web_gamepad_host:="${WEB_GAMEPAD_HOST}" \
      web_gamepad_port:="${WEB_GAMEPAD_PORT}"
    ;;
  *)
    echo "DEMO_MODE must be 'rviz' or 'gazebo'. Current value: ${DEMO_MODE}" >&2
    exit 1
    ;;
esac
