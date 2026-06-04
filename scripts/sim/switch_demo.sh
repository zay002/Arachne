#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

DEMO_MODE="${DEMO_MODE:-gazebo}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
INPUT_BACKEND="${INPUT_BACKEND:-auto}"
WEB_GAMEPAD_HOST="${WEB_GAMEPAD_HOST:-127.0.0.1}"
WEB_GAMEPAD_PORT="${WEB_GAMEPAD_PORT:-8787}"
GAZEBO_CAMERA_DISTANCE="${GAZEBO_CAMERA_DISTANCE:-2.0}"
FORWARD_AXIS_SIGN="${FORWARD_AXIS_SIGN:--1.0}"
LATERAL_AXIS_SIGN="${LATERAL_AXIS_SIGN:-1.0}"
GZ_RENDER_BACKEND="${GZ_RENDER_BACKEND:-opengl}"
GZ_UPDATE_RATE="${GZ_UPDATE_RATE:-180}"

arachne_source_ros_setup
arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first."

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

if [[ "${IS_WSL}" == "true" ]]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
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
echo "Starting Arachne demo mode: ${DEMO_MODE}"

case "${DEMO_MODE}" in
  rviz)
    exec ros2 launch arachne_demo switch_rviz_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}" \
      with_joy:="${WITH_JOY}" \
      with_web_gamepad:="${WITH_WEB_GAMEPAD}" \
      web_gamepad_host:="${WEB_GAMEPAD_HOST}" \
      web_gamepad_port:="${WEB_GAMEPAD_PORT}" \
      forward_axis_multiplier:="${FORWARD_AXIS_SIGN}" \
      lateral_axis_multiplier:="${LATERAL_AXIS_SIGN}" \
      gazebo_camera_distance:="${GAZEBO_CAMERA_DISTANCE}"
    ;;
  gazebo)
    exec ros2 launch arachne_demo switch_gazebo_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}" \
      with_joy:="${WITH_JOY}" \
      with_web_gamepad:="${WITH_WEB_GAMEPAD}" \
      web_gamepad_host:="${WEB_GAMEPAD_HOST}" \
      web_gamepad_port:="${WEB_GAMEPAD_PORT}" \
      forward_axis_multiplier:="${FORWARD_AXIS_SIGN}" \
      lateral_axis_multiplier:="${LATERAL_AXIS_SIGN}" \
      gazebo_camera_distance:="${GAZEBO_CAMERA_DISTANCE}" \
      gazebo_render_backend:="${GZ_RENDER_BACKEND}" \
      gazebo_update_rate:="${GZ_UPDATE_RATE}"
    ;;
  *)
    echo "DEMO_MODE must be 'rviz' or 'gazebo'. Current value: ${DEMO_MODE}" >&2
    exit 1
    ;;
esac
