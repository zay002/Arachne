#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
GZ_RENDER_BACKEND="${GZ_RENDER_BACKEND:-opengl}"
GZ_UPDATE_RATE="${GZ_UPDATE_RATE:-180}"
GAZEBO_CAMERA_DISTANCE="${GAZEBO_CAMERA_DISTANCE:-2.2}"

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

if [[ -r /proc/sys/kernel/osrelease ]] && grep -qi "microsoft\\|wsl" /proc/sys/kernel/osrelease; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
fi

exec ros2 launch arachne_demo gazebo_autopick_demo.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  gazebo_camera_distance:="${GAZEBO_CAMERA_DISTANCE}" \
  gazebo_render_backend:="${GZ_RENDER_BACKEND}" \
  gazebo_update_rate:="${GZ_UPDATE_RATE}"
