#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
PLANNER_ID="${PLANNER_ID:-RRTConnectkConfigDefault}"
PLAYBACK_SPEED="${PLAYBACK_SPEED:-0.8}"
LOOP="${LOOP:-true}"
WITH_RVIZ="${WITH_RVIZ:-true}"
ALLOW_INTERPOLATION_FALLBACK="${ALLOW_INTERPOLATION_FALLBACK:-false}"

cleanup_stale_demo() {
  local patterns=(
    "[r]os2 launch arachne_sim moveit_grasp_planning_demo"
    "[r]viz2.*moveit_grasp_demo\\.rviz"
    "[a]rachne_moveit_grasp_demo_rviz"
    "[a]rachne_sim/lib/arachne_sim/moveit_grasp_planning_demo"
    "[m]oveit_ros_move_group/move_group"
    "[_]_node:=arachne_display_robot_state_publisher"
    "[_]_node:=arachne_display_base_link_bridge"
    "[j]oint_state_publisher .*arachne_display\\.urdf"
    "[a]rachne_gripper/lib/arachne_gripper/joint_state_mux"
    "[a]rachne_sim/lib/arachne_sim/base_sim_controller"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" >/dev/null 2>&1 || true
  done
}

arachne_source_ros_setup
arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first."

if [[ -r /proc/sys/kernel/osrelease ]] && grep -qi "microsoft\\|wsl" /proc/sys/kernel/osrelease; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
fi

cleanup_stale_demo

exec ros2 launch arachne_sim moveit_grasp_planning_demo.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  planner_id:="${PLANNER_ID}" \
  playback_speed:="${PLAYBACK_SPEED}" \
  loop:="${LOOP}" \
  launch_demo_rviz:="${WITH_RVIZ}" \
  allow_interpolation_fallback:="${ALLOW_INTERPOLATION_FALLBACK}"
