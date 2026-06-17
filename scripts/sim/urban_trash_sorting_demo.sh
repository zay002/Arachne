#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
PLANNER_ID="${PLANNER_ID:-RRTConnectkConfigDefault}"
PLAYBACK_SPEED="${PLAYBACK_SPEED:-0.85}"
LOOP="${LOOP:-true}"
WITH_RVIZ="${WITH_RVIZ:-true}"
PATROL_PATTERN="${PATROL_PATTERN:-box_entry}"
PATROL_DISTANCE_M="${PATROL_DISTANCE_M:-1.2}"
PATROL_BOX_WIDTH_M="${PATROL_BOX_WIDTH_M:-1.0}"
PATROL_BOX_HEIGHT_M="${PATROL_BOX_HEIGHT_M:-1.2}"
PATROL_ENTRY_M="${PATROL_ENTRY_M:-0.3}"
SHOW_KEEPOUT_MARKERS="${SHOW_KEEPOUT_MARKERS:-false}"
SLAM_MAP_YAML="${SLAM_MAP_YAML:-${ROOT_DIR}/src/arachne_nav/maps/road_lab_apriltag.yaml}"
TRASH_SEED="${TRASH_SEED:-26}"
TRASH_COUNT="${TRASH_COUNT:-10}"
SCAN_ARC_RADIUS_M="${SCAN_ARC_RADIUS_M:-0.32}"
SCAN_ARC_ANGLE_DEG="${SCAN_ARC_ANGLE_DEG:-72.0}"
SCAN_ARC_SAMPLES="${SCAN_ARC_SAMPLES:-5}"
SCAN_CYCLE_DURATION_SEC="${SCAN_CYCLE_DURATION_SEC:-4.2}"
DETECTION_LOCK_FRAMES="${DETECTION_LOCK_FRAMES:-1}"

cleanup_stale_demo() {
  local patterns=(
    "[r]os2 launch arachne_sim urban_trash_sorting_demo"
    "[r]viz2.*urban_trash_sorting_demo\\.rviz"
    "[a]rachne_urban_trash_sorting_demo_rviz"
    "[a]rachne_sim/lib/arachne_sim/urban_trash_sorting_demo"
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

exec ros2 launch arachne_sim urban_trash_sorting_demo.launch.py \
  gripper_type:="${GRIPPER_TYPE}" \
  planner_id:="${PLANNER_ID}" \
  playback_speed:="${PLAYBACK_SPEED}" \
  loop:="${LOOP}" \
  patrol_pattern:="${PATROL_PATTERN}" \
  patrol_distance_m:="${PATROL_DISTANCE_M}" \
  patrol_box_width_m:="${PATROL_BOX_WIDTH_M}" \
  patrol_box_height_m:="${PATROL_BOX_HEIGHT_M}" \
  patrol_entry_m:="${PATROL_ENTRY_M}" \
  show_keepout_markers:="${SHOW_KEEPOUT_MARKERS}" \
  slam_map_yaml:="${SLAM_MAP_YAML}" \
  trash_seed:="${TRASH_SEED}" \
  trash_count:="${TRASH_COUNT}" \
  scan_arc_radius_m:="${SCAN_ARC_RADIUS_M}" \
  scan_arc_angle_deg:="${SCAN_ARC_ANGLE_DEG}" \
  scan_arc_samples:="${SCAN_ARC_SAMPLES}" \
  scan_cycle_duration_sec:="${SCAN_CYCLE_DURATION_SEC}" \
  detection_lock_frames:="${DETECTION_LOCK_FRAMES}" \
  launch_demo_rviz:="${WITH_RVIZ}"
