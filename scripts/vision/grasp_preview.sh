#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

MODEL="${ARACHNE_GRASP_YOLO_MODEL:-${ROOT_DIR}/yolo_workspace/weights/yolo26n.pt}"
VENV="${ARACHNE_GRASP_YOLO_VENV:-${ROOT_DIR}/yolo_workspace/.venv}"
CLASSES="${ARACHNE_GRASP_CLASSES:-bottle}"
CONF="${ARACHNE_GRASP_CONF:-0.25}"
IMGSZ="${ARACHNE_GRASP_IMGSZ:-640}"
DEVICE_ID="${ARACHNE_GRASP_DEVICE_ID:-0}"
DISPLAY_FRAME_PREFIX="${ARACHNE_GRASP_DISPLAY_FRAME_PREFIX:-grasp_preview_}"
GRIPPER_TYPE="${ARACHNE_GRASP_GRIPPER_TYPE:-ms42dc}"
START_MODEL="${ARACHNE_GRASP_START_MODEL:-true}"
START_MOVEIT="${ARACHNE_GRASP_START_MOVEIT:-true}"
START_CAMERA="${ARACHNE_GRASP_START_CAMERA:-true}"
WITH_RVIZ="${ARACHNE_GRASP_WITH_RVIZ:-true}"
ARM_JOINTS_OVERRIDE="${ARACHNE_GRASP_ARM_JOINTS:-}"
LOG_DIR="${ROOT_DIR}/log/grasp_preview/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

if [[ ! -x "${VENV}/bin/python" ]]; then
  "${ROOT_DIR}/scripts/vision/setup_yolo_env.sh"
fi
if [[ ! -f "${MODEL}" ]]; then
  "${ROOT_DIR}/scripts/vision/download_yolo_weights.sh"
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

cleanup_stale_preview_nodes() {
  pkill -f "[_]_node:=arachne_display_robot_state_publisher" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=arachne_display_base_link_bridge" >/dev/null 2>&1 || true
  pkill -f "[r]os2 launch arachne_sensors gemini335.launch.py" >/dev/null 2>&1 || true
  pkill -f "[g]emini335_v4l2_node" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=gemini335_color_tf" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=gemini335_depth_tf" >/dev/null 2>&1 || true
  pkill -f "[r]os2 launch arachne_moveit_config moveit_planning.launch.py" >/dev/null 2>&1 || true
  pkill -f "[m]oveit_ros_move_group/move_group.*joint_states:=/arachne/display/joint_states" >/dev/null 2>&1 || true
}

fail_if_camera_died() {
  local log_file="$1"
  if grep -Eq "cannot open color device|depth device does not exist|RuntimeError:|process has died.*gemini335_v4l2_node" "${log_file}" 2>/dev/null; then
    echo "Gemini335 failed to start. See ${log_file}" >&2
    tail -80 "${log_file}" >&2 || true
    exit 1
  fi
}

cd "${ROOT_DIR}"

cleanup_stale_preview_nodes

ARM_ARGS=()
ARM_POSE_SOURCE="display.launch.py defaults"
if [[ -n "${ARM_JOINTS_OVERRIDE}" ]]; then
  ARM_JOINTS_TEXT="${ARM_JOINTS_OVERRIDE//,/ }"
  read -r -a ARM_VALUES <<<"${ARM_JOINTS_TEXT}"
  if [[ "${#ARM_VALUES[@]}" -ne 6 ]]; then
    echo "ARACHNE_GRASP_ARM_JOINTS must contain 6 comma- or space-separated joint values." >&2
    exit 2
  fi
  ARM_ARGS=(
    "aubo_shoulder_joint:=${ARM_VALUES[0]}"
    "aubo_upperArm_joint:=${ARM_VALUES[1]}"
    "aubo_foreArm_joint:=${ARM_VALUES[2]}"
    "aubo_wrist1_joint:=${ARM_VALUES[3]}"
    "aubo_wrist2_joint:=${ARM_VALUES[4]}"
    "aubo_wrist3_joint:=${ARM_VALUES[5]}"
  )
  ARM_POSE_SOURCE="ARACHNE_GRASP_ARM_JOINTS"
  {
    echo "source_kind: env_override"
    echo "source: ARACHNE_GRASP_ARM_JOINTS"
    echo "joints_csv: ${ARM_VALUES[*]}"
    echo "launch_args: ${ARM_ARGS[*]}"
  } >"${LOG_DIR}/00_arm_pose.txt"
else
  {
    echo "source_kind: display_default"
    echo "source: display.launch.py"
    echo "note: default arm pose is seeded from the last known teach pose for offline visualization"
  } >"${LOG_DIR}/00_arm_pose.txt"
fi

if [[ "${START_MODEL}" == "true" ]]; then
  echo "Starting model with arm pose source: ${ARM_POSE_SOURCE}"
  ros2 launch arachne_description display.launch.py \
    with_rviz:=false \
    with_base_gui:=false \
    with_gripper_gui:=false \
    with_gripper_sim:=false \
    "gripper_type:=${GRIPPER_TYPE}" \
    use_gui:=false \
    "display_frame_prefix:=${DISPLAY_FRAME_PREFIX}" \
    "${ARM_ARGS[@]}" \
    >"${LOG_DIR}/01_model.log" 2>&1 &
  PIDS+=("$!")
  sleep 1.0
fi

if [[ "${START_MOVEIT}" == "true" ]]; then
  ros2 launch arachne_moveit_config moveit_planning.launch.py \
    launch_rviz:=false \
    with_robot_state_publisher:=false \
    "gripper_type:=${GRIPPER_TYPE}" \
    joint_states_topic:=/arachne/display/joint_states \
    >"${LOG_DIR}/02_moveit.log" 2>&1 &
  PIDS+=("$!")
  sleep 1.0
fi

if [[ "${START_CAMERA}" == "true" ]]; then
  CAMERA_LOG="${LOG_DIR}/03_gemini335.log"
  ros2 launch arachne_sensors gemini335.launch.py \
    publish_pointcloud:=false \
    with_color_view:=false \
    with_depth_view:=false \
    with_tf:=true \
    "camera_parent_frame:=${DISPLAY_FRAME_PREFIX}ee_camera_link" \
    >"${CAMERA_LOG}" 2>&1 &
  PIDS+=("$!")
  sleep 2.0
  fail_if_camera_died "${CAMERA_LOG}"
fi

if [[ "${WITH_RVIZ}" == "true" ]]; then
  rviz2 -d "${ROOT_DIR}/src/arachne_description/rviz/grasp_preview.rviz" \
    >"${LOG_DIR}/04_rviz.log" 2>&1 &
  PIDS+=("$!")
fi

echo "Grasp preview logs: ${LOG_DIR}"
echo "RViz topics:"
echo "  /arachne/grasp_preview/markers"
echo "  /arachne/grasp_preview/roi_cloud"
echo "  /arachne/grasp_preview/path"
echo "  /arachne/grasp_preview/annotated_image"
echo "Planner backend: MoveIt 2 + OMPL via /plan_kinematic_path"
echo "RViz MarkerArray shows named task waypoints and a magenta playback cursor."
echo "Restart search after PLAN_LOCKED:"
echo "  ros2 topic pub --once /arachne/grasp_preview/restart_search std_msgs/msg/Empty '{}'"

exec "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/vision/grasp_preview_pipeline.py" \
  --model "${MODEL}" \
  --venv "${VENV}" \
  --classes "${CLASSES}" \
  --conf "${CONF}" \
  --imgsz "${IMGSZ}" \
  --device-id "${DEVICE_ID}" \
  --gripper-type "${GRIPPER_TYPE}" \
  --aubo-base-frame "${DISPLAY_FRAME_PREFIX}aubo_base_link" \
  "$@"
