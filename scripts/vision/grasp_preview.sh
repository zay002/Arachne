#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

refresh_arachne_environment() {
  export ARACHNE_ENV_NO_WORKSPACE=0
  set +u
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/scripts/env/arachne_env.sh"
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
  fi
  if [[ -f "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh"
  fi
  set -u
  hash -r
}

refresh_arachne_environment

export YOLO_AUTOINSTALL="${YOLO_AUTOINSTALL:-false}"

DEFAULT_YOLO_ENGINE="${ROOT_DIR}/yolo_workspace/engines/trash_yolo26n_seg_best_fp16_640.engine"
DEFAULT_YOLO_ONNX="${ROOT_DIR}/yolo_workspace/weights/trash_yolo26n_seg_best.onnx"
DEFAULT_YOLO_PT="${ROOT_DIR}/yolo_workspace/weights/trash_yolo26n_seg_best.pt"
if [[ -n "${ARACHNE_GRASP_YOLO_MODEL:-}" ]]; then
  MODEL="${ARACHNE_GRASP_YOLO_MODEL}"
elif [[ -f "${DEFAULT_YOLO_ENGINE}" ]]; then
  MODEL="${DEFAULT_YOLO_ENGINE}"
elif [[ -f "${DEFAULT_YOLO_ONNX}" ]]; then
  MODEL="${DEFAULT_YOLO_ONNX}"
else
  MODEL="${DEFAULT_YOLO_PT}"
fi
VENV="${ARACHNE_GRASP_YOLO_VENV:-${ROOT_DIR}/yolo_workspace/.venv}"
YOLO_TASK="${ARACHNE_GRASP_YOLO_TASK:-segment}"
CLASSES="${ARACHNE_GRASP_CLASSES:-trash}"
CONF="${ARACHNE_GRASP_CONF:-0.25}"
IMGSZ="${ARACHNE_GRASP_IMGSZ:-640}"
DEVICE_ID="${ARACHNE_GRASP_DEVICE_ID:-0}"
ONNX_DEVICE="${ARACHNE_GRASP_ONNX_DEVICE:-cpu}"
DISPLAY_FRAME_PREFIX="${ARACHNE_GRASP_DISPLAY_FRAME_PREFIX:-grasp_preview_}"
GRIPPER_TYPE="${ARACHNE_GRASP_GRIPPER_TYPE:-ms42dc}"
TOOL_ADAPTER_XYZ="${ARACHNE_GRASP_TOOL_ADAPTER_XYZ:-0.0 0.0 0.0}"
TOOL_ADAPTER_RPY="${ARACHNE_GRASP_TOOL_ADAPTER_RPY:-0.0 0.0 0.785398163397}"
DEPTH_PROJECTION_FLIP_X="${ARACHNE_GRASP_DEPTH_PROJECTION_FLIP_X:-true}"
DEPTH_PROJECTION_FLIP_Y="${ARACHNE_GRASP_DEPTH_PROJECTION_FLIP_Y:-true}"
CAMERA_PARENT_FRAME="${ARACHNE_GRASP_CAMERA_PARENT_FRAME:-${DISPLAY_FRAME_PREFIX}ee_camera_link}"
CAMERA_COLOR_WIDTH="${ARACHNE_GRASP_CAMERA_COLOR_WIDTH:-640}"
CAMERA_COLOR_HEIGHT="${ARACHNE_GRASP_CAMERA_COLOR_HEIGHT:-480}"
CAMERA_COLOR_FPS="${ARACHNE_GRASP_CAMERA_COLOR_FPS:-30.0}"
CAMERA_DEPTH_WIDTH="${ARACHNE_GRASP_CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${ARACHNE_GRASP_CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${ARACHNE_GRASP_CAMERA_DEPTH_FPS:-5.0}"
GRASP_BASE_OFFSET="${ARACHNE_GRASP_BASE_OFFSET:-0,0,0}"
EXECUTE_REAL="${ARACHNE_GRASP_EXECUTE_REAL:-false}"
EXECUTE_REAL_CONFIRM="${ARACHNE_CONFIRM_GRASP_EXECUTE_REAL:-}"
REAL_JOINT_STATES_TOPIC="${ARACHNE_GRASP_REAL_JOINT_STATES_TOPIC:-/joint_states}"
REAL_EXECUTE_BACKEND="${ARACHNE_GRASP_REAL_EXECUTE_BACKEND:-sdk_move_joint}"
REAL_SDK_IP="${ARACHNE_GRASP_REAL_SDK_IP:-${AUBO_ROBOT_IP:-192.168.127.128}}"
REAL_SDK_MOVE_SPEED="${ARACHNE_GRASP_REAL_SDK_MOVE_SPEED:-0.25}"
REAL_SDK_MOVE_ACCEL="${ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL:-0.45}"
REAL_SDK_TEACH_FLAG_PATH="${ARACHNE_GRASP_REAL_SDK_TEACH_FLAG_PATH:-/tmp/arachne_aubo_teach_mode}"
REAL_SDK_CONTROL_OWNER_PATH="${ARACHNE_GRASP_AUBO_CONTROL_OWNER_PATH:-/tmp/arachne_aubo_control_owner}"
REAL_SDK_CONTROL_OWNER_NAME="${ARACHNE_GRASP_AUBO_CONTROL_OWNER_NAME:-grasp_task_server}"
REAL_RETURN_HOME="${ARACHNE_GRASP_REAL_RETURN_HOME:-true}"
REAL_HOME_JOINTS="${ARACHNE_GRASP_REAL_HOME_JOINTS:-${ARACHNE_AUBO_HOME_JOINTS_RAD:--1.5707963267949,0.201570428261868,1.65970467002488,0.485178041391533,1.67675136677345,0.76432946885334}}"
REAL_HOME_DURATION="${ARACHNE_GRASP_REAL_HOME_DURATION:-2.5}"
REMOTE_PLANNER_URL="${ARACHNE_REMOTE_PLANNER_URL:-}"
REMOTE_PLANNER_TIMEOUT="${ARACHNE_REMOTE_PLANNER_TIMEOUT:-2.0}"
START_MODEL="${ARACHNE_GRASP_START_MODEL:-true}"
START_MOVEIT="${ARACHNE_GRASP_START_MOVEIT:-true}"
START_CAMERA="${ARACHNE_GRASP_START_CAMERA:-true}"
WITH_RVIZ="${ARACHNE_GRASP_WITH_RVIZ:-true}"
ARM_JOINTS_OVERRIDE="${ARACHNE_GRASP_ARM_JOINTS:-}"
if [[ " $* " == *" --planner-backend remote "* || " $* " == *" --planner-backend local "* || " $* " == *" --planner-backend none "* ]]; then
  START_MOVEIT=false
fi
LOG_DIR="${ROOT_DIR}/log/grasp_preview/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

{
  echo "timestamp: $(date --iso-8601=seconds)"
  echo "root_dir: ${ROOT_DIR}"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "workspace_setup: ${ROOT_DIR}/install/setup.bash"
  if command -v sha256sum >/dev/null 2>&1; then
    echo "pipeline_sha256: $(sha256sum "${ROOT_DIR}/scripts/vision/grasp_preview_pipeline.py" | awk '{print $1}')"
    echo "script_sha256: $(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
  fi
  echo "python: ${ARACHNE_SYSTEM_PYTHON}"
  echo "yolo_model: ${MODEL}"
  echo "yolo_task: ${YOLO_TASK}"
  echo "yolo_autoinstall: ${YOLO_AUTOINSTALL}"
  echo "device_id: ${DEVICE_ID}"
  echo "onnx_device: ${ONNX_DEVICE}"
  echo "ament_prefix_path: ${AMENT_PREFIX_PATH:-}"
  echo "depth_projection_flip_x: ${DEPTH_PROJECTION_FLIP_X}"
  echo "depth_projection_flip_y: ${DEPTH_PROJECTION_FLIP_Y}"
  echo "camera_parent_frame: ${CAMERA_PARENT_FRAME}"
  echo "camera_color_size: ${CAMERA_COLOR_WIDTH}x${CAMERA_COLOR_HEIGHT}@${CAMERA_COLOR_FPS}"
  echo "camera_depth_size: ${CAMERA_DEPTH_WIDTH}x${CAMERA_DEPTH_HEIGHT}@${CAMERA_DEPTH_FPS}"
  echo "grasp_base_offset: ${GRASP_BASE_OFFSET}"
  echo "execute_real: ${EXECUTE_REAL}"
  echo "execute_real_confirmed: $([[ "${EXECUTE_REAL_CONFIRM}" == "YES" ]] && echo true || echo false)"
  echo "real_joint_states_topic: ${REAL_JOINT_STATES_TOPIC}"
  echo "real_execute_backend: ${REAL_EXECUTE_BACKEND}"
  echo "real_sdk_ip: ${REAL_SDK_IP}"
  echo "real_sdk_move_speed: ${REAL_SDK_MOVE_SPEED}"
  echo "real_sdk_move_accel: ${REAL_SDK_MOVE_ACCEL}"
  echo "real_sdk_teach_flag_path: ${REAL_SDK_TEACH_FLAG_PATH}"
  echo "real_sdk_control_owner_path: ${REAL_SDK_CONTROL_OWNER_PATH}"
  echo "real_sdk_control_owner_name: ${REAL_SDK_CONTROL_OWNER_NAME}"
  echo "real_return_home: ${REAL_RETURN_HOME}"
  echo "real_home_joints: ${REAL_HOME_JOINTS}"
  echo "real_home_duration: ${REAL_HOME_DURATION}"
  echo "remote_planner_url: ${REMOTE_PLANNER_URL:-disabled}"
  echo "remote_planner_timeout: ${REMOTE_PLANNER_TIMEOUT}"
} >"${LOG_DIR}/00_environment.txt"

if [[ "${EXECUTE_REAL}" == "true" && "${EXECUTE_REAL_CONFIRM}" != "YES" ]]; then
  cat >&2 <<'EOF'
Refusing real grasp execution without confirmation.

Set both variables after confirming the workspace is clear and E-stop is within reach:

  ARACHNE_GRASP_EXECUTE_REAL=true
  ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES
EOF
  exit 2
fi

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
  sleep 0.3
  cleanup_stale_preview_nodes
}
trap cleanup EXIT INT TERM

cleanup_stale_preview_nodes() {
  pkill -f "[r]os2 launch arachne_description display.launch.py .*display_frame_prefix:=grasp_preview_" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=arachne_display_robot_state_publisher" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=arachne_display_base_link_bridge" >/dev/null 2>&1 || true
  pkill -f "[j]oint_state_publisher .*arachne_display\\.urdf" >/dev/null 2>&1 || true
  pkill -f "[a]rachne_gripper/lib/arachne_gripper/joint_state_mux" >/dev/null 2>&1 || true
  if [[ "${START_CAMERA}" == "true" ]]; then
    pkill -f "[r]os2 launch arachne_sensors gemini335.launch.py" >/dev/null 2>&1 || true
    pkill -f "[g]emini335_v4l2_node" >/dev/null 2>&1 || true
    pkill -f "[_]_node:=gemini335_color_tf" >/dev/null 2>&1 || true
    pkill -f "[_]_node:=gemini335_depth_tf" >/dev/null 2>&1 || true
  fi
  pkill -f "[r]os2 launch arachne_moveit_config moveit_planning.launch.py" >/dev/null 2>&1 || true
  pkill -f "[m]oveit_ros_move_group/move_group.*joint_states:=/arachne/display/joint_states" >/dev/null 2>&1 || true
  pkill -f "[g]rasp_preview_pipeline.py" >/dev/null 2>&1 || true
  pkill -f "[r]viz2 -d .*grasp_preview\\.rviz" >/dev/null 2>&1 || true
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
    "tool_adapter_xyz:=${TOOL_ADAPTER_XYZ}" \
    "tool_adapter_rpy:=${TOOL_ADAPTER_RPY}" \
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
    "tool_adapter_xyz:=${TOOL_ADAPTER_XYZ}" \
    "tool_adapter_rpy:=${TOOL_ADAPTER_RPY}" \
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
    "color_width:=${CAMERA_COLOR_WIDTH}" \
    "color_height:=${CAMERA_COLOR_HEIGHT}" \
    "color_fps:=${CAMERA_COLOR_FPS}" \
    "depth_width:=${CAMERA_DEPTH_WIDTH}" \
    "depth_height:=${CAMERA_DEPTH_HEIGHT}" \
    "depth_fps:=${CAMERA_DEPTH_FPS}" \
    "projection_flip_x:=${DEPTH_PROJECTION_FLIP_X}" \
    "projection_flip_y:=${DEPTH_PROJECTION_FLIP_Y}" \
    "camera_parent_frame:=${CAMERA_PARENT_FRAME}" \
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
planner_backend_label="MoveIt 2 + OMPL via /plan_kinematic_path"
if [[ " $* " == *" --planner-backend remote "* ]]; then
  planner_backend_label="remote HTTP planner via ${REMOTE_PLANNER_URL:-http://127.0.0.1:8765}"
elif [[ " $* " == *" --planner-backend local "* ]]; then
  planner_backend_label="local IK constrained trajectory"
elif [[ " $* " == *" --planner-backend none "* ]]; then
  planner_backend_label="perception only"
fi
echo "Planner backend: ${planner_backend_label}"
echo "RViz MarkerArray shows named task waypoints and a magenta playback cursor."
if [[ "${EXECUTE_REAL}" == "true" ]]; then
  echo "REAL execution is armed: backend=${REAL_EXECUTE_BACKEND}; default sends key joint targets through Aubo SDK moveJoint."
else
  echo "REAL execution is disabled; this run only previews in RViz."
fi
echo "Restart search after PLAN_LOCKED:"
echo "  ros2 topic pub --once /arachne/grasp_preview/restart_search std_msgs/msg/Empty '{}'"
echo "Pipeline log:"
echo "  ${LOG_DIR}/05_pipeline.log"

run_pipeline() {
  local device_id="$1"
  shift
  local projection_args=()
  if [[ "${DEPTH_PROJECTION_FLIP_X}" == "true" ]]; then
    projection_args+=(--depth-projection-flip-x)
  else
    projection_args+=(--no-depth-projection-flip-x)
  fi
  if [[ "${DEPTH_PROJECTION_FLIP_Y}" == "true" ]]; then
    projection_args+=(--depth-projection-flip-y)
  else
    projection_args+=(--no-depth-projection-flip-y)
  fi
  local execute_args=()
  if [[ "${EXECUTE_REAL}" == "true" ]]; then
    local return_home_arg="--real-return-home"
    if [[ "${REAL_RETURN_HOME}" != "true" ]]; then
      return_home_arg="--no-real-return-home"
    fi
    execute_args+=(
      --execute-real
      --execute-real-confirm "${EXECUTE_REAL_CONFIRM}"
      --real-execute-backend "${REAL_EXECUTE_BACKEND}"
      --real-joint-states-topic "${REAL_JOINT_STATES_TOPIC}"
      --real-sdk-ip "${REAL_SDK_IP}"
      --real-sdk-teach-flag-path "${REAL_SDK_TEACH_FLAG_PATH}"
      --real-sdk-control-owner-path "${REAL_SDK_CONTROL_OWNER_PATH}"
      --real-sdk-control-owner-name "${REAL_SDK_CONTROL_OWNER_NAME}"
      --real-sdk-move-speed "${REAL_SDK_MOVE_SPEED}"
      --real-sdk-move-accel "${REAL_SDK_MOVE_ACCEL}"
      "${return_home_arg}"
      "--real-home-joints=${REAL_HOME_JOINTS}"
      --real-home-duration "${REAL_HOME_DURATION}"
    )
  fi
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/vision/grasp_preview_pipeline.py" \
    --model "${MODEL}" \
    --venv "${VENV}" \
    --yolo-task "${YOLO_TASK}" \
    --classes "${CLASSES}" \
    --conf "${CONF}" \
    --imgsz "${IMGSZ}" \
    --device-id "${device_id}" \
    --onnx-device "${ONNX_DEVICE}" \
    --gripper-type "${GRIPPER_TYPE}" \
    --aubo-base-frame "${DISPLAY_FRAME_PREFIX}aubo_base_link" \
    --grasp-base-offset "${GRASP_BASE_OFFSET}" \
    --remote-planner-url "${REMOTE_PLANNER_URL:-http://127.0.0.1:8765}" \
    --remote-planner-timeout "${REMOTE_PLANNER_TIMEOUT}" \
    "${projection_args[@]}" \
    "${execute_args[@]}" \
    "$@"
}

set +e
run_pipeline "${DEVICE_ID}" "$@" 2>&1 | tee "${LOG_DIR}/05_pipeline.log"
PIPELINE_STATUS=${PIPESTATUS[0]}
set -e
if [[ "${PIPELINE_STATUS}" -ne 0 ]]; then
  echo "grasp_preview_pipeline exited with status ${PIPELINE_STATUS}. See ${LOG_DIR}/05_pipeline.log" >&2
fi
exit "${PIPELINE_STATUS}"
