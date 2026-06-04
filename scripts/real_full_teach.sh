#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_TYPE="${AUBO_TYPE:-aubo_i5}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
AUBO_PAYLOAD_MASS="${ARACHNE_AUBO_PAYLOAD_MASS:-0.818}"
AUBO_PAYLOAD_COG="${ARACHNE_AUBO_PAYLOAD_COG:-0.039927,0.045067,0.143233}"
AUBO_PAYLOAD_AOM="${ARACHNE_AUBO_PAYLOAD_AOM:-0,0,0}"
AUBO_PAYLOAD_INERTIA="${ARACHNE_AUBO_PAYLOAD_INERTIA:-0,0,0,0,0,0}"
TEACH_WITH_CAMERA="${ARACHNE_TEACH_WITH_CAMERA:-true}"
TEACH_CAMERA_COLOR_VIEW="${ARACHNE_TEACH_CAMERA_COLOR_VIEW:-true}"
TEACH_CAMERA_DEPTH_VIEW="${ARACHNE_TEACH_CAMERA_DEPTH_VIEW:-false}"
TEACH_CAMERA_POINTCLOUD="${ARACHNE_TEACH_CAMERA_POINTCLOUD:-false}"
AUBO_ARM_VELOCITY_COMMAND_TOPIC="${ARACHNE_AUBO_ARM_VELOCITY_COMMAND_TOPIC:-/arachne/aubo/joint_velocity_command}"
RUN_ENV_CHECK=true
KEEP_RUNNING=false
STOP_EXISTING=true
CONFIRM=false
PANEL_ARGS=()

usage() {
  cat <<EOF
Usage:
  ./scripts/real_full_teach.sh --yes [-- teach_panel_launch_args...]

Starts the complete real teach/replay stack:
  1. check real-hardware environment,
  2. start Aubo ROS2 driver in guarded prestart mode,
  3. remotely power on / startup Aubo and verify velocity hold control,
  4. start Scout + MS42DC bringup,
  5. open the teach/replay panel,
  6. stop background bringup processes when the panel exits.

Options:
  -y, --yes           Confirm real hardware control startup.
  --skip-env-check   Skip scripts/check_real_hardware_env.sh --strict.
  --keep-running     Leave bringup processes running after the panel exits.
  --no-stop-existing Do not stop stale Arachne real-stack processes first.
  -h, --help         Show this help.

Environment:
  AUBO_ROBOT_IP=${AUBO_ROBOT_IP}
  AUBO_TYPE=${AUBO_TYPE}
  WAIT_TIMEOUT_SEC=${WAIT_TIMEOUT_SEC}
  ARACHNE_TEACH_RECORDING_DIR=${RECORDING_DIR}
  ARACHNE_AUBO_PAYLOAD_MASS=${AUBO_PAYLOAD_MASS}
  ARACHNE_AUBO_PAYLOAD_COG=${AUBO_PAYLOAD_COG}
  ARACHNE_AUBO_PAYLOAD_AOM=${AUBO_PAYLOAD_AOM}
  ARACHNE_AUBO_PAYLOAD_INERTIA=${AUBO_PAYLOAD_INERTIA}
  ARACHNE_TEACH_WITH_CAMERA=${TEACH_WITH_CAMERA}
  ARACHNE_TEACH_CAMERA_COLOR_VIEW=${TEACH_CAMERA_COLOR_VIEW}
  ARACHNE_TEACH_CAMERA_DEPTH_VIEW=${TEACH_CAMERA_DEPTH_VIEW}
  ARACHNE_TEACH_CAMERA_POINTCLOUD=${TEACH_CAMERA_POINTCLOUD}
  ARACHNE_AUBO_ARM_VELOCITY_COMMAND_TOPIC=${AUBO_ARM_VELOCITY_COMMAND_TOPIC}

Examples:
  ./scripts/real_full_teach.sh --yes
  ./scripts/real_full_teach.sh --yes -- recording_dir:=recordings/teach
EOF
}

while (($#)); do
  case "$1" in
    -y|--yes)
      CONFIRM=true
      ;;
    --skip-env-check)
      RUN_ENV_CHECK=false
      ;;
    --keep-running)
      KEEP_RUNNING=true
      ;;
    --no-stop-existing)
      STOP_EXISTING=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      PANEL_ARGS+=("$@")
      break
      ;;
    *)
      PANEL_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${ARACHNE_CONFIRM_REAL_TEACH:-}" == "YES" ]]; then
  CONFIRM=true
fi

if [[ "${CONFIRM}" != "true" ]]; then
  cat >&2 <<'EOF'
Refusing to start the full real teach stack without confirmation.

This command powers on / starts the real Aubo arm and opens controls for the
Scout base, Aubo arm, and MS42DC gripper. Confirm the workspace is clear and an
emergency stop / power cut is within reach, then rerun with:

  ./scripts/real_full_teach.sh --yes
EOF
  exit 2
fi

set +u
export ARACHNE_ENV_QUIET=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
unset ARACHNE_ENV_QUIET
set -u

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run ./scripts/build_workspace.sh first." >&2
  exit 1
fi

LOG_DIR="${ROOT_DIR}/log/real_full_teach/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}" "${RECORDING_DIR}"

BACKGROUND_PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ "${KEEP_RUNNING}" == "true" ]]; then
    echo "Keeping bringup processes running (--keep-running). Logs: ${LOG_DIR}"
    exit "${status}"
  fi

  if ((${#BACKGROUND_PIDS[@]})); then
    echo "Stopping bringup processes..."
    local pid
    for pid in "${BACKGROUND_PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -INT "${pid}" 2>/dev/null || true
      fi
    done
    sleep 2
    for pid in "${BACKGROUND_PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    done
  fi

  echo "Logs: ${LOG_DIR}"
  exit "${status}"
}
trap cleanup EXIT INT TERM

tail_log() {
  local log_file="$1"
  if [[ -f "${log_file}" ]]; then
    echo "---- tail: ${log_file} ----" >&2
    tail -80 "${log_file}" >&2 || true
    echo "---- end tail ----" >&2
  fi
}

start_background() {
  local label="$1"
  local log_file="$2"
  shift 2

  echo "Starting ${label}; log: ${log_file}"
  "$@" >"${log_file}" 2>&1 &
  local pid=$!
  BACKGROUND_PIDS+=("${pid}")
  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "${label} exited during startup." >&2
    tail_log "${log_file}"
    exit 1
  fi
}

run_logged() {
  local label="$1"
  local log_file="$2"
  shift 2

  echo "Running ${label}; log: ${log_file}"
  set +e
  "$@" 2>&1 | tee "${log_file}"
  local status=${PIPESTATUS[0]}
  set -e
  if [[ "${status}" != "0" ]]; then
    echo "${label} failed with exit code ${status}." >&2
    tail_log "${log_file}"
    exit "${status}"
  fi
}

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  local info
  while (( SECONDS < deadline )); do
    info="$(timeout 3 ros2 topic info "${topic}" 2>/dev/null || true)"
    if grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"${info}"; then
      echo "  ready: ${label} (${topic})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} (${topic})." >&2
  exit 1
}

wait_for_action() {
  local action="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 action list 2>/dev/null | grep -qx "${action}"; then
      echo "  ready: ${label} (${action})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} (${action})." >&2
  exit 1
}

wait_for_controller_active() {
  local controller="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  local output
  local clean_output
  while (( SECONDS < deadline )); do
    output="$(timeout 3 ros2 control list_controllers 2>/dev/null || true)"
    clean_output="$("${ARACHNE_SYSTEM_PYTHON}" -c 'import re,sys; print(re.sub("\x1b\\[[0-9;]*m", "", sys.stdin.read()), end="")' <<<"${output}")"
    if awk '{print $1, $NF}' <<<"${clean_output}" | grep -qx "${controller} active"; then
      echo "  ready: ${label} (${controller})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} (${controller})." >&2
  timeout 3 ros2 control list_controllers 2>/dev/null || true
  exit 1
}

has_recording_dir=false
for arg in "${PANEL_ARGS[@]}"; do
  if [[ "${arg}" == recording_dir:=* || "${arg}" == --recording_dir:=* ]]; then
    has_recording_dir=true
    break
  fi
done

echo "Arachne full real teach startup"
echo "  workspace: ${ROOT_DIR}"
echo "  Aubo IP/model: ${AUBO_ROBOT_IP} / ${AUBO_TYPE}"
echo "  Aubo payload: mass=${AUBO_PAYLOAD_MASS}kg cog=${AUBO_PAYLOAD_COG}"
echo "  camera: enabled=${TEACH_WITH_CAMERA} color_view=${TEACH_CAMERA_COLOR_VIEW} depth_view=${TEACH_CAMERA_DEPTH_VIEW} pointcloud=${TEACH_CAMERA_POINTCLOUD}"
echo "  recording dir: ${RECORDING_DIR}"
echo "  logs: ${LOG_DIR}"

cd "${ROOT_DIR}"

if [[ "${STOP_EXISTING}" == "true" ]]; then
  run_logged "stop existing real stack" \
    "${LOG_DIR}/00_stop_existing_real_stack.log" \
    "${ROOT_DIR}/scripts/stop_real_stack.sh"
fi

if [[ "${RUN_ENV_CHECK}" == "true" ]]; then
  run_logged "real-hardware environment check" \
    "${LOG_DIR}/01_check_real_hardware_env.log" \
    "${ROOT_DIR}/scripts/check_real_hardware_env.sh" --strict
fi

start_background "Aubo driver" \
  "${LOG_DIR}/02_aubo_driver.log" \
  env ARACHNE_CONFIRM_AUBO_DRIVER=YES \
      ARACHNE_AUBO_ALLOW_PRESTART=YES \
      AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
      AUBO_TYPE="${AUBO_TYPE}" \
      "${ROOT_DIR}/scripts/real_aubo_bringup.sh"

run_logged "Aubo payload configure" \
  "${LOG_DIR}/03_aubo_payload.log" \
  env ARACHNE_CONFIRM_AUBO_PAYLOAD=YES \
      AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
      "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_payload.py" \
      --mass "${AUBO_PAYLOAD_MASS}" \
      --cog "${AUBO_PAYLOAD_COG}" \
      --aom "${AUBO_PAYLOAD_AOM}" \
      --inertia "${AUBO_PAYLOAD_INERTIA}"

run_logged "Aubo guarded remote startup" \
  "${LOG_DIR}/04_aubo_remote_start.log" \
  env ARACHNE_CONFIRM_AUBO_REMOTE_START=YES \
      AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
      "${ROOT_DIR}/scripts/real_aubo_remote_start.sh"

run_logged "Aubo running/safety check" \
  "${LOG_DIR}/05_aubo_prepare.log" \
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"

wait_for_topic "/joint_states" "Aubo joint states"
wait_for_controller_active "forward_command_controller_velocity" "Aubo velocity controller"

start_background "Scout + MS42DC bringup" \
  "${LOG_DIR}/06_base_gripper_bringup.log" \
  "${ROOT_DIR}/scripts/real_bringup.sh" --no-aubo

wait_for_topic "/odom" "Scout odometry"
wait_for_topic "/arachne/hardware/gripper_status" "MS42DC status"

echo "Opening teach/replay panel..."
if [[ "${has_recording_dir}" == "true" ]]; then
  run_logged "teach/replay panel" \
    "${LOG_DIR}/07_teach_panel.log" \
    ros2 launch arachne_operator teach_panel.launch.py \
      with_camera:="${TEACH_WITH_CAMERA}" \
      camera_with_color_view:="${TEACH_CAMERA_COLOR_VIEW}" \
      camera_with_depth_view:="${TEACH_CAMERA_DEPTH_VIEW}" \
      camera_publish_pointcloud:="${TEACH_CAMERA_POINTCLOUD}" \
      arm_velocity_command_topic:="${AUBO_ARM_VELOCITY_COMMAND_TOPIC}" \
      "${PANEL_ARGS[@]}"
else
  run_logged "teach/replay panel" \
    "${LOG_DIR}/07_teach_panel.log" \
    ros2 launch arachne_operator teach_panel.launch.py \
      with_camera:="${TEACH_WITH_CAMERA}" \
      camera_with_color_view:="${TEACH_CAMERA_COLOR_VIEW}" \
      camera_with_depth_view:="${TEACH_CAMERA_DEPTH_VIEW}" \
      camera_publish_pointcloud:="${TEACH_CAMERA_POINTCLOUD}" \
      arm_velocity_command_topic:="${AUBO_ARM_VELOCITY_COMMAND_TOPIC}" \
      recording_dir:="${RECORDING_DIR}" "${PANEL_ARGS[@]}"
fi
