#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_TYPE="${AUBO_TYPE:-aubo_i5}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
AUBO_PAYLOAD_MASS="${ARACHNE_AUBO_PAYLOAD_MASS:-3.5}"
AUBO_PAYLOAD_COG="${ARACHNE_AUBO_PAYLOAD_COG:-0,0,0.18}"
AUBO_PAYLOAD_AOM="${ARACHNE_AUBO_PAYLOAD_AOM:-0,0,0}"
AUBO_PAYLOAD_INERTIA="${ARACHNE_AUBO_PAYLOAD_INERTIA:-0,0,0,0,0,0}"
RUN_ENV_CHECK=true
KEEP_RUNNING=false
STOP_EXISTING=true
CONFIRM=false
ACCEPTANCE_ARGS=()

usage() {
  cat <<EOF
Usage:
  ./scripts/real_full_acceptance.sh --yes [acceptance launch args...]

Runs the complete real-hardware acceptance flow:
  1. check real-hardware environment,
  2. start Aubo ROS2 driver in guarded prestart mode,
  3. remotely power on / startup Aubo and verify hold control,
  4. start Scout + MS42DC bringup,
  5. run the real joint acceptance test,
  6. stop background bringup processes on exit.

Options:
  -y, --yes           Confirm real hardware motion.
  --skip-env-check   Skip scripts/check_real_hardware_env.sh --strict.
  --keep-running     Leave bringup processes running after the test.
  --no-stop-existing Do not stop stale Arachne real-stack processes first.
  -h, --help         Show this help.

Environment:
  AUBO_ROBOT_IP=${AUBO_ROBOT_IP}
  AUBO_TYPE=${AUBO_TYPE}
  WAIT_TIMEOUT_SEC=${WAIT_TIMEOUT_SEC}
  ARACHNE_AUBO_PAYLOAD_MASS=${AUBO_PAYLOAD_MASS}
  ARACHNE_AUBO_PAYLOAD_COG=${AUBO_PAYLOAD_COG}
  ARACHNE_AUBO_PAYLOAD_AOM=${AUBO_PAYLOAD_AOM}
  ARACHNE_AUBO_PAYLOAD_INERTIA=${AUBO_PAYLOAD_INERTIA}

Example:
  ./scripts/real_full_acceptance.sh --yes
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
      ACCEPTANCE_ARGS+=("$@")
      break
      ;;
    *)
      ACCEPTANCE_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${ARACHNE_CONFIRM_REAL_MOTION:-}" == "YES" ]]; then
  CONFIRM=true
fi

if [[ "${CONFIRM}" != "true" ]]; then
  cat >&2 <<'EOF'
Refusing to run the complete real-hardware acceptance test without confirmation.

This command will move the Scout base, Aubo arm, and MS42DC gripper.
Confirm the workspace is clear and an emergency stop / power cut is within reach,
then rerun with:

  ./scripts/real_full_acceptance.sh --yes
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

LOG_DIR="${ROOT_DIR}/log/real_full_acceptance/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

BACKGROUND_PIDS=()
BACKGROUND_LABELS=()

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
  BACKGROUND_LABELS+=("${label}")
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
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
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

echo "Arachne full real-hardware acceptance"
echo "  workspace: ${ROOT_DIR}"
echo "  Aubo IP/model: ${AUBO_ROBOT_IP} / ${AUBO_TYPE}"
echo "  Aubo payload: mass=${AUBO_PAYLOAD_MASS}kg cog=${AUBO_PAYLOAD_COG}"
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
wait_for_action "/joint_trajectory_controller/follow_joint_trajectory" "Aubo trajectory action"

start_background "Scout + MS42DC bringup" \
  "${LOG_DIR}/06_base_gripper_bringup.log" \
  "${ROOT_DIR}/scripts/real_bringup.sh" --no-aubo

wait_for_topic "/odom" "Scout odometry"
wait_for_topic "/arachne/hardware/gripper_status" "MS42DC status"

run_logged "joint acceptance test" \
  "${LOG_DIR}/07_real_hardware_acceptance_test.log" \
  env ARACHNE_CONFIRM_REAL_MOTION=YES \
      "${ROOT_DIR}/scripts/real_hardware_acceptance_test.sh" \
      "${ACCEPTANCE_ARGS[@]}"

echo "Full real-hardware acceptance complete."
