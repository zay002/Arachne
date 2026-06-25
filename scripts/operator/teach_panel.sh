#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
START_REAL_BRINGUP="${ARACHNE_TEACH_START_REAL_BRINGUP:-true}"
STOP_EXISTING="${ARACHNE_TEACH_STOP_EXISTING:-true}"
STARTUP_CHECKS="${ARACHNE_TEACH_STARTUP_CHECKS:-false}"
STARTUP_CHECK_TIMEOUT="${ARACHNE_TEACH_STARTUP_CHECK_TIMEOUT_SEC:-20}"
mkdir -p "${RECORDING_DIR}"

wait_topic_once() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + STARTUP_CHECK_TIMEOUT))
  echo "Checking ${label}: ${topic}"
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      timeout 5 ros2 topic echo --once "${topic}" >/dev/null && return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for ${label}: ${topic}" >&2
  return 1
}

wait_action_exists() {
  local action="$1"
  local label="$2"
  local deadline=$((SECONDS + STARTUP_CHECK_TIMEOUT))
  echo "Checking ${label}: ${action}"
  while (( SECONDS < deadline )); do
    if ros2 action list 2>/dev/null | grep -Fxq "${action}"; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for ${label}: ${action}" >&2
  return 1
}

PANEL_ARGS=()
while (($#)); do
  case "$1" in
    --panel-only|--no-real-bringup)
      START_REAL_BRINGUP=false
      ;;
    --no-stop-existing)
      STOP_EXISTING=false
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

if [[ "${STOP_EXISTING}" == "true" ]]; then
  "${ROOT_DIR}/scripts/hardware/stop_real_stack.sh" || true
fi

has_recording_dir=false
for arg in "${PANEL_ARGS[@]}"; do
  if [[ "${arg}" == recording_dir:=* || "${arg}" == --recording_dir:=* ]]; then
    has_recording_dir=true
    break
  fi
done

bringup_pid=""
if [[ "${START_REAL_BRINGUP}" == "true" ]]; then
  "${ROOT_DIR}/scripts/hardware/real_bringup.sh" &
  bringup_pid=$!
  trap '[[ -n "${bringup_pid}" ]] && kill "${bringup_pid}" 2>/dev/null || true' EXIT
  bringup_grace_sec="${ARACHNE_TEACH_BRINGUP_GRACE_SEC:-30}"
  for ((i = 0; i < bringup_grace_sec; i++)); do
    sleep 1
    if ! kill -0 "${bringup_pid}" 2>/dev/null; then
      wait "${bringup_pid}" || true
      echo "real_bringup exited before teach panel startup; fix the hardware bringup error first." >&2
      exit 1
    fi
  done
  if [[ "${STARTUP_CHECKS}" == "true" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
    set -u
    if [[ "${USE_AUBO:-true}" == "true" ]]; then
      wait_topic_once "/arachne/hardware/aubo_status" "Aubo status"
      wait_action_exists "/arachne/aubo/move_joint" "Aubo move_joint action"
    fi
    if [[ "${USE_MS42DC:-true}" == "true" ]]; then
      wait_topic_once "/arachne/hardware/gripper_status" "MS42DC status"
    fi
    if [[ "${USE_SCOUT:-true}" == "true" ]]; then
      wait_topic_once "/odom" "base odom"
    fi
  fi
fi

if [[ "${has_recording_dir}" == "true" ]]; then
  exec ros2 launch arachne_operator teach_panel.launch.py "${PANEL_ARGS[@]}"
fi

exec ros2 launch arachne_operator teach_panel.launch.py \
  recording_dir:="${RECORDING_DIR}" "${PANEL_ARGS[@]}"
