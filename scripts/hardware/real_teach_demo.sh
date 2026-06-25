#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
USE_SCOUT="${USE_SCOUT:-true}"
USE_MS42DC="${USE_MS42DC:-true}"
USE_AUBO="${USE_AUBO:-true}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-35}"
RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
BRINGUP_ARGS=()
PANEL_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/hardware/real_teach_demo.sh [options] [-- panel_launch_args...]

One-command real-hardware teach demo:
  1. auto-detect Scout/MS42DC serial ports,
  2. start real bringup,
  3. wait for core ROS interfaces,
  4. open the teach/replay panel,
  5. stop bringup when the panel exits.

Options are forwarded to real_bringup.sh:
  --no-scout, --no-ms42dc, --no-gripper, --no-aubo, --skip-aubo-check

Example:
  ./scripts/hardware/real_teach_demo.sh
  ./scripts/hardware/real_teach_demo.sh -- recording_dir:=recordings/demo_day_1

Recordings are saved locally under recordings/teach by default.
EOF
}

while (($#)); do
  case "$1" in
    --)
      shift
      PANEL_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --no-scout)
      USE_SCOUT=false
      BRINGUP_ARGS+=("$1")
      ;;
    --no-ms42dc|--no-gripper)
      USE_MS42DC=false
      BRINGUP_ARGS+=("$1")
      ;;
    --no-aubo)
      USE_AUBO=false
      BRINGUP_ARGS+=("$1")
      ;;
    --skip-aubo-check)
      BRINGUP_ARGS+=("$1")
      ;;
    *)
      BRINGUP_ARGS+=("$1")
      ;;
  esac
  shift
done

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

BRINGUP_LOG_DIR="${ROOT_DIR}/log/real_teach_demo"
mkdir -p "${BRINGUP_LOG_DIR}"
mkdir -p "${RECORDING_DIR}"
BRINGUP_LOG="${BRINGUP_LOG_DIR}/bringup_$(date +%Y%m%d_%H%M%S).log"

has_recording_dir=false
for arg in "${PANEL_ARGS[@]}"; do
  if [[ "${arg}" == recording_dir:=* || "${arg}" == --recording_dir:=* ]]; then
    has_recording_dir=true
    break
  fi
done

cleanup() {
  if [[ -n "${BRINGUP_PID:-}" ]] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
    echo "Stopping real bringup..."
    kill -INT "${BRINGUP_PID}" 2>/dev/null || true
    sleep 2
    kill "${BRINGUP_PID}" 2>/dev/null || true
    wait "${BRINGUP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if timeout 2 ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      echo "  ready: ${label} (${topic})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} (${topic}). See ${BRINGUP_LOG}" >&2
  return 1
}

wait_for_action() {
  local action="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if timeout 2 ros2 action list 2>/dev/null | grep -qx "${action}"; then
      echo "  ready: ${label} (${action})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} (${action}). See ${BRINGUP_LOG}" >&2
  return 1
}

echo "Starting real bringup; log: ${BRINGUP_LOG}"
USE_SCOUT="${USE_SCOUT}" USE_MS42DC="${USE_MS42DC}" USE_AUBO="${USE_AUBO}" \
  "${ROOT_DIR}/scripts/hardware/real_bringup.sh" "${BRINGUP_ARGS[@]}" > >(tee "${BRINGUP_LOG}") 2>&1 &
BRINGUP_PID=$!

echo "Waiting for hardware interfaces..."
if [[ "${USE_SCOUT}" == "true" ]]; then
  wait_for_topic "/odom" "Scout odometry"
fi
if [[ "${USE_MS42DC}" == "true" ]]; then
  wait_for_topic "/arachne/hardware/gripper_status" "MS42DC status"
fi
if [[ "${USE_AUBO}" == "true" ]]; then
  wait_for_topic "/joint_states" "Aubo joint states"
  wait_for_action "/joint_trajectory_controller/follow_joint_trajectory" "Aubo trajectory action"
fi

echo "Opening teach/replay panel..."
if [[ "${has_recording_dir}" == "true" ]]; then
  ros2 launch arachne_operator teach_panel.launch.py "${PANEL_ARGS[@]}"
else
  ros2 launch arachne_operator teach_panel.launch.py \
    recording_dir:="${RECORDING_DIR}" "${PANEL_ARGS[@]}"
fi
