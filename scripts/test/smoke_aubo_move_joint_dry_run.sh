#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

echo "DRY RUN ONLY: this script must not connect to real Aubo or send real motion."

LOG_DIR="${ROOT_DIR}/log/offline_smoke"
mkdir -p "$LOG_DIR"
SERVER_LOG="${LOG_DIR}/aubo_move_joint_dry_run_server.log"
GOAL_LOG="${LOG_DIR}/aubo_move_joint_dry_run_goal.log"

set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run arachne_hardware aubo_move_joint_action_server \
  --ros-args \
  -p dry_run:=true \
  -p action_name:=/arachne/aubo/move_joint \
  >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!

for _ in {1..40}; do
  if ros2 action list 2>/dev/null | grep -q '^/arachne/aubo/move_joint$'; then
    break
  fi
  sleep 0.25
done

if ! ros2 action list 2>/dev/null | grep -q '^/arachne/aubo/move_joint$'; then
  echo "dry-run action server did not become available" >&2
  tail -80 "$SERVER_LOG" >&2 || true
  exit 1
fi

timeout 15s ros2 action send_goal \
  /arachne/aubo/move_joint \
  arachne_hardware/action/AuboMoveJoint \
  "{target_joints: [0,0,0,0,0,0], speed_rad_sec: 0.1, accel_rad_sec2: 0.1, blend_radius: 0.0, duration_sec: 0.0, goal_tolerance_rad: 0.04, timeout_sec: 3.0, label: 'dry_run_smoke_test'}" \
  >"$GOAL_LOG" 2>&1

if ! grep -Eq 'success[=:] true|success[=:] True|success: true' "$GOAL_LOG"; then
  echo "dry-run goal result did not contain success=true" >&2
  cat "$GOAL_LOG" >&2
  exit 1
fi

if ! grep -q 'dry-run completed' "$GOAL_LOG"; then
  echo "dry-run goal result did not contain expected message" >&2
  cat "$GOAL_LOG" >&2
  exit 1
fi

echo "[smoke_aubo_move_joint_dry_run] passed"
