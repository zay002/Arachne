#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

echo "OFFLINE ONLY: this script must not call start_visual_grasp or start_road_cleanup."

LOG_DIR="${ROOT_DIR}/log/offline_smoke"
mkdir -p "$LOG_DIR"
ORCH_LOG="${LOG_DIR}/demo_orchestrator_offline.log"
STATUS_LOG="${LOG_DIR}/demo_orchestrator_status.log"
PREFLIGHT_LOG="${LOG_DIR}/demo_orchestrator_preflight.log"

set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

cleanup() {
  if [[ -n "${ORCH_PID:-}" ]] && kill -0 "$ORCH_PID" 2>/dev/null; then
    kill "$ORCH_PID" 2>/dev/null || true
    wait "$ORCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch arachne_operator demo_orchestrator.launch.py autostart:=false \
  >"$ORCH_LOG" 2>&1 &
ORCH_PID=$!

for _ in {1..40}; do
  if ros2 service list 2>/dev/null | grep -q '^/arachne/demo/status$'; then
    break
  fi
  sleep 0.25
done

if ! ros2 service list 2>/dev/null | grep -q '^/arachne/demo/status$'; then
  echo "demo_orchestrator status service did not become available" >&2
  tail -100 "$ORCH_LOG" >&2 || true
  exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -q '^/arachne/demo/state$'; then
  echo "/arachne/demo/state topic is not present" >&2
  ros2 topic list >&2 || true
  exit 1
fi

timeout 10s ros2 service call /arachne/demo/status std_srvs/srv/Trigger '{}' \
  >"$STATUS_LOG" 2>&1

timeout 10s ros2 service call /arachne/demo/preflight std_srvs/srv/Trigger '{}' \
  >"$PREFLIGHT_LOG" 2>&1

if ! grep -q 'success=True' "$STATUS_LOG" && ! grep -q 'success: true' "$STATUS_LOG"; then
  echo "demo status did not return success=true" >&2
  cat "$STATUS_LOG" >&2
  exit 1
fi

if ! grep -q 'checks' "$PREFLIGHT_LOG"; then
  echo "demo preflight response did not contain checks payload" >&2
  cat "$PREFLIGHT_LOG" >&2
  exit 1
fi

echo "[smoke_demo_orchestrator_offline] passed"
