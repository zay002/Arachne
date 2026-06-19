#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_RPC_PORT="${AUBO_RPC_PORT:-30004}"
AUBO_PROBE_TIMEOUT_SEC="${AUBO_PROBE_TIMEOUT_SEC:-1.0}"

echo "[check_aubo_readonly] loading environment"
set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
else
  echo "[check_aubo_readonly] install/setup.bash not found; ROS package checks may fail"
fi
set -u

echo "[check_aubo_readonly] target Aubo IP: ${AUBO_ROBOT_IP}"
echo "[check_aubo_readonly] no motion commands will be sent"

echo
echo "== stale teach gate / control owner files =="
ls -l /tmp/arachne_aubo_teach_mode /tmp/arachne_aubo_control_owner 2>/dev/null || true
echo "[check_aubo_readonly] if stale files are shown above, remove them only after operator confirmation"

echo
echo "== network ping =="
ping -c 2 -W 1 "$AUBO_ROBOT_IP"

echo
echo "== TCP ${AUBO_RPC_PORT} connectivity =="
"${ARACHNE_SYSTEM_PYTHON:-python3}" - "$AUBO_ROBOT_IP" "$AUBO_RPC_PORT" "$AUBO_PROBE_TIMEOUT_SEC" <<'PY'
import socket
import sys

ip = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
try:
    with socket.create_connection((ip, port), timeout=timeout):
        print(f"{ip}:{port} open")
except Exception as exc:
    print(f"{ip}:{port} failed: {type(exc).__name__}: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY

echo
echo "== read-only Aubo JSON-RPC probe =="
"${ARACHNE_SYSTEM_PYTHON:-python3}" "$ROOT_DIR/scripts/hardware/real_aubo_probe.py" \
  --ip "$AUBO_ROBOT_IP" \
  --timeout "$AUBO_PROBE_TIMEOUT_SEC" \
  --ports "$AUBO_RPC_PORT"

echo
echo "== ROS interface =="
ros2 interface show arachne_hardware/action/AuboMoveJoint >/dev/null
echo "arachne_hardware/action/AuboMoveJoint available"

echo
echo "== optional ROS graph observations =="
if ros2 topic list | grep -q '^/joint_states$'; then
  echo "/joint_states present"
else
  echo "/joint_states not present; start real_bringup for ROS graph checks"
fi

if ros2 topic list | grep -q '^/arachne/hardware/aubo_status$'; then
  echo "/arachne/hardware/aubo_status present"
else
  echo "/arachne/hardware/aubo_status not present; start real_bringup for ROS graph checks"
fi

if ros2 action list | grep -q '^/arachne/aubo/move_joint$'; then
  echo "/arachne/aubo/move_joint action present"
else
  echo "/arachne/aubo/move_joint action not present; start real_bringup with aubo_move_joint_dry_run:=true for dry-run graph checks"
fi

echo
echo "[check_aubo_readonly] read-only checks completed. No action goals or motion commands were sent."
