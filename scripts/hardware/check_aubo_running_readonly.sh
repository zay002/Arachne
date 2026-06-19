#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_RPC_PORT="${AUBO_RPC_PORT:-30004}"
AUBO_PROBE_TIMEOUT_SEC="${AUBO_PROBE_TIMEOUT_SEC:-1.0}"

echo "[check_aubo_running_readonly] loading environment"
set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
else
  echo "[check_aubo_running_readonly] install/setup.bash not found; ROS graph checks may fail"
fi
set -u

echo "[check_aubo_running_readonly] target Aubo IP: ${AUBO_ROBOT_IP}"
echo "[check_aubo_running_readonly] read-only only: no goals, no owner claim, no teach gate writes"

echo
echo "== stale teach gate / control owner files =="
ls -l /tmp/arachne_aubo_teach_mode /tmp/arachne_aubo_control_owner 2>/dev/null || true
echo "[check_aubo_running_readonly] shown only; not deleting or modifying these files"

echo
echo "== TCP ${AUBO_RPC_PORT} connectivity =="
"${ARACHNE_SYSTEM_PYTHON:-python3}" - "$AUBO_ROBOT_IP" "$AUBO_RPC_PORT" "$AUBO_PROBE_TIMEOUT_SEC" <<'PY'
import socket
import sys

ip = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
with socket.create_connection((ip, port), timeout=timeout):
    print(f"{ip}:{port} open")
PY

echo
echo "== read-only Aubo RobotState =="
probe_output="$("${ARACHNE_SYSTEM_PYTHON:-python3}" "$ROOT_DIR/scripts/hardware/real_aubo_probe.py" \
  --ip "$AUBO_ROBOT_IP" \
  --timeout "$AUBO_PROBE_TIMEOUT_SEC" \
  --ports "$AUBO_RPC_PORT")"
printf '%s\n' "$probe_output"

robot_mode="$(printf '%s\n' "$probe_output" | awk -F': ' '/getRobotModeType/ {print $2; exit}')"
safety_mode="$(printf '%s\n' "$probe_output" | awk -F': ' '/getSafetyModeType/ {print $2; exit}')"
if [[ "${robot_mode:-}" == "Running" ]]; then
  echo "[check_aubo_running_readonly] RobotMode is Running"
else
  echo "[check_aubo_running_readonly] RobotMode is ${robot_mode:-unknown}; reporting only, not starting Aubo"
fi
if [[ "${safety_mode:-}" == "Normal" ]]; then
  echo "[check_aubo_running_readonly] SafetyMode is Normal"
else
  echo "[check_aubo_running_readonly] SafetyMode is ${safety_mode:-unknown}; operator review required"
fi

echo
echo "== ROS graph observations =="
if ros2 topic list | grep -q '^/joint_states$'; then
  echo "/joint_states present"
  timeout 5 ros2 topic echo /joint_states --once || true
else
  echo "/joint_states not present"
fi

if ros2 topic list | grep -q '^/arachne/hardware/aubo_status$'; then
  echo "/arachne/hardware/aubo_status present"
  timeout 5 ros2 topic echo /arachne/hardware/aubo_status --once || true
else
  echo "/arachne/hardware/aubo_status not present"
fi

if ros2 action list | grep -q '^/arachne/aubo/move_joint$'; then
  echo "/arachne/aubo/move_joint action present"
  ros2 action info /arachne/aubo/move_joint || true
else
  echo "/arachne/aubo/move_joint action not present"
fi

echo
echo "[check_aubo_running_readonly] completed. No motion commands were sent."
