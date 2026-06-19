#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

echo "[check_aubo_action_stack] loading environment"
set +u
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
else
  echo "[check_aubo_action_stack] install/setup.bash not found; using sourced ROS/workspace environment only"
fi
set -u

echo "[check_aubo_action_stack] checking key files"
required_files=(
  "src/arachne_hardware/action/AuboMoveJoint.action"
  "src/arachne_hardware/arachne_hardware/aubo_move_joint_action_server.py"
  "src/arachne_operator/arachne_operator/aubo_move_joint_client.py"
  "src/arachne_operator/arachne_operator/demo_orchestrator.py"
)

for path in "${required_files[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "[check_aubo_action_stack] missing required file: $path" >&2
    exit 1
  fi
done

echo "[check_aubo_action_stack] compiling Python modules"
"${ARACHNE_SYSTEM_PYTHON:-python3}" -m compileall \
  src/arachne_hardware/arachne_hardware \
  src/arachne_operator/arachne_operator \
  scripts/vision

echo "[check_aubo_action_stack] checking ROS action interface"
ros2 interface show arachne_hardware/action/AuboMoveJoint >/dev/null

echo "[check_aubo_action_stack] checking ROS executables"
ros2 pkg executables arachne_hardware | grep -q 'aubo_move_joint_action_server'
ros2 pkg executables arachne_operator | grep -q 'demo_orchestrator'

echo "[check_aubo_action_stack] Aubo action stack checks passed."
