#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

cat <<'EOF'
Aubo startup readiness check (read-only)

This script does not power on, release brakes, clear protective stops, or change
servo mode. For safety, perform connect -> power on -> start on the teach
pendant/control cabinet, then use this script to verify the controller reports
RobotMode=Running and SafetyMode=Normal/ReducedMode.
EOF

exec "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}" "$@"
