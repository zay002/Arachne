#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

cat <<'EOF'
Aubo startup readiness check (read-only)

This script does not power on, release brakes, clear protective stops, or change
servo mode. For safety, perform connect -> power on -> start on the teach
pendant/control cabinet, then use this script to verify the controller reports
RobotMode=Running and SafetyMode=Normal/ReducedMode.
EOF

exec "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/hardware/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}" "$@"
