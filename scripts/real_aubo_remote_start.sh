#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/ros_env.sh"

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

if [[ "${ARACHNE_CONFIRM_AUBO_REMOTE_START:-}" != "YES" ]]; then
  cat <<EOF
Refusing to remotely prepare the real Aubo arm without confirmation.

This script is intentionally blocking: every step waits for the matching ROS or
robot-controller state and aborts on timeout before the next step can run.
It never calls releaseRobotBrake directly. Aubo documents RobotManage.startup
as the lifecycle call that starts the robot and releases the brake; using only
releaseRobotBrake bypasses that startup path and is unsafe for this project.

Required safe order:
  1. Start the Aubo ROS driver in prestart mode.
  2. Confirm joint_state_broadcaster and joint_trajectory_controller are active.
  3. Read current joint angles.
  4. Send a hold-position action goal at the measured joint angles.
  5. Power on and wait for Idle.
  6. Re-send hold-position.
  7. Call RobotManage.startup and wait for Running.
  8. Verify steady joint feedback and hold-position again.

Driver terminal:
  ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES ./scripts/real_aubo_bringup.sh

Startup terminal:
  ARACHNE_CONFIRM_AUBO_REMOTE_START=YES AUBO_ROBOT_IP=${AUBO_ROBOT_IP} ./scripts/real_aubo_remote_start.sh
EOF
  exit 2
fi

arachne_require_ros_distro

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
set -u

arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Build arachne_operator and aubo_ros2_driver first."

exec "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_remote_start.py" --ip "${AUBO_ROBOT_IP}" "$@"
