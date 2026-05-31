#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_TEACH_FLAG_PATH="${AUBO_TEACH_FLAG_PATH:-/tmp/arachne_aubo_teach_mode}"
AUBO_KEEP_TEACH_FLAG="${AUBO_KEEP_TEACH_FLAG:-false}"

if [[ "${ARACHNE_CONFIRM_AUBO_DRIVER:-}" != "YES" ]]; then
  cat <<EOF
Refusing to start the real Aubo ROS2 driver without confirmation.

The Aubo driver is a real-hardware control mode. In normal mode it requires the
arm to already be Running. For remote startup, use ARACHNE_AUBO_ALLOW_PRESTART=YES
so the driver can start controllers and hold measured joint positions before
the RobotManage.startup lifecycle step.

Before running:
  1. Confirm the robot workspace is clear.
  2. Keep the emergency stop or power cut within reach.
  3. Either complete connect -> power on -> start on the teach pendant/control cabinet,
     or use the guarded remote startup flow in ./scripts/real_aubo_remote_start.sh.
  4. Confirm RobotMode/SafetyMode using ./scripts/real_aubo_prepare.sh.

Start driver:
  ARACHNE_CONFIRM_AUBO_DRIVER=YES AUBO_ROBOT_IP=${AUBO_ROBOT_IP} ./scripts/real_aubo_bringup.sh

Remote-start driver terminal:
  ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES AUBO_ROBOT_IP=${AUBO_ROBOT_IP} ./scripts/real_aubo_bringup.sh
EOF
  exit 2
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Build aubo_ros2_driver and arachne_hardware first." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
set -u

if [[ "${ARACHNE_AUBO_ALLOW_PRESTART:-}" == "YES" ]]; then
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}" --allow-not-running
else
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"
fi

if [[ "${AUBO_KEEP_TEACH_FLAG}" != "true" ]]; then
  rm -f "${AUBO_TEACH_FLAG_PATH}"
fi

exec ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false \
  use_ms42dc:=false \
  use_aubo:=true \
  aubo_robot_ip:="${AUBO_ROBOT_IP}" \
  "$@"
