#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

if [[ "${ARACHNE_CONFIRM_AUBO_DRIVER:-}" != "YES" ]]; then
  cat <<EOF
Refusing to start the real Aubo ROS2 driver without confirmation.

The current Aubo driver activates servo mode during ros2_control hardware activation.
It should hold the current joint positions, but it is still a real-hardware control mode.

Before running:
  1. Confirm the robot workspace is clear.
  2. Keep the emergency stop or power cut within reach.
  3. Use the teach pendant/control cabinet to complete connect -> power on -> start.
  4. Run ./scripts/real_aubo_prepare.sh and confirm RobotMode=Running.

Start driver:
  ARACHNE_CONFIRM_AUBO_DRIVER=YES AUBO_ROBOT_IP=${AUBO_ROBOT_IP} ./scripts/real_aubo_bringup.sh
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
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/setup.bash"
set -u

"${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"

exec ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false \
  use_ms42dc:=false \
  use_aubo:=true \
  aubo_robot_ip:="${AUBO_ROBOT_IP}" \
  "$@"
