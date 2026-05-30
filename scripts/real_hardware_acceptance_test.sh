#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Build arachne_operator first." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/setup.bash"
set -u

CONFIRM_MOTION=false
if [[ "${ARACHNE_CONFIRM_REAL_MOTION:-}" == "YES" ]]; then
  CONFIRM_MOTION=true
fi

if [[ "${CONFIRM_MOTION}" != "true" ]]; then
  cat <<'EOF'
Dry run only. This script will not command real hardware unless explicitly confirmed.

Before real motion:
  1. Run ./scripts/check_real_hardware_env.sh --strict
  2. Bring up the hardware drivers in another terminal.
  3. Keep an emergency stop / power cut within reach.
  4. Run with ARACHNE_CONFIRM_REAL_MOTION=YES.

Example:
  ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_hardware_acceptance_test.sh
EOF
fi

exec ros2 launch arachne_operator real_hardware_acceptance_test.launch.py \
  confirm_motion:="${CONFIRM_MOTION}" "$@"
