#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URDF_OUT="${URDF_OUT:-/tmp/arachne.urdf}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"

if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
fi

if ! command -v xacro >/dev/null; then
  echo "xacro is not installed. Run scripts/setup_ubuntu.sh first." >&2
  exit 1
fi

xacro "${ROOT_DIR}/src/arachne_description/urdf/arachne.urdf.xacro" gripper_type:="${GRIPPER_TYPE}" > "${URDF_OUT}"
echo "Generated ${URDF_OUT}"

if command -v check_urdf >/dev/null; then
  check_urdf "${URDF_OUT}"
else
  echo "check_urdf is not installed; skipped structural URDF validation."
fi
