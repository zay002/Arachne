#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"

URDF_OUT="${URDF_OUT:-/tmp/arachne.urdf}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"

if ROS_DISTRO="$(arachne_detect_ros_distro 2>/dev/null)"; then
  export ROS_DISTRO
  arachne_source_bash_file "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  arachne_source_bash_file "${ROOT_DIR}/install/setup.bash"
fi

if ! command -v xacro >/dev/null; then
  echo "xacro is not installed. Run scripts/build/setup_ubuntu.sh first." >&2
  exit 1
fi

xacro "${ROOT_DIR}/src/arachne_description/urdf/arachne.urdf.xacro" gripper_type:="${GRIPPER_TYPE}" > "${URDF_OUT}"
echo "Generated ${URDF_OUT}"

if command -v check_urdf >/dev/null; then
  check_urdf "${URDF_OUT}"
else
  echo "check_urdf is not installed; skipped structural URDF validation."
fi
