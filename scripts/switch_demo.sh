#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEMO_MODE="${DEMO_MODE:-rviz}"
GRIPPER_TYPE="${GRIPPER_TYPE:-ms42dc}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run the README build command first." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u

GZ_RESOURCE_DIRS=()
for package_name in arachne_description aubo_description scout_description dh_ag95_description; do
  package_share_parent="${ROOT_DIR}/install/${package_name}/share"
  if [[ -d "${package_share_parent}" ]]; then
    GZ_RESOURCE_DIRS+=("${package_share_parent}")
  fi
done
if [[ ${#GZ_RESOURCE_DIRS[@]} -gt 0 ]]; then
  GZ_RESOURCE_PATH="$(IFS=:; echo "${GZ_RESOURCE_DIRS[*]}")${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
  export GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_PATH}"
fi

case "${DEMO_MODE}" in
  rviz)
    exec ros2 launch arachne_demo switch_rviz_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}"
    ;;
  gazebo)
    exec ros2 launch arachne_demo switch_gazebo_demo.launch.py \
      gripper_type:="${GRIPPER_TYPE}" \
      joy_dev:="${JOY_DEV}"
    ;;
  *)
    echo "DEMO_MODE must be 'rviz' or 'gazebo'. Current value: ${DEMO_MODE}" >&2
    exit 1
    ;;
esac
