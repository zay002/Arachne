#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

"${ROOT_DIR}/scripts/hardware/fetch_third_party.sh"

mkdir -p "${ROOT_DIR}/src/vendor"

ln -sfn ../../third_party/ugv_sdk "${ROOT_DIR}/src/vendor/ugv_sdk"
ln -sfn ../../third_party/scout_ros2/scout_base "${ROOT_DIR}/src/vendor/scout_base"
ln -sfn ../../third_party/scout_ros2/scout_msgs "${ROOT_DIR}/src/vendor/scout_msgs"

for package in \
  aubo_dashboard_msgs \
  aubo_msgs \
  aubo_ros2_driver \
  aubo_moveit_config \
  ros_joints_plan
do
  if [[ -d "${ROOT_DIR}/third_party/aubo_ros2_driver/${package}" ]]; then
    ln -sfn "../../third_party/aubo_ros2_driver/${package}" "${ROOT_DIR}/src/vendor/${package}"
  fi
done

if [[ -f "${ROOT_DIR}/third_party/MS42DC步进电机版柔性机械爪用户资料_V2.2_2024.08.28/5.ROS例程与教程/源码/ROS2.zip" ]]; then
  "${ROOT_DIR}/scripts/hardware/prepare_ms42dc_ros2.sh"
else
  echo "MS42DC ROS2.zip not found locally; skipping step_motor vendor package links." >&2
fi

echo "Real-hardware ROS package links are ready."
echo "Build the runtime packages with:"
echo "  colcon build --base-paths src --packages-select ugv_sdk scout_msgs scout_base serial step_motor arachne_hardware"
echo "Add aubo_ros2_driver packages after installing their SDK dependencies."
