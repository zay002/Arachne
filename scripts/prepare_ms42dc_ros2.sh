#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ZIP_PATH="${MS42DC_ROS2_ZIP:-${ROOT_DIR}/third_party/MS42DC步进电机版柔性机械爪用户资料_V2.2_2024.08.28/5.ROS例程与教程/源码/ROS2.zip}"
DEST_DIR="${ROOT_DIR}/third_party/ms42dc_step_motor_ros2"

if [[ ! -f "${ZIP_PATH}" ]]; then
  echo "MS42DC ROS2 source zip not found:" >&2
  echo "  ${ZIP_PATH}" >&2
  echo "Set MS42DC_ROS2_ZIP=/path/to/ROS2.zip or place the vendor 자료 under third_party/." >&2
  exit 1
fi

mkdir -p "${DEST_DIR}" "${ROOT_DIR}/src/vendor"
find "${DEST_DIR}" -mindepth 1 -maxdepth 1 ! -name .gitkeep -exec rm -rf {} +
unzip -q "${ZIP_PATH}" -d "${DEST_DIR}"

ln -sfn ../../third_party/ms42dc_step_motor_ros2/Step_Motor_ROS2/src/serial_ros2 \
  "${ROOT_DIR}/src/vendor/serial"
ln -sfn ../../third_party/ms42dc_step_motor_ros2/Step_Motor_ROS2/src/step_motor \
  "${ROOT_DIR}/src/vendor/step_motor"
ln -sfn ../../third_party/ms42dc_step_motor_ros2/Step_Motor_ROS2/src/wheeltec_robot_keyboard \
  "${ROOT_DIR}/src/vendor/wheeltec_robot_keyboard"

echo "Prepared MS42DC vendor ROS2 packages from ${ZIP_PATH}"
echo "Linked: serial, step_motor, wheeltec_robot_keyboard"
