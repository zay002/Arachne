#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${ROOT_DIR}/third_party" "${ROOT_DIR}/src/vendor"

if [[ ! -d "${ROOT_DIR}/third_party/aubo_description/.git" ]]; then
  git clone --depth 1 https://github.com/AuboRobot/aubo_description.git \
    "${ROOT_DIR}/third_party/aubo_description"
fi

if [[ ! -d "${ROOT_DIR}/third_party/scout_ros2/.git" ]]; then
  git clone --depth 1 https://github.com/agilexrobotics/scout_ros2.git \
    "${ROOT_DIR}/third_party/scout_ros2"
fi

if [[ ! -d "${ROOT_DIR}/third_party/dh_ag95_gripper_ros2/.git" ]]; then
  git clone --depth 1 -b humble https://github.com/ian-chuang/dh_ag95_gripper_ros2.git \
    "${ROOT_DIR}/third_party/dh_ag95_gripper_ros2" \
    || git clone --depth 1 https://github.com/ian-chuang/dh_ag95_gripper_ros2.git \
      "${ROOT_DIR}/third_party/dh_ag95_gripper_ros2"
fi

ln -sfn ../../third_party/aubo_description "${ROOT_DIR}/src/vendor/aubo_description"
ln -sfn ../../third_party/dh_ag95_gripper_ros2/dh_ag95_description "${ROOT_DIR}/src/vendor/dh_ag95_description"
ln -sfn ../../third_party/scout_ros2/scout_description "${ROOT_DIR}/src/vendor/scout_description"

echo "Third-party model packages are ready."
