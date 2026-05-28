#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${ROOT_DIR}/third_party" "${ROOT_DIR}/src/vendor"

fetch_repo() {
  local name="$1"
  local url="$2"
  local ref="$3"
  local dest="${ROOT_DIR}/third_party/${name}"

  if [[ ! -d "${dest}/.git" ]]; then
    rm -rf "${dest}"
    git init "${dest}"
    git -C "${dest}" remote add origin "${url}"
  fi

  git -C "${dest}" remote set-url origin "${url}"

  local current_ref=""
  current_ref="$(git -C "${dest}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${current_ref}" == "${ref}" ]]; then
    return
  fi

  if [[ -n "$(git -C "${dest}" status --porcelain)" ]]; then
    echo "Refusing to overwrite dirty third-party repo: ${dest}" >&2
    echo "Commit/stash local changes there, or remove the directory and rerun this script." >&2
    exit 1
  fi

  git -C "${dest}" fetch --depth 1 origin "${ref}"
  git -C "${dest}" checkout --detach FETCH_HEAD
}

fetch_repo aubo_description \
  https://github.com/AuboRobot/aubo_description.git \
  47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1

fetch_repo scout_ros2 \
  https://github.com/agilexrobotics/scout_ros2.git \
  bdbb90471613831fb0b2ec01fecac043445313c4

fetch_repo ugv_sdk \
  https://github.com/agilexrobotics/ugv_sdk.git \
  c3dfaf444f9bae10757e546acae055aaf4a13de7

fetch_repo aubo_ros2_driver \
  https://github.com/AuboRobot/aubo_ros2_driver.git \
  85684075d6ff06c5385e39611208e99ebf0f94c6

fetch_repo dh_ag95_gripper_ros2 \
  https://github.com/ian-chuang/dh_ag95_gripper_ros2.git \
  fc4f80fdfb3acae5626df4359aec1401cb71a9a3

ln -sfn ../../third_party/aubo_description "${ROOT_DIR}/src/vendor/aubo_description"
ln -sfn ../../third_party/dh_ag95_gripper_ros2/dh_ag95_description "${ROOT_DIR}/src/vendor/dh_ag95_description"
ln -sfn ../../third_party/scout_ros2/scout_description "${ROOT_DIR}/src/vendor/scout_description"

echo "Third-party model and hardware ROS packages are ready at pinned revisions."
