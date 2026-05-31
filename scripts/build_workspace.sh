#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
export ARACHNE_ENV_NO_WORKSPACE=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
unset ARACHNE_ENV_NO_WORKSPACE
set -u

cd "${ROOT_DIR}"

if [[ -f build/aubo_description/cmake_install.cmake ]] \
  && grep -q 'ament_cmake_symlink_install' build/aubo_description/cmake_install.cmake; then
  echo "Removing stale symlink-install cache for aubo_description."
  rm -rf build/aubo_description install/aubo_description
fi

colcon build --base-paths src --packages-select \
  aubo_description scout_description dh_ag95_description \
  arachne_sim arachne_gripper arachne_hardware arachne_control arachne_moveit_config \
  arachne_nav arachne_operator arachne_description arachne_gazebo arachne_demo \
  --cmake-args -DPython3_EXECUTABLE="${ARACHNE_SYSTEM_PYTHON}"
