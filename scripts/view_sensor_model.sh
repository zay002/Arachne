#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"

export USE_GUI="${USE_GUI:-true}"
export WITH_RVIZ="${WITH_RVIZ:-true}"
export WITH_BASE_SIM="${WITH_BASE_SIM:-false}"
export WITH_BASE_GUI="${WITH_BASE_GUI:-false}"
export WITH_GRIPPER_SIM="${WITH_GRIPPER_SIM:-false}"
export WITH_GRIPPER_GUI="${WITH_GRIPPER_GUI:-false}"
export WITH_LIDAR="${WITH_LIDAR:-true}"
export WITH_EE_CAMERA="${WITH_EE_CAMERA:-true}"
export WITH_REAR_RACK="${WITH_REAR_RACK:-true}"

exec "${ROOT_DIR}/scripts/view_model.sh"
