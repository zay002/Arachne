#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

WITH_LSLIDAR_DRIVER="${ARACHNE_NAV_WITH_LSLIDAR_DRIVER:-false}"
WITH_POINTCLOUD_TO_SCAN="${ARACHNE_NAV_WITH_POINTCLOUD_TO_SCAN:-true}"
WITH_RSP="${ARACHNE_NAV_WITH_ROBOT_STATE_PUBLISHER:-false}"
WITH_RVIZ="${ARACHNE_NAV_WITH_RVIZ:-true}"
USE_COMPOSITION="${ARACHNE_NAV_USE_COMPOSITION:-True}"
LOG_LEVEL="${ARACHNE_NAV_LOG_LEVEL:-warn}"

exec ros2 launch arachne_nav nav2_lidar.launch.py \
  with_lslidar_driver:="${WITH_LSLIDAR_DRIVER}" \
  with_pointcloud_to_scan:="${WITH_POINTCLOUD_TO_SCAN}" \
  with_robot_state_publisher:="${WITH_RSP}" \
  with_rviz:="${WITH_RVIZ}" \
  use_composition:="${USE_COMPOSITION}" \
  log_level:="${LOG_LEVEL}" \
  "$@"
