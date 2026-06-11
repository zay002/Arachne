#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

WITH_LSLIDAR_DRIVER="${ARACHNE_NAV_WITH_LSLIDAR_DRIVER:-false}"
WITH_RSP="${ARACHNE_NAV_WITH_ROBOT_STATE_PUBLISHER:-false}"
USE_COMPOSITION="${ARACHNE_NAV_USE_COMPOSITION:-true}"
LOG_LEVEL="${ARACHNE_NAV_LOG_LEVEL:-warn}"

exec ros2 launch arachne_nav nav2_lidar.launch.py \
  with_lslidar_driver:="${WITH_LSLIDAR_DRIVER}" \
  with_robot_state_publisher:="${WITH_RSP}" \
  use_composition:="${USE_COMPOSITION}" \
  log_level:="${LOG_LEVEL}" \
  "$@"
