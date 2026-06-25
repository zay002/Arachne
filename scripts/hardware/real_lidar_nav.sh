#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

WITH_LSLIDAR_DRIVER="${ARACHNE_NAV_WITH_LSLIDAR_DRIVER:-true}"
WITH_POINTCLOUD_TO_SCAN="${ARACHNE_NAV_WITH_POINTCLOUD_TO_SCAN:-true}"
WITH_RSP="${ARACHNE_NAV_WITH_ROBOT_STATE_PUBLISHER:-false}"
WITH_RVIZ="${ARACHNE_NAV_WITH_RVIZ:-true}"
RVIZ_CONFIG="${ARACHNE_NAV_RVIZ_CONFIG:-}"
PARAMS_FILE="${ARACHNE_NAV_PARAMS_FILE:-}"
USE_COMPOSITION="${ARACHNE_NAV_USE_COMPOSITION:-False}"
LOG_LEVEL="${ARACHNE_NAV_LOG_LEVEL:-warn}"
DEFAULT_MAP_FILE="${ROOT_DIR}/src/arachne_nav/maps/road_lab_apriltag.yaml"
NAV_MODE="${ARACHNE_NAV_MODE:-localization}"
MAP_FILE="${ARACHNE_NAV_MAP:-${DEFAULT_MAP_FILE}}"

case "${NAV_MODE}" in
  mapping)
    SLAM="true"
    RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/arachne_nav/rviz/arachne_mapping_lidar.rviz}"
    if [[ "${MAP_FILE}" == "${DEFAULT_MAP_FILE}" && -z "${ARACHNE_NAV_MAP:-}" ]]; then
      MAP_FILE=""
    fi
    ;;
  localization|localisation|loc)
    SLAM="false"
    RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/arachne_nav/rviz/arachne_nav_topdown.rviz}"
    if [[ -z "${MAP_FILE}" ]]; then
      echo "ARACHNE_NAV_MODE=localization requires ARACHNE_NAV_MAP=/path/to/map.yaml" >&2
      exit 2
    fi
    if [[ ! -f "${MAP_FILE}" ]]; then
      echo "Localization map does not exist: ${MAP_FILE}" >&2
      exit 2
    fi
    ;;
  *)
    echo "Unsupported ARACHNE_NAV_MODE=${NAV_MODE}; expected mapping or localization" >&2
    exit 2
    ;;
esac

echo "Arachne lidar nav:"
echo "  mode=${NAV_MODE}"
echo "  slam=${SLAM}"
echo "  map=${MAP_FILE:-<none>}"
echo "  params_file=${PARAMS_FILE:-<default>}"
echo "  rviz_config=${RVIZ_CONFIG}"
echo "  lslidar_driver=${WITH_LSLIDAR_DRIVER}"

LAUNCH_ARGS=(
  "slam:=${SLAM}"
  "rviz_config:=${RVIZ_CONFIG}"
  "with_lslidar_driver:=${WITH_LSLIDAR_DRIVER}"
  "with_pointcloud_to_scan:=${WITH_POINTCLOUD_TO_SCAN}"
  "with_robot_state_publisher:=${WITH_RSP}"
  "with_rviz:=${WITH_RVIZ}"
  "use_composition:=${USE_COMPOSITION}"
  "log_level:=${LOG_LEVEL}"
)
if [[ -n "${PARAMS_FILE}" ]]; then
  LAUNCH_ARGS+=("params_file:=${PARAMS_FILE}")
fi
if [[ -n "${MAP_FILE}" ]]; then
  LAUNCH_ARGS+=("map:=${MAP_FILE}")
fi

exec ros2 launch arachne_nav nav2_lidar.launch.py "${LAUNCH_ARGS[@]}" "$@"
