#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
if [[ -f "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh"
fi
set -u

PYTHON_USER_SITE="$("${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}" - <<'PY'
import site
print(site.getusersitepackages())
PY
)"
if [[ -d "${PYTHON_USER_SITE}" ]]; then
  export PYTHONPATH="${PYTHON_USER_SITE}:${PYTHONPATH:-}"
fi

IMAGE_TOPIC="${ARACHNE_APRILTAG_NAV_IMAGE_TOPIC:-/camera/color/image_raw}"
CAMERA_INFO_TOPIC="${ARACHNE_APRILTAG_NAV_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
BASE_FRAME="${ARACHNE_APRILTAG_NAV_BASE_FRAME:-base_link}"
ODOM_FRAME="${ARACHNE_APRILTAG_NAV_ODOM_FRAME:-odom}"
CAMERA_FRAME="${ARACHNE_APRILTAG_NAV_CAMERA_FRAME:-}"
TAG_FAMILY="${ARACHNE_APRILTAG_NAV_TAG_FAMILY:-tagStandard41h12}"
TAG_ID="${ARACHNE_APRILTAG_NAV_TAG_ID:--1}"
TAG_SIZE_M="${ARACHNE_APRILTAG_NAV_TAG_SIZE_M:-0.070}"
BOARD_IMAGE_PATH="${ARACHNE_APRILTAG_NAV_BOARD_IMAGE_PATH:-/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png}"
BOARD_WIDTH_M="${ARACHNE_APRILTAG_NAV_BOARD_WIDTH_M:-0.420}"
BOARD_HEIGHT_M="${ARACHNE_APRILTAG_NAV_BOARD_HEIGHT_M:-0.297}"
TAG_MAP_XYZ="${ARACHNE_APRILTAG_NAV_TAG_MAP_XYZ:-0.0,0.0,1.2}"
TAG_MAP_RPY="${ARACHNE_APRILTAG_NAV_TAG_MAP_RPY:-0.0,-1.57079632679,0.0}"
TIMEOUT_SEC="${ARACHNE_APRILTAG_NAV_TIMEOUT_SEC:-20.0}"
BASE_PARAMS_FILE="${ARACHNE_APRILTAG_NAV_BASE_PARAMS_FILE:-src/arachne_nav/config/nav2_params.yaml}"
OUTPUT_PARAMS_FILE="${ARACHNE_APRILTAG_NAV_OUTPUT_PARAMS_FILE:-/tmp/arachne_nav_apriltag_params.yaml}"
OUTPUT_POSE_FILE="${ARACHNE_APRILTAG_NAV_OUTPUT_POSE_FILE:-/tmp/arachne_nav_apriltag_pose.json}"

cd "${ROOT_DIR}"

echo "AprilTag nav initialization:"
echo "  family=${TAG_FAMILY}"
echo "  tag_id=${TAG_ID}"
echo "  tag_size_m=${TAG_SIZE_M}"
echo "  board_image=${BOARD_IMAGE_PATH}"
echo "  tag_map_xyz=${TAG_MAP_XYZ}"
echo "  tag_map_rpy=${TAG_MAP_RPY}"
echo "  output_params=${OUTPUT_PARAMS_FILE}"

ROS_ARGS=(
  -p once:=true
  -p image_topic:="${IMAGE_TOPIC}"
  -p camera_info_topic:="${CAMERA_INFO_TOPIC}"
  -p base_frame:="${BASE_FRAME}"
  -p odom_frame:="${ODOM_FRAME}"
  -p tag_family:="${TAG_FAMILY}"
  -p tag_id:="${TAG_ID}"
  -p tag_size_m:="${TAG_SIZE_M}"
  -p board_image_path:="${BOARD_IMAGE_PATH}"
  -p board_width_m:="${BOARD_WIDTH_M}"
  -p board_height_m:="${BOARD_HEIGHT_M}"
  -p tag_map_xyz:="${TAG_MAP_XYZ}"
  -p tag_map_rpy:="${TAG_MAP_RPY}"
  -p timeout_sec:="${TIMEOUT_SEC}"
  -p base_params_file:="${BASE_PARAMS_FILE}"
  -p output_params_file:="${OUTPUT_PARAMS_FILE}"
  -p output_pose_file:="${OUTPUT_POSE_FILE}"
)
if [[ -n "${CAMERA_FRAME}" ]]; then
  ROS_ARGS+=(-p camera_frame:="${CAMERA_FRAME}")
fi

ros2 run arachne_operator apriltag_nav_initializer --ros-args "${ROS_ARGS[@]}"

echo "AprilTag nav pose:"
cat "${OUTPUT_POSE_FILE}"

if [[ "${ARACHNE_APRILTAG_NAV_START_ONLY:-false}" == "true" ]]; then
  exit 0
fi

exec env \
  ARACHNE_NAV_MODE=mapping \
  ARACHNE_NAV_PARAMS_FILE="${OUTPUT_PARAMS_FILE}" \
  "${ROOT_DIR}/scripts/hardware/real_lidar_nav.sh" "$@"
