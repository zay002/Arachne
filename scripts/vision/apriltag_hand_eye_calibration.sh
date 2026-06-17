#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export ARACHNE_ENV_NO_WORKSPACE=0
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
hash -r
PYTHON_USER_SITE="$("${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}" - <<'PY'
import site
print(site.getusersitepackages())
PY
)"
if [[ -d "${PYTHON_USER_SITE}" ]]; then
  export PYTHONPATH="${PYTHON_USER_SITE}:${PYTHONPATH:-}"
fi

IMAGE_TOPIC="${ARACHNE_HAND_EYE_IMAGE_TOPIC:-/camera/color/image_raw}"
CAMERA_INFO_TOPIC="${ARACHNE_HAND_EYE_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
BASE_FRAME="${ARACHNE_HAND_EYE_BASE_FRAME:-base_link}"
GRIPPER_FRAME="${ARACHNE_HAND_EYE_GRIPPER_FRAME:-tool0}"
BOARD_IMAGE_PATH="${ARACHNE_HAND_EYE_BOARD_IMAGE_PATH:-/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png}"
BOARD_WIDTH_M="${ARACHNE_HAND_EYE_BOARD_WIDTH_M:-0.420}"
BOARD_HEIGHT_M="${ARACHNE_HAND_EYE_BOARD_HEIGHT_M:-0.297}"
MIN_BOARD_MATCHES="${ARACHNE_HAND_EYE_MIN_BOARD_MATCHES:-24}"
TAG_FAMILY="${ARACHNE_HAND_EYE_TAG_FAMILY:-tagStandard41h12}"
TAG_SIZE_M="${ARACHNE_HAND_EYE_TAG_SIZE_M:-0.070}"
TAG_PITCH_M="${ARACHNE_HAND_EYE_TAG_PITCH_M:-0.100}"
TAG_ID="${ARACHNE_HAND_EYE_TAG_ID:--1}"
TAG_DICTIONARY="${ARACHNE_HAND_EYE_TAG_DICTIONARY:-DICT_APRILTAG_36h11}"
ENABLE_BOARD_TEMPLATE_FALLBACK="${ARACHNE_HAND_EYE_ENABLE_BOARD_TEMPLATE_FALLBACK:-false}"
OUTPUT_DIR="${ARACHNE_HAND_EYE_OUTPUT_DIR:-log/calibration/hand_eye}"

cd "${ROOT_DIR}"

ros2 run arachne_operator apriltag_hand_eye_calibrator --ros-args \
  -p image_topic:="${IMAGE_TOPIC}" \
  -p camera_info_topic:="${CAMERA_INFO_TOPIC}" \
  -p base_frame:="${BASE_FRAME}" \
  -p gripper_frame:="${GRIPPER_FRAME}" \
  -p board_image_path:="${BOARD_IMAGE_PATH}" \
  -p board_width_m:="${BOARD_WIDTH_M}" \
  -p board_height_m:="${BOARD_HEIGHT_M}" \
  -p min_board_matches:="${MIN_BOARD_MATCHES}" \
  -p tag_family:="${TAG_FAMILY}" \
  -p tag_size_m:="${TAG_SIZE_M}" \
  -p tag_pitch_m:="${TAG_PITCH_M}" \
  -p tag_id:="${TAG_ID}" \
  -p dictionary:="${TAG_DICTIONARY}" \
  -p enable_board_template_fallback:="${ENABLE_BOARD_TEMPLATE_FALLBACK}" \
  -p output_dir:="${OUTPUT_DIR}" \
  "$@"
