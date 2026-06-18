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
set -u
hash -r

IMAGE_TOPIC="${ARACHNE_HAND_EYE_IMAGE_TOPIC:-/camera/color/image_raw}"
CALIBRATOR="${ARACHNE_HAND_EYE_CALIBRATOR_NODE:-/arachne_apriltag_hand_eye_calibrator}"

cd "${ROOT_DIR}"
exec "${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}" \
  scripts/vision/apriltag_hand_eye_interactive.py \
  --image-topic "${IMAGE_TOPIC}" \
  --calibrator "${CALIBRATOR}" \
  "$@"
