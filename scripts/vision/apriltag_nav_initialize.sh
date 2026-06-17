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
set -u

cd "${ROOT_DIR}"

ARACHNE_APRILTAG_NAV_START_ONLY=true \
  "${ROOT_DIR}/scripts/vision/apriltag_nav_start_mapping.sh" --show-args
