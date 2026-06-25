#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
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

NAME="${1:-arachne_map_$(date +%Y%m%d_%H%M%S)}"
if [[ "${NAME}" = /* ]]; then
  OUTPUT="${NAME}"
elif [[ "${NAME}" == */* ]]; then
  OUTPUT="${ROOT_DIR}/${NAME}"
else
  OUTPUT="${ROOT_DIR}/src/arachne_nav/maps/${NAME}"
fi
OUTPUT="${OUTPUT%.yaml}"
mkdir -p "$(dirname "${OUTPUT}")"

echo "Saving current /map to:"
echo "  ${OUTPUT}.yaml"
echo "  ${OUTPUT}.pgm"

exec ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "${OUTPUT}" \
  --fmt pgm \
  --mode trinary \
  --ros-args \
  -p map_subscribe_transient_local:=true
