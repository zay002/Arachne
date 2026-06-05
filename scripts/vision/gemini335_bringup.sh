#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"

arachne_require_ros_distro

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Build arachne_sensors first."

exec ros2 launch arachne_sensors gemini335.launch.py "$@"
