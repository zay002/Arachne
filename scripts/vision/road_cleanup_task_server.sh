#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
  cat <<'EOF'
Usage: ./scripts/vision/road_cleanup_task_server.sh [road_cleanup_task_server.launch.py args...]

Primary road-cleanup task server entrypoint. Loads the Arachne environment and
launches arachne_operator road_cleanup_task_server.launch.py.

The road-cleanup server does not bypass grasping. It watches detection events,
commands base stop/recovery through the grasp task base interface, and calls the
grasp task server for camera-first target acquisition, point-cloud planning, arm
execution, and basket drop-off.

Common launch arguments:
  detection_topic:=/arachne/perception/taco_instances
  grasp_start_service:=/arachne/grasp_task/start
  base_command_topic:=/arachne/grasp_task/base_command
  reach_recovery_enabled:=true|false
EOF
}

case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac

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

cd "${ROOT_DIR}"
exec ros2 launch arachne_operator road_cleanup_task_server.launch.py "$@"
