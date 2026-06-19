#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
  cat <<'EOF'
Usage: ./scripts/vision/grasp_task_server.sh [grasp_task_server.launch.py args...]

Primary visual-grasp task server entrypoint. Loads the Arachne environment and
launches arachne_operator grasp_task_server.launch.py.

Common launch arguments:
  execute_real:=false|true
  confirm_execute_real:=false|true
  with_rviz:=false|true
  require_camera_topics:=false|true
  require_odom:=false|true

Safety:
  Real execution must remain explicitly guarded. Use execute_real:=true together
  with confirm_execute_real:=true only after real hardware is ready.
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
exec ros2 launch arachne_operator grasp_task_server.launch.py "$@"
