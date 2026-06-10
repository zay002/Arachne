#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/load_local_env.sh"
arachne_load_local_env "${ROOT_DIR}"
set -u

ACTION="start"
if (($#)); then
  case "$1" in
    start|restart|status|stop|-h|--help|help)
      ACTION="$1"
      shift
      ;;
  esac
fi

REMOTE_HOST="${ARACHNE_REMOTE_HOST:-}"
REMOTE_USER="${ARACHNE_REMOTE_USER:-}"
REMOTE_DIR="${ARACHNE_REMOTE_FULL_DIR:-${ARACHNE_REMOTE_DIR:-~/projects/arachne_remote_full}}"
REMOTE_MOVEIT_PORT="${ARACHNE_REMOTE_MOVEIT_PORT:-8766}"
LOCAL_PORT="${ARACHNE_REMOTE_LOCAL_PORT:-8767}"
REMOTE_URL="${ARACHNE_CONSOLE_REMOTE_PLANNER_URL:-${ARACHNE_REMOTE_PLANNER_URL:-http://127.0.0.1:${LOCAL_PORT}}}"
REMOTE_TIMEOUT="${ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT:-${ARACHNE_REMOTE_PLANNER_TIMEOUT:-20}}"
SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
SSH_CONTROL_PATH="${ARACHNE_REMOTE_SSH_CONTROL_PATH:-/tmp/arachne_remote_moveit_${USER}_${LOCAL_PORT}.sock}"
SSH_OPTS=(
  -o StrictHostKeyChecking=accept-new
  -o ExitOnForwardFailure=yes
  -S "${SSH_CONTROL_PATH}"
)

usage() {
  cat <<EOF
Usage:
  ./scripts/hardware/real_grasp_console_remote.sh [start|restart|status|stop] [console options]

Default action is start. It reads .env.local, starts the remote MoveIt planner
stack, opens a local SSH tunnel, then launches real_grasp_console.sh with
remote planning enabled.

Required in .env.local:
  ARACHNE_REMOTE_HOST=...
  ARACHNE_REMOTE_USER=...

Common optional values:
  ARACHNE_REMOTE_FULL_DIR=~/projects/arachne_remote_full
  ARACHNE_REMOTE_MOVEIT_PORT=8766
  ARACHNE_REMOTE_LOCAL_PORT=8767
  ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT=20
EOF
}

require_remote_config() {
  if [[ -z "${REMOTE_HOST}" || -z "${REMOTE_USER}" ]]; then
    cat >&2 <<EOF
Missing remote planner config. Create ${ROOT_DIR}/.env.local with:

ARACHNE_REMOTE_HOST=<server-host>
ARACHNE_REMOTE_USER=<server-user>
ARACHNE_REMOTE_FULL_DIR=~/projects/arachne_remote_full
ARACHNE_REMOTE_LOCAL_PORT=8767
ARACHNE_REMOTE_MOVEIT_PORT=8766
ARACHNE_USE_REMOTE_PLANNER_DEFAULT=true
ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT=20
EOF
    exit 2
  fi
}

ssh_check_master() {
  ssh "${SSH_OPTS[@]}" -O check "${SSH_TARGET}" >/dev/null 2>&1
}

start_tunnel() {
  if ssh_check_master; then
    echo "Remote planner SSH tunnel already running on local port ${LOCAL_PORT}."
    return 0
  fi
  rm -f "${SSH_CONTROL_PATH}"
  echo "Opening SSH tunnel: 127.0.0.1:${LOCAL_PORT} -> remote 127.0.0.1:${REMOTE_MOVEIT_PORT}"
  ssh -M -fN \
    "${SSH_OPTS[@]}" \
    -L "${LOCAL_PORT}:127.0.0.1:${REMOTE_MOVEIT_PORT}" \
    "${SSH_TARGET}"
}

stop_tunnel() {
  if ssh_check_master; then
    ssh "${SSH_OPTS[@]}" -O exit "${SSH_TARGET}" >/dev/null 2>&1 || true
    echo "Stopped remote planner SSH tunnel."
  fi
  rm -f "${SSH_CONTROL_PATH}"
}

remote_stack() {
  local stack_action="$1"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
    "cd ${REMOTE_DIR} && source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/remote/remote_moveit_planner_stack.sh ${stack_action}"
}

check_health() {
  "${ROOT_DIR}/scripts/remote/remote_planner_client.py" \
    --url "${REMOTE_URL}" \
    --health \
    --timeout 5
}

start_all() {
  require_remote_config
  start_tunnel
  remote_stack restart
  check_health
  export ARACHNE_USE_REMOTE_PLANNER_DEFAULT=true
  export ARACHNE_CONSOLE_REMOTE_PLANNER_URL="${REMOTE_URL}"
  export ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT="${REMOTE_TIMEOUT}"
  "${ROOT_DIR}/scripts/hardware/real_grasp_console.sh" --yes --quick "$@"
}

case "${ACTION}" in
  start)
    start_all "$@"
    ;;
  restart)
    require_remote_config
    stop_tunnel
    start_all "$@"
    ;;
  status)
    require_remote_config
    start_tunnel
    remote_stack status
    check_health
    ;;
  stop)
    require_remote_config
    remote_stack stop || true
    stop_tunnel
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    start_all "$@"
    ;;
esac
