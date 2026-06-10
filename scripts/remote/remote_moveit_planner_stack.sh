#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ACTION="${1:-start}"
PORT="${ARACHNE_REMOTE_MOVEIT_PORT:-8766}"
LOG_DIR="${ARACHNE_REMOTE_MOVEIT_LOG_DIR:-${ROOT_DIR}/log/remote_moveit_planner}"
PID_DIR="${ARACHNE_REMOTE_MOVEIT_PID_DIR:-${LOG_DIR}}"

mkdir -p "${LOG_DIR}" "${PID_DIR}"

source_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
  fi
  set -u
}

stop_stack() {
  for name in bridge moveit; do
    pid_file="${PID_DIR}/${name}.pid"
    if [[ -f "${pid_file}" ]]; then
      kill "$(cat "${pid_file}")" >/dev/null 2>&1 || true
      rm -f "${pid_file}"
    fi
  done
  while read -r pid; do
    [[ -n "${pid}" ]] && kill "${pid}" >/dev/null 2>&1 || true
  done < <(pgrep -u "$(id -u)" -f "remote_moveit_planner_server.py" || true)
  pkill -f "[r]os2 launch arachne_moveit_config moveit_planning.launch.py.*remote_joint_states" >/dev/null 2>&1 || true
  sleep 0.5
  while read -r pid; do
    [[ -n "${pid}" ]] && kill -KILL "${pid}" >/dev/null 2>&1 || true
  done < <(pgrep -u "$(id -u)" -f "remote_moveit_planner_server.py" || true)
}

case "${ACTION}" in
  start)
    source_env
    stop_stack
    ros2 launch arachne_moveit_config moveit_planning.launch.py \
      launch_rviz:=false \
      with_robot_state_publisher:=true \
      joint_states_topic:=/arachne/remote_joint_states \
      >"${LOG_DIR}/moveit.log" 2>&1 &
    echo "$!" >"${PID_DIR}/moveit.pid"
    sleep 2
    python3 "${ROOT_DIR}/scripts/remote/remote_moveit_planner_server.py" \
      --host 127.0.0.1 \
      --port "${PORT}" \
      >"${LOG_DIR}/bridge.log" 2>&1 &
    echo "$!" >"${PID_DIR}/bridge.pid"
    sleep 1
    "${BASH_SOURCE[0]}" status
    ;;
  stop)
    stop_stack
    ;;
  restart)
    "${BASH_SOURCE[0]}" stop
    "${BASH_SOURCE[0]}" start
    ;;
  status)
    source_env
    echo "remote MoveIt planner stack"
    for name in moveit bridge; do
      pid_file="${PID_DIR}/${name}.pid"
      if [[ -f "${pid_file}" ]]; then
        ps -p "$(cat "${pid_file}")" -o pid,cmd || true
      else
        echo "${name}: no pid file"
      fi
    done
    python3 "${ROOT_DIR}/scripts/remote/remote_planner_client.py" \
      --url "http://127.0.0.1:${PORT}" \
      --health \
      --timeout 3 || true
    ;;
  *)
    echo "Usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
