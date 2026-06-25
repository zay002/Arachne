#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
export ARACHNE_ENV_QUIET=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
unset ARACHNE_ENV_QUIET
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

TOPICS="$(timeout 3 ros2 topic list --no-daemon 2>/dev/null || true)"
SERVICES="$(timeout 3 ros2 service list --no-daemon 2>/dev/null || true)"

topic_status() {
  local topic="$1"
  local label="$2"
  if grep -qx "${topic}" <<<"${TOPICS}"; then
    printf '[OK]   %-28s %s\n' "${label}" "${topic}"
  else
    printf '[MISS] %-28s %s\n' "${label}" "${topic}"
  fi
}

service_status() {
  local service="$1"
  local label="$2"
  if grep -qx "${service}" <<<"${SERVICES}"; then
    printf '[OK]   %-28s %s\n' "${label}" "${service}"
  else
    printf '[MISS] %-28s %s\n' "${label}" "${service}"
  fi
}

echo "Arachne real grasp status"
echo "workspace: ${ROOT_DIR}"
echo

service_status "/arachne/grasp_task/start" "grasp start service"
service_status "/arachne/grasp_task/stop" "grasp stop service"
service_status "/arachne/grasp_task/restore" "grasp restore service"
echo

topic_status "/joint_states" "Aubo joint states"
topic_status "/arachne/hardware/gripper_status" "MS42DC gripper status"
topic_status "/odom" "Scout odometry"
topic_status "/camera/color/image_raw" "Gemini color image"
topic_status "/camera/depth/image_raw" "Gemini depth image"
topic_status "/arachne/grasp_task/state" "grasp task state"
topic_status "/arachne/grasp_task/event" "grasp task event"
echo

if grep -qx "/arachne/grasp_task/status" <<<"${SERVICES}"; then
  echo "grasp task snapshot:"
  timeout 4 ros2 service call /arachne/grasp_task/status std_srvs/srv/Trigger "{}" 2>/dev/null || true
else
  echo "grasp task snapshot: service unavailable"
fi
echo

if [[ -L "${ROOT_DIR}/log/real_grasp_console/latest" ]]; then
  echo "console log dir: $(readlink -f "${ROOT_DIR}/log/real_grasp_console/latest")"
fi
if [[ -d "${ROOT_DIR}/log/grasp_tasks" ]]; then
  latest_task="$(find "${ROOT_DIR}/log/grasp_tasks" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2- || true)"
  if [[ -n "${latest_task}" ]]; then
    echo "latest task dir: ${latest_task}"
  fi
fi
