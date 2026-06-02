#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/ros_env.sh"

if ROS_DISTRO="$(arachne_detect_ros_distro 2>/dev/null)"; then
  export ROS_DISTRO
  arachne_source_bash_file "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  arachne_source_bash_file "${ROOT_DIR}/install/setup.bash"
fi

test_profile() {
  local profile="$1"
  local expected_closed="$2"
  local expected_open="$3"
  local expected_closed_secondary="${4:-}"
  local closed_file="/tmp/arachne_${profile}_closed_joint_state.txt"
  local open_file="/tmp/arachne_${profile}_open_joint_state.txt"

  run_controller() {
    local ns="$1"
    local log_file="$2"
    ros2 run arachne_gripper gripper_sim_controller --ros-args \
      -p profile:="${profile}" \
      -p joint_state_topic:="${ns}/joint_states" \
      -p command_topic:="${ns}/command" \
      -p position_topic:="${ns}/position" \
      -p open_service:="${ns}/open" \
      -p close_service:="${ns}/close" \
      -p stop_service:="${ns}/stop" >"${log_file}" 2>&1 &
    controller_pid="$!"
  }

  call_service() {
    local service_name="$1"
    SERVICE_NAME="${service_name}" /usr/bin/python3 - <<'PY'
import os
import time

import rclpy
from std_srvs.srv import Trigger


service_name = os.environ["SERVICE_NAME"]
rclpy.init()
node = rclpy.create_node("arachne_gripper_test_client")
client = node.create_client(Trigger, service_name)

deadline = time.monotonic() + 8.0
while not client.wait_for_service(timeout_sec=0.2):
    if time.monotonic() > deadline:
        node.destroy_node()
        rclpy.shutdown()
        raise TimeoutError(f"Timed out waiting for {service_name}")

future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
response = future.result() if future.done() else None
node.destroy_node()
rclpy.shutdown()

if response is None:
    raise TimeoutError(f"Timed out calling {service_name}")
if not response.success:
    raise RuntimeError(response.message)
PY
  }

  cleanup_pid() {
    local pid="$1"
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  }

  local close_ns="/arachne/test_${profile}_close_$$"
  local close_pid
  run_controller "${close_ns}" "/tmp/arachne_${profile}_close_gripper_sim.log"
  close_pid="${controller_pid}"
  sleep 2
  call_service "${close_ns}/close"
  sleep 1.2
  timeout 5s ros2 topic echo "${close_ns}/joint_states" sensor_msgs/msg/JointState --once --no-daemon >"${closed_file}"
  cleanup_pid "${close_pid}"

  local open_ns="/arachne/test_${profile}_open_$$"
  local open_pid
  run_controller "${open_ns}" "/tmp/arachne_${profile}_open_gripper_sim.log"
  open_pid="${controller_pid}"
  sleep 2
  call_service "${open_ns}/close"
  sleep 1.2
  call_service "${open_ns}/open"
  sleep 1.2
  timeout 5s ros2 topic echo "${open_ns}/joint_states" sensor_msgs/msg/JointState --once --no-daemon >"${open_file}"
  cleanup_pid "${open_pid}"

  if ! grep -q -- "- ${expected_closed}" "${closed_file}"; then
    echo "Expected closed position ${expected_closed} in ${closed_file}" >&2
    exit 1
  fi

  if [[ -n "${expected_closed_secondary}" ]] && ! grep -q -- "- ${expected_closed_secondary}" "${closed_file}"; then
    echo "Expected secondary closed position ${expected_closed_secondary} in ${closed_file}" >&2
    exit 1
  fi

  if ! grep -q -- "- ${expected_open}" "${open_file}"; then
    echo "Expected open position ${expected_open} in ${open_file}" >&2
    exit 1
  fi

  echo "Validated ${profile} gripper simulation profile"
  echo "  closed: ${closed_file}"
  echo "  open:   ${open_file}"
}

test_profile ms42dc 0.6 0.0 "-0.6"
test_profile ag95 0.93 0.0
