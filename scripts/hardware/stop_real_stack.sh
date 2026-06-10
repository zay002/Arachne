#!/usr/bin/env bash
set -euo pipefail

GRACE_SEC="${GRACE_SEC:-2}"

PATTERNS=(
  "/install/aubo_ros2_driver/lib/aubo_ros2_driver/aubo_ros2_control_node"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_official_status_probe"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_teach_command_bridge"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_sdk_velocity_bridge"
  "/install/arachne_hardware/lib/arachne_hardware/ms42dc_direct_serial_driver"
  "/install/arachne_hardware/lib/arachne_hardware/ms42dc_official_bridge"
  "/install/arachne_hardware/lib/arachne_hardware/scout_waveshare_serial_driver"
  "/install/arachne_hardware/lib/arachne_hardware/scout_official_status_bridge"
  "/install/arachne_sim/lib/arachne_sim/base_sim_controller"
  "base_sim_controller"
  "/install/step_motor/lib/step_motor/motor_node"
  "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"
  "ros2 launch arachne_hardware real_bringup.launch.py"
  "ros2 launch arachne_operator teach_panel.launch.py"
  "ros2 launch arachne_operator grasp_task_server.launch.py"
  "/install/arachne_operator/lib/arachne_operator/grasp_task_server"
  "/scripts/hardware/real_grasp_console.sh"
  "/scripts/vision/grasp_preview.sh"
  "grasp_preview_pipeline.py"
  "raw_image_viewer.py"
  "image_view.*arachne/grasp_preview/annotated_image"
  "image_view.*camera/color/image_raw"
)

collect_pids() {
  local pattern pid
  for pattern in "${PATTERNS[@]}"; do
    pgrep -f "${pattern}" || true
  done | sort -n -u | while read -r pid; do
    if [[ -n "${pid}" && "${pid}" != "$$" && "${pid}" != "${PPID}" ]]; then
      printf '%s\n' "${pid}"
    fi
  done
}

mapfile -t PIDS < <(collect_pids)

if ((${#PIDS[@]} == 0)); then
  echo "No existing Arachne real stack processes found."
  exit 0
fi

echo "Stopping existing Arachne real stack processes: ${PIDS[*]}"
for pid in "${PIDS[@]}"; do
  kill -INT "${pid}" 2>/dev/null || true
done

sleep "${GRACE_SEC}"

for pid in "${PIDS[@]}"; do
  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
done

for pid in "${PIDS[@]}"; do
  wait "${pid}" 2>/dev/null || true
done

echo "Existing Arachne real stack stopped."
