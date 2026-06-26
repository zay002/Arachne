#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

GRACE_SEC="${GRACE_SEC:-2}"

usage() {
  cat <<EOF
Usage: ./scripts/hardware/stop_real_stack.sh [--help]

Stops known Arachne real-stack processes with INT, TERM, then KILL fallback.

Environment:
  GRACE_SEC=${GRACE_SEC}

This command is intended as a recovery/cleanup helper. It does not start or move
hardware, but it can stop drivers, task servers, RViz, camera nodes, and panels.
EOF
}

case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac

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
  "ros2 launch arachne_sensors gemini335.launch.py"
  "/install/arachne_sensors/lib/arachne_sensors/gemini335_v4l2_node"
  "ros2 run arachne_sensors depth_to_pointcloud"
  "/install/arachne_sensors/lib/arachne_sensors/depth_to_pointcloud"
  "[_]_node:=arachne_depth_to_pointcloud"
  "[_]_node:=gemini335_color_tf"
  "[_]_node:=gemini335_depth_tf"
  "[_]_node:=arachne_calibrated_color_tf"
  "[_]_node:=arachne_calibrated_depth_tf"
  "static_transform_publisher.*camera_color_optical_frame"
  "static_transform_publisher.*camera_depth_optical_frame"
  "/install/step_motor/lib/step_motor/motor_node"
  "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"
  "ros2 launch arachne_hardware real_bringup.launch.py"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_move_joint_action_server"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_official_status_probe"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_teach_command_bridge"
  "/install/arachne_hardware/lib/arachne_hardware/aubo_sdk_velocity_bridge"
  "ros2 launch arachne_operator teach_panel.launch.py"
  "/install/arachne_operator/lib/arachne_operator/teach_panel"
  "rviz2.*arachne_lidar_fusion.rviz"
  "rviz2.*arachne_nav_topdown.rviz"
  "rviz2.*arachne_model.rviz"
  "ros2 launch arachne_operator grasp_task_server.launch.py"
  "/install/arachne_operator/lib/arachne_operator/grasp_task_server"
  "ros2 launch arachne_operator road_cleanup_task_server.launch.py"
  "/install/arachne_operator/lib/arachne_operator/road_cleanup_task_server"
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

sleep 0.5

for pid in "${PIDS[@]}"; do
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
done

for pid in "${PIDS[@]}"; do
  wait "${pid}" 2>/dev/null || true
done

echo "Existing Arachne real stack stopped."
