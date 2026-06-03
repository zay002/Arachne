#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_Z_DELTA_M="${AUBO_Z_DELTA_M:-0.01}"
AUBO_ARM_DURATION_SEC="${AUBO_ARM_DURATION_SEC:-8.0}"
AUBO_ARM_MAX_JOINT_DELTA="${AUBO_ARM_MAX_JOINT_DELTA:-0.12}"
AUBO_REAL_JOINTS="shoulder_joint,upperArm_joint,foreArm_joint,wrist1_joint,wrist2_joint,wrist3_joint"

cat <<EOF
Aubo real-arm Z test
  delta: ${AUBO_Z_DELTA_M} m in aubo_base_link Z
  duration: ${AUBO_ARM_DURATION_SEC} s
  max joint delta guard: ${AUBO_ARM_MAX_JOINT_DELTA} rad

This script expects the real Aubo driver to be running in another terminal:
  ARACHNE_CONFIRM_AUBO_DRIVER=YES ./scripts/real_aubo_bringup.sh

Dry run is the default. Real motion requires:
  ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_aubo_z_test.sh
EOF

if [[ "${ARACHNE_CONFIRM_REAL_MOTION:-}" == "YES" ]]; then
  "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"
fi

exec "${ROOT_DIR}/scripts/real_hardware_acceptance_test.sh" \
  run_base_test:=false \
  run_arm_test:=true \
  run_gripper_test:=false \
  joint_states_topic:=/joint_states \
  arm_command_mode:=action \
  arm_follow_joint_trajectory_action:=/joint_trajectory_controller/follow_joint_trajectory \
  arm_trajectory_topic:=/joint_trajectory_controller/joint_trajectory \
  legacy_arm_trajectory_topic:=/joint_trajectory_controller/joint_trajectory \
  arm_state_joint_names:="${AUBO_REAL_JOINTS}" \
  arm_command_joint_names:="${AUBO_REAL_JOINTS}" \
  arm_z_delta_m:="${AUBO_Z_DELTA_M}" \
  arm_z_frame:=aubo_base \
  arm_duration_sec:="${AUBO_ARM_DURATION_SEC}" \
  arm_max_joint_delta:="${AUBO_ARM_MAX_JOINT_DELTA}" \
  "$@"
