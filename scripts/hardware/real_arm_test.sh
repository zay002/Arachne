#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "${ROOT_DIR}/scripts/hardware/real_hardware_acceptance_test.sh" \
  sequence_mode:=sequential \
  run_base_test:=false \
  run_arm_test:=true \
  run_gripper_test:=false \
  "$@"
