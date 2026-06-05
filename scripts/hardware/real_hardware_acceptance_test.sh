#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

arachne_source_ros_setup
arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Build arachne_operator first."

CONFIRM_MOTION=false
if [[ "${ARACHNE_CONFIRM_REAL_MOTION:-}" == "YES" ]]; then
  CONFIRM_MOTION=true
fi

if [[ "${CONFIRM_MOTION}" != "true" ]]; then
  cat <<'EOF'
Dry run only. This script will not command real hardware unless explicitly confirmed.

Before real motion:
  1. Run ./scripts/hardware/check_real_hardware_env.sh --strict
  2. Bring up the hardware drivers in another terminal.
  3. Keep an emergency stop / power cut within reach.
  4. Run with ARACHNE_CONFIRM_REAL_MOTION=YES.

Example:
  ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
EOF
fi

exec ros2 launch arachne_operator real_hardware_acceptance_test.launch.py \
  confirm_motion:="${CONFIRM_MOTION}" "$@"
