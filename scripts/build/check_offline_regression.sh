#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

echo "[check_offline_regression] offline only: no real hardware will be contacted"

echo
echo "== Python compile =="
python3 -m compileall \
  src/arachne_hardware/arachne_hardware \
  src/arachne_operator/arachne_operator \
  scripts/vision

echo
echo "== Shell syntax =="
bash -n scripts/build/check_aubo_action_stack.sh
bash -n scripts/hardware/check_aubo_readonly.sh
bash -n scripts/operator/teach_panel.sh
bash -n scripts/hardware/real_bringup.sh
bash -n scripts/hardware/real_teach_demo.sh
bash -n scripts/vision/grasp_task_server.sh
bash -n scripts/vision/road_cleanup_task_server.sh
bash -n scripts/vision/grasp_preview_real_sync.sh

echo
echo "== Workspace contract =="
./scripts/build/check_workspace.sh

if command -v ros2 >/dev/null 2>&1 && command -v colcon >/dev/null 2>&1; then
  echo
  echo "== Selected ROS package build =="
  set +u
  # shellcheck disable=SC1091
  source "$ROOT_DIR/scripts/env/arachne_env.sh"
  set -u
  colcon build --base-paths src --packages-select arachne_hardware arachne_operator
else
  echo
  echo "[check_offline_regression] ros2/colcon not found; skipped selected package build"
fi

echo
echo "[check_offline_regression] passed"
