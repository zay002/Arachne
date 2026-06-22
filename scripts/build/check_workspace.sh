#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

cd "${ROOT_DIR}"

if [[ -z "${MAKEFLAGS:-}" && "$(uname -m)" == "aarch64" ]]; then
  export MAKEFLAGS="-j2"
fi

COLCON_ARGS=()
if [[ -n "${ARACHNE_COLCON_PARALLEL_WORKERS:-}" ]]; then
  COLCON_ARGS+=(--parallel-workers "${ARACHNE_COLCON_PARALLEL_WORKERS}")
elif [[ "$(uname -m)" == "aarch64" ]]; then
  COLCON_ARGS+=(--parallel-workers 2)
fi

echo "== Python syntax =="
/usr/bin/python3 -m py_compile \
  src/arachne_demo/arachne_demo/*.py \
  src/arachne_agent_bridge/arachne_agent_bridge/*.py \
  src/arachne_gripper/arachne_gripper/*.py \
  src/arachne_hardware/arachne_hardware/*.py \
  src/arachne_operator/arachne_operator/*.py \
  src/arachne_sensors/arachne_sensors/*.py \
  src/arachne_sim/arachne_sim/*.py

echo "== Build local packages =="
colcon build "${COLCON_ARGS[@]}" --base-paths src --packages-select \
  aubo_description aubo_msgs aubo_dashboard_msgs aubo_ros2_driver \
  scout_description dh_ag95_description \
  arachne_description arachne_sim arachne_gripper arachne_hardware \
  arachne_control arachne_moveit_config arachne_nav arachne_operator arachne_sensors \
  arachne_agent_bridge \
  --cmake-args -DPython3_EXECUTABLE="${ARACHNE_SYSTEM_PYTHON}"

set +u
source install/setup.bash
set -u

echo "== Optional runtime package hints =="
for package in moveit_ros_move_group nav2_bringup controller_manager; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "[OK] ${package}"
  else
    echo "[WARN] ${package} not found; run ./scripts/build/setup_ubuntu.sh before launching that stack"
  fi
done

echo "== Xacro generation =="
for gripper in ms42dc ag95; do
  ros2 run xacro xacro src/arachne_description/urdf/arachne.urdf.xacro \
    gripper_type:="${gripper}" >/tmp/arachne_${gripper}.urdf
  ros2 run xacro xacro src/arachne_description/urdf/arachne.urdf.xacro \
    gripper_type:="${gripper}" with_ros2_control:=true with_mimic_joints:=false \
    >/tmp/arachne_${gripper}_control.urdf
  ros2 run xacro xacro src/arachne_moveit_config/config/arachne_${gripper}.srdf.xacro \
    >/tmp/arachne_${gripper}.srdf
done

echo "== Launch contract smoke test =="
timeout 5s ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false use_ms42dc:=false use_aubo:=false

echo "Arachne workspace checks passed."
