#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/model/use_gripper.sh <ms42dc|ag95> [mode] [extra launch args...]

Modes:
  view              RViz model view with base, arm, and gripper GUI (default)
  gazebo            Switch/Gazebo playable demo
  switch            Alias of gazebo
  rviz-demo         Switch/RViz lightweight demo
  prehardware       Mock Nav2 + MoveIt2 + operator + action translator
  moveit            MoveIt2 starter launch
  ros2-control      Mock ros2_control launch
  nav2              Nav2 starter launch
  translator        VLA/WAM action chunk translator only
  check             Generate the selected URDF as a quick model check

Examples:
  ./scripts/model/use_gripper.sh ms42dc
  ./scripts/model/use_gripper.sh ag95 view
  ./scripts/model/use_gripper.sh ms42dc prehardware launch_rviz:=false
  ./scripts/model/use_gripper.sh ag95 translator input_topic:=/external/action_chunk
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

GRIPPER_TYPE="$1"
shift
MODE="${1:-view}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${GRIPPER_TYPE}" in
  ms42dc|ag95) ;;
  *)
    echo "Unsupported gripper: ${GRIPPER_TYPE}. Expected ms42dc or ag95." >&2
    exit 1
    ;;
esac

source_ros() {
  arachne_source_ros_setup
  arachne_source_workspace_setup \
    "${ROOT_DIR}" \
    "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first."
}

export GRIPPER_TYPE
export GRIPPER_SIM_PROFILE="${GRIPPER_SIM_PROFILE:-${GRIPPER_TYPE}}"

case "${MODE}" in
  view|model|rviz)
    exec "${ROOT_DIR}/scripts/model/view_model.sh" "$@"
    ;;
  gazebo|switch|demo)
    export DEMO_MODE="gazebo"
    exec "${ROOT_DIR}/scripts/sim/switch_demo.sh" "$@"
    ;;
  rviz-demo|switch-rviz)
    export DEMO_MODE="rviz"
    exec "${ROOT_DIR}/scripts/sim/switch_demo.sh" "$@"
    ;;
  prehardware)
    source_ros
    exec ros2 launch arachne_control prehardware_control.launch.py \
      gripper_type:="${GRIPPER_TYPE}" "$@"
    ;;
  moveit)
    source_ros
    exec ros2 launch arachne_moveit_config moveit_planning.launch.py \
      gripper_type:="${GRIPPER_TYPE}" "$@"
    ;;
  ros2-control|ros2_control|control)
    source_ros
    exec ros2 launch arachne_control mock_ros2_control.launch.py \
      gripper_type:="${GRIPPER_TYPE}" "$@"
    ;;
  nav2)
    source_ros
    exec ros2 launch arachne_nav nav2_sim.launch.py \
      gripper_type:="${GRIPPER_TYPE}" "$@"
    ;;
  translator|vla|wam)
    source_ros
    exec ros2 launch arachne_operator action_chunk_translator.launch.py "$@"
    ;;
  check)
    source_ros
    output="/tmp/arachne_${GRIPPER_TYPE}.urdf"
    ros2 run xacro xacro "${ROOT_DIR}/src/arachne_description/urdf/arachne.urdf.xacro" \
      gripper_type:="${GRIPPER_TYPE}" >"${output}"
    echo "Generated ${output}"
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    usage >&2
    exit 1
    ;;
esac
