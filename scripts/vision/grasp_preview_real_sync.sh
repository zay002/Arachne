#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARACHNE_ENV_NO_WORKSPACE=0
set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
if [[ -f "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh"
fi
set -u
hash -r

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
SYNC_SOURCE="${ARACHNE_GRASP_REAL_SYNC_SOURCE:-auto}"
REAL_JOINT_TOPIC="${ARACHNE_GRASP_REAL_JOINT_STATES_TOPIC:-/joint_states}"
SYNC_TIMEOUT="${ARACHNE_GRASP_REAL_SYNC_TIMEOUT:-3.0}"
RPC_TIMEOUT="${ARACHNE_GRASP_REAL_RPC_TIMEOUT:-2.0}"
CLEAN_STALE="${ARACHNE_GRASP_REAL_SYNC_CLEAN_STALE:-true}"
ALLOW_POWERED_OFF_RPC_POSE="${ARACHNE_GRASP_ALLOW_POWERED_OFF_RPC_POSE:-false}"
SYNC_ONLY=false
EXECUTE_REAL=false
PREVIEW_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/vision/grasp_preview_real_sync.sh [options] [-- grasp_preview_args...]

Synchronize the RViz grasp preview model from the real Aubo pose, then start
scripts/vision/grasp_preview.sh. By default this script reads robot state only;
--execute-real additionally arms the guarded real arm/gripper execution path.

Options:
  --sync-only       Print the synchronized pose and exit.
  --execute-real    Send the planned grasp trajectory to the real arm after planning.
  --no-clean        Do not clean stale preview-only display nodes first.
  -h, --help        Show this help.

Environment:
  AUBO_ROBOT_IP=192.168.127.128
  ARACHNE_GRASP_REAL_SYNC_SOURCE=auto|topic|rpc
  ARACHNE_GRASP_REAL_JOINT_STATES_TOPIC=/joint_states
  ARACHNE_GRASP_REAL_SYNC_TIMEOUT=3.0
  ARACHNE_GRASP_REAL_RPC_TIMEOUT=2.0
  ARACHNE_GRASP_ALLOW_POWERED_OFF_RPC_POSE=false
  ARACHNE_GRASP_REAL_EXECUTE_BACKEND=sdk_move_joint|follow_joint_trajectory
  ARACHNE_GRASP_REAL_RETURN_HOME=true
  ARACHNE_GRASP_REAL_HOME_JOINTS=$ARACHNE_AUBO_HOME_JOINTS_RAD
  ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES  # required with --execute-real

Examples:
  ./scripts/vision/grasp_preview_real_sync.sh --sync-only
  ./scripts/vision/grasp_preview_real_sync.sh
  ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES ./scripts/vision/grasp_preview_real_sync.sh --execute-real
  ./scripts/vision/grasp_preview_real_sync.sh -- --moveit-planning-time 3.0
EOF
}

while (($#)); do
  case "$1" in
    --sync-only)
      SYNC_ONLY=true
      ;;
    --execute-real)
      EXECUTE_REAL=true
      ;;
    --no-clean)
      CLEAN_STALE=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      PREVIEW_ARGS+=("$@")
      break
      ;;
    *)
      PREVIEW_ARGS+=("$1")
      ;;
  esac
  shift
done

cleanup_stale_preview_nodes() {
  pkill -f "[_]_node:=arachne_display_robot_state_publisher" >/dev/null 2>&1 || true
  pkill -f "[_]_node:=arachne_display_base_link_bridge" >/dev/null 2>&1 || true
  pkill -f "[j]oint_state_publisher .*arachne_display\\.urdf" >/dev/null 2>&1 || true
  pkill -f "[a]rachne_gripper/lib/arachne_gripper/joint_state_mux" >/dev/null 2>&1 || true
  pkill -f "[r]os2 launch arachne_moveit_config moveit_planning.launch.py" >/dev/null 2>&1 || true
  pkill -f "[m]oveit_ros_move_group/move_group.*joint_states:=/arachne/display/joint_states" >/dev/null 2>&1 || true
  pkill -f "[g]rasp_preview_pipeline.py" >/dev/null 2>&1 || true
}

cd "${ROOT_DIR}"

if [[ "${CLEAN_STALE}" == "true" ]]; then
  cleanup_stale_preview_nodes
  sleep 0.5
fi

ALLOW_POWERED_OFF_ARG=()
if [[ "${ALLOW_POWERED_OFF_RPC_POSE}" == "true" ]]; then
  ALLOW_POWERED_OFF_ARG=(--allow-powered-off-rpc-pose)
fi

if ! POSE_LINE="$(
  "${ARACHNE_SYSTEM_PYTHON}" - \
    --source "${SYNC_SOURCE}" \
    --joint-topic "${REAL_JOINT_TOPIC}" \
    --topic-timeout "${SYNC_TIMEOUT}" \
    --ip "${AUBO_ROBOT_IP}" \
    --rpc-timeout "${RPC_TIMEOUT}" \
    "${ALLOW_POWERED_OFF_ARG[@]}" <<'PY'
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from typing import Any


JOINT_ALIASES = (
    ("aubo_shoulder_joint", ("aubo_shoulder_joint", "shoulder_joint")),
    ("aubo_upperArm_joint", ("aubo_upperArm_joint", "upperArm_joint", "upper_arm_joint")),
    ("aubo_foreArm_joint", ("aubo_foreArm_joint", "foreArm_joint", "fore_arm_joint")),
    ("aubo_wrist1_joint", ("aubo_wrist1_joint", "wrist1_joint")),
    ("aubo_wrist2_joint", ("aubo_wrist2_joint", "wrist2_joint")),
    ("aubo_wrist3_joint", ("aubo_wrist3_joint", "wrist3_joint")),
)


class AuboJsonRpc:
    def __init__(self, ip: str, timeout: float) -> None:
        self.ip = ip
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def __enter__(self) -> "AuboJsonRpc":
        self.sock = socket.create_connection((self.ip, 30004), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if names:
            self.robot_name = str(names[0])
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self.sock is not None:
            self.sock.close()

    def call(self, method: str, params: list[Any] | None = None) -> Any:
        if self.sock is None:
            raise RuntimeError("not connected")
        self.request_id += 1
        request = {"jsonrpc": "2.0", "method": method, "params": params or [], "id": self.request_id}
        self.sock.sendall(json.dumps(request, separators=(",", ":")).encode("utf-8"))
        response = json.loads(self.sock.recv(8192).decode("utf-8", errors="replace"))
        if "error" in response:
            raise RuntimeError(f"{method} failed: {response['error']}")
        return response.get("result")

    def robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        return self.call(f"{self.robot_name}.{suffix}", params)


def normalize_joints(value: Any) -> list[float]:
    if isinstance(value, dict):
        for key in ("joint_positions", "jointPositions", "positions", "q", "data", "value"):
            if key in value:
                return normalize_joints(value[key])
    if isinstance(value, (list, tuple)) and len(value) >= 6:
        joints = [float(item) for item in value[:6]]
        if max(abs(item) for item in joints) > 10.0:
            raise RuntimeError(f"joint values do not look like radians: {joints}")
        return joints
    raise RuntimeError(f"cannot parse Aubo joint positions from {value!r}")


def read_rpc_joints(
    ip: str, timeout: float, allow_powered_off_rpc_pose: bool
) -> tuple[str, list[float]]:
    with AuboJsonRpc(ip, timeout) as rpc:
        joints = normalize_joints(rpc.robot_call("RobotState.getJointPositions"))
        mode = rpc.robot_call("RobotState.getRobotModeType")
        safety = rpc.robot_call("RobotState.getSafetyModeType")
        if (
            not allow_powered_off_rpc_pose
            and str(mode) == "PowerOff"
            and str(safety) == "Undefined"
            and max(abs(value) for value in joints) < 1e-9
        ):
            raise RuntimeError(
                "Aubo RPC is reachable, but RobotMode=PowerOff/SafetyMode=Undefined "
                "returned all-zero joints. Power on/start the robot or start the ROS "
                "driver so a real measured joint state is available."
            )
        return f"rpc:{ip}:30004 robot={rpc.robot_name} mode={mode} safety={safety}", joints


def read_topic_joints(topic: str, timeout: float) -> tuple[str, list[float]]:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError(f"ROS topic reader unavailable: {exc}") from exc

    found: list[float] | None = None

    def callback(msg: JointState) -> None:
        nonlocal found
        values: list[float] = []
        for canonical, aliases in JOINT_ALIASES:
            for alias in aliases:
                if alias in msg.name:
                    index = msg.name.index(alias)
                    if index < len(msg.position):
                        values.append(float(msg.position[index]))
                        break
            else:
                return
        found = values

    rclpy.init(args=None)
    node: Node | None = None
    try:
        node = rclpy.create_node("grasp_preview_real_pose_sync")
        node.create_subscription(JointState, topic, callback, 10)
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while found is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if found is None:
            raise RuntimeError(f"no complete Aubo joint state on {topic} within {timeout:.1f}s")
        return f"topic:{topic}", found
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def format_result(source: str, joints: list[float]) -> str:
    return source + "|" + ",".join(f"{value:.15g}" for value in joints)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=("auto", "topic", "rpc"), default="auto")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--topic-timeout", type=float, default=3.0)
    parser.add_argument("--ip", default="192.168.127.128")
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--allow-powered-off-rpc-pose", action="store_true")
    args = parser.parse_args()

    errors: list[str] = []
    if args.source in ("auto", "topic"):
        try:
            print(format_result(*read_topic_joints(args.joint_topic, args.topic_timeout)))
            return 0
        except Exception as exc:
            errors.append(f"topic: {exc}")
            if args.source == "topic":
                print(errors[-1], file=sys.stderr)
                return 1

    if args.source in ("auto", "rpc"):
        try:
            print(
                format_result(
                    *read_rpc_joints(
                        args.ip,
                        args.rpc_timeout,
                        bool(args.allow_powered_off_rpc_pose),
                    )
                )
            )
            return 0
        except Exception as exc:
            errors.append(f"rpc: {exc}")
            print("; ".join(errors), file=sys.stderr)
            return 1

    print("no synchronization source selected", file=sys.stderr)
    return 1


raise SystemExit(main())
PY
)"; then
  cat >&2 <<EOF
Failed to synchronize the real Aubo pose.

Checked source: ${SYNC_SOURCE}
ROS joint topic: ${REAL_JOINT_TOPIC}
Aubo RPC: ${AUBO_ROBOT_IP}:30004

If the arm is connected but ROS is not publishing state yet, verify the robot IP
or start the real Aubo driver in another terminal. This script only reads state;
it does not command motion.
EOF
  exit 1
fi

POSE_SOURCE="${POSE_LINE%%|*}"
JOINTS_CSV="${POSE_LINE#*|}"
if [[ "${POSE_SOURCE}" == "${POSE_LINE}" || -z "${JOINTS_CSV}" ]]; then
  echo "Failed to parse synchronized Aubo pose: ${POSE_LINE}" >&2
  exit 1
fi

LOG_DIR="${ROOT_DIR}/log/grasp_preview_real_sync/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"
{
  echo "source=${POSE_SOURCE}"
  echo "joints_csv=${JOINTS_CSV}"
  echo "export ARACHNE_GRASP_ARM_JOINTS=\"${JOINTS_CSV}\""
} >"${LOG_DIR}/real_pose.env"

echo "Synchronized real Aubo pose:"
echo "  source: ${POSE_SOURCE}"
echo "  joints: ${JOINTS_CSV}"
echo "  saved: ${LOG_DIR}/real_pose.env"

if [[ "${SYNC_ONLY}" == "true" ]]; then
  exit 0
fi

if [[ "${EXECUTE_REAL}" == "true" ]]; then
  export ARACHNE_GRASP_EXECUTE_REAL=true
fi
export ARACHNE_GRASP_REAL_JOINT_STATES_TOPIC="${REAL_JOINT_TOPIC}"
export ARACHNE_GRASP_ARM_JOINTS="${JOINTS_CSV}"
exec "${ROOT_DIR}/scripts/vision/grasp_preview.sh" "${PREVIEW_ARGS[@]}"
