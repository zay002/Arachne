#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

if [[ "${ARACHNE_CONFIRM_REAL_AUBO_HOLD:-}" != "YES" ]]; then
  echo "Refusing real current-state hold: set ARACHNE_CONFIRM_REAL_AUBO_HOLD=YES" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/env/arachne_env.sh"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
ACTION_NAME="${AUBO_HOLD_ACTION_NAME:-/arachne/aubo/move_joint}"
MAX_DELTA_RAD="${AUBO_HOLD_MAX_DELTA_RAD:-0.005}"

AUBO_ROBOT_IP="$AUBO_ROBOT_IP" ./scripts/hardware/check_aubo_running_readonly.sh

ros2 action info "$ACTION_NAME" >/dev/null

"${ARACHNE_SYSTEM_PYTHON:-python3}" - "$ACTION_NAME" "$MAX_DELTA_RAD" <<'PY'
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arachne_hardware.action import AuboMoveJoint


JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)


class CurrentHoldNode(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("aubo_current_hold_check")
        self.latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.client = ActionClient(self, AuboMoveJoint, action_name)

    def _joint_cb(self, msg: JointState) -> None:
        self.latest = msg

    def read_joints(self, timeout_sec: float) -> list[float]:
        deadline = time.monotonic() + timeout_sec
        self.latest = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                by_name = dict(zip(self.latest.name, self.latest.position))
                missing = [name for name in JOINTS if name not in by_name]
                if missing:
                    raise RuntimeError(f"/joint_states missing joints: {missing}")
                return [float(by_name[name]) for name in JOINTS]
        raise RuntimeError("timed out waiting for /joint_states")

    def send_hold(self, target: list[float], timeout_sec: float) -> tuple[int, AuboMoveJoint.Result]:
        if not self.client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("AuboMoveJoint action server unavailable; no SDK fallback allowed")
        goal = AuboMoveJoint.Goal()
        goal.target_joints = [float(value) for value in target]
        goal.speed_rad_sec = 0.05
        goal.accel_rad_sec2 = 0.10
        goal.blend_radius = 0.0
        goal.duration_sec = 0.0
        goal.goal_tolerance_rad = 0.03
        goal.timeout_sec = 5.0
        goal.label = "current_state_hold_check"
        print("action_goal:", json.dumps({
            "target_joints": goal.target_joints,
            "speed_rad_sec": goal.speed_rad_sec,
            "accel_rad_sec2": goal.accel_rad_sec2,
            "blend_radius": goal.blend_radius,
            "duration_sec": goal.duration_sec,
            "goal_tolerance_rad": goal.goal_tolerance_rad,
            "timeout_sec": goal.timeout_sec,
            "label": goal.label,
        }, separators=(",", ":")))
        send_future = self.client.send_goal_async(goal)
        self._wait_future(send_future, 2.0, "send goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("AuboMoveJoint hold goal rejected")
        result_future = goal_handle.get_result_async()
        self._wait_future(result_future, timeout_sec, "hold result")
        wrapped = result_future.result()
        return int(wrapped.status), wrapped.result

    def _wait_future(self, future, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"timed out waiting for {label}")


def main() -> int:
    action_name = sys.argv[1]
    max_delta = float(sys.argv[2])
    rclpy.init()
    node = CurrentHoldNode(action_name)
    try:
        target = node.read_joints(5.0)
        current = node.read_joints(2.0)
        deltas = [abs(a - b) for a, b in zip(target, current)]
        max_seen = max(deltas) if deltas else math.inf
        print("current_joint_target:", json.dumps(target, separators=(",", ":")))
        print(f"max_target_current_delta_rad: {max_seen:.9f}")
        if max_seen > max_delta:
            raise RuntimeError(
                f"target/current max delta {max_seen:.9f} exceeds {max_delta:.9f}; refusing hold"
            )
        status, result = node.send_hold(target, 8.0)
        print(f"action_status: {status}")
        print(
            "action_result:",
            json.dumps(
                {
                    "success": bool(status == GoalStatus.STATUS_SUCCEEDED and result.success),
                    "message": str(result.message),
                    "final_error_rad": float(result.final_error_rad),
                },
                separators=(",", ":"),
            ),
        )
        return 0 if status == GoalStatus.STATUS_SUCCEEDED and result.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
PY

echo "[real_aubo_current_hold_check] completed. No further motion commands were sent."
