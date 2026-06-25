from __future__ import annotations

import time
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from arachne_hardware.action import AuboMoveJoint


@dataclass(frozen=True)
class AuboMoveJointResult:
    success: bool
    message: str
    final_error_rad: float


class AuboMoveJointClient:
    """Small synchronous wrapper around /arachne/aubo/move_joint.

    The helper owns no JSON-RPC fallback. Callers decide whether to use their
    legacy guarded path when the action server is unavailable or fails.
    """

    def __init__(self, node, action_name: str = "/arachne/aubo/move_joint") -> None:
        self.node = node
        self.action_name = action_name
        self.client = ActionClient(node, AuboMoveJoint, action_name)

    def wait_for_server(self, timeout_sec: float) -> bool:
        try:
            return bool(self.client.wait_for_server(timeout_sec=max(float(timeout_sec), 0.0)))
        except Exception:
            return False

    def move_joint(
        self,
        target_joints: list[float],
        *,
        label: str = "task_waypoint",
        speed_rad_sec: float = 0.25,
        accel_rad_sec2: float = 0.45,
        blend_radius: float = 0.0,
        duration_sec: float = 0.0,
        goal_tolerance_rad: float = 0.04,
        timeout_sec: float = 30.0,
    ) -> tuple[bool, str, float]:
        if len(target_joints) != 6:
            return False, f"{label}: target_joints must contain 6 values", -1.0
        if not self.wait_for_server(0.0):
            return False, f"AuboMoveJoint action server unavailable: {self.action_name}", -1.0

        goal = AuboMoveJoint.Goal()
        goal.target_joints = [float(value) for value in target_joints]
        goal.speed_rad_sec = float(speed_rad_sec)
        goal.accel_rad_sec2 = float(accel_rad_sec2)
        goal.blend_radius = float(blend_radius)
        goal.duration_sec = float(duration_sec)
        goal.goal_tolerance_rad = float(goal_tolerance_rad)
        goal.timeout_sec = float(timeout_sec)
        goal.label = str(label)

        try:
            send_future = self.client.send_goal_async(goal)
        except Exception as exc:
            return False, f"AuboMoveJoint send failed at {label}: {exc}", -1.0
        if not self._wait_future(send_future, timeout_sec=2.0):
            return False, f"AuboMoveJoint send timeout at {label}", -1.0
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, f"AuboMoveJoint rejected at {label}", -1.0

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=max(float(timeout_sec), 0.1)):
            try:
                cancel_future = goal_handle.cancel_goal_async()
                self._wait_future(cancel_future, timeout_sec=1.0)
            except Exception:
                pass
            return False, f"AuboMoveJoint timeout at {label}", -1.0
        wrapped = result_future.result()
        result = wrapped.result
        success = bool(wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success)
        return success, str(result.message), float(result.final_error_rad)

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while not future.done() and time.monotonic() < deadline:
            if not self._spin_once():
                time.sleep(0.02)
        return bool(future.done())

    def _spin_once(self) -> bool:
        try:
            import rclpy

            rclpy.spin_once(self.node, timeout_sec=0.02)
            return True
        except Exception:
            return False
