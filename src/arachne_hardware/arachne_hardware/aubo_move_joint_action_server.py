from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from arachne_hardware.action import AuboMoveJoint
from arachne_hardware.aubo_sdk import (
    DEFAULT_AUBO_CONTROL_OWNER_PATH,
    DEFAULT_AUBO_TEACH_FLAG_PATH,
    AuboDirectJsonRpc,
)
from arachne_hardware.aubo_sdk.move_joint import (
    MoveJointConfig,
    execute_move_joint,
    read_final_error,
)
from arachne_hardware.aubo_sdk.safety import stop_joint


class AuboMoveJointActionServer(Node):
    """ROS action boundary for guarded Aubo SDK moveJoint."""

    def __init__(self) -> None:
        super().__init__("aubo_move_joint_action_server")
        self.declare_parameter("action_name", "/arachne/aubo/move_joint")
        self.declare_parameter("robot_ip", "192.168.127.128")
        self.declare_parameter("rpc_port", 30004)
        self.declare_parameter("rpc_timeout_sec", 3.0)
        self.declare_parameter("teach_flag_path", DEFAULT_AUBO_TEACH_FLAG_PATH)
        self.declare_parameter("control_owner_path", DEFAULT_AUBO_CONTROL_OWNER_PATH)
        self.declare_parameter("control_owner_name", "aubo_move_joint_action_server")
        self.declare_parameter("default_speed_rad_sec", 0.25)
        self.declare_parameter("default_accel_rad_sec2", 0.45)
        self.declare_parameter("default_goal_tolerance_rad", 0.04)
        self.declare_parameter("default_timeout_sec", 12.0)
        self.declare_parameter("arrival_timeout_padding_sec", 3.0)
        self.declare_parameter("gate_settle_sec", 0.15)
        self.declare_parameter("dry_run", False)

        self.status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        action_name = str(self.get_parameter("action_name").value)
        self.action_server = ActionServer(
            self,
            AuboMoveJoint,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self._publish_status(f"aubo moveJoint action server ready: {action_name}")

    def destroy_node(self) -> bool:
        self.action_server.destroy()
        return super().destroy_node()

    def _goal_callback(self, goal_request: AuboMoveJoint.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: Any) -> AuboMoveJoint.Result:
        goal = goal_handle.request
        started = time.monotonic()
        label = goal.label.strip() or "move_joint"
        feedback = AuboMoveJoint.Feedback()

        def publish_feedback(state: str, max_error: float = -1.0) -> None:
            feedback.state = state
            feedback.elapsed_sec = time.monotonic() - started
            feedback.max_error_rad = float(max_error)
            goal_handle.publish_feedback(feedback)

        publish_feedback("accepted")
        if len(goal.target_joints) != 6:
            goal_handle.abort()
            return self._result(False, "target_joints must contain exactly 6 values", -1.0)

        target = [float(value) for value in goal.target_joints]
        if bool(self.get_parameter("dry_run").value):
            return self._execute_dry_run(goal_handle, label, started, feedback)

        config = self._config_from_goal(goal)
        final_error = -1.0
        try:
            success = execute_move_joint(
                target,
                label,
                config,
                cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
                progress=publish_feedback,
                status=lambda text, warn: self._publish_status(text, warn=warn),
            )
            final_error = self._read_final_error_best_effort(config, target)
            if goal_handle.is_cancel_requested:
                self._stop_joint_best_effort(config, "cancel")
                publish_feedback("canceled", final_error)
                goal_handle.canceled()
                return self._result(False, f"Aubo SDK moveJoint canceled at {label}", final_error)
            if success:
                publish_feedback("completed", final_error)
                goal_handle.succeed()
                return self._result(True, f"Aubo SDK moveJoint completed: {label}", final_error)
            publish_feedback("failed", final_error)
            goal_handle.abort()
            return self._result(False, f"Aubo SDK moveJoint failed: {label}", final_error)
        except Exception as exc:  # pragma: no cover - live hardware dependent.
            final_error = self._read_final_error_best_effort(config, target)
            if goal_handle.is_cancel_requested or "cancel" in str(exc).lower():
                self._stop_joint_best_effort(config, "cancel")
                publish_feedback("canceled", final_error)
                goal_handle.canceled()
                return self._result(False, f"Aubo SDK moveJoint canceled at {label}: {exc}", final_error)
            publish_feedback("failed", final_error)
            self._publish_status(f"Aubo SDK moveJoint action failed at {label}: {exc}", warn=True)
            goal_handle.abort()
            return self._result(False, f"Aubo SDK moveJoint failed at {label}: {exc}", final_error)

    def _execute_dry_run(
        self,
        goal_handle: Any,
        label: str,
        started: float,
        feedback: AuboMoveJoint.Feedback,
    ) -> AuboMoveJoint.Result:
        """Simulate the action contract without touching the Aubo SDK."""

        def publish_feedback(state: str, max_error: float = 0.0) -> None:
            feedback.state = state
            feedback.elapsed_sec = time.monotonic() - started
            feedback.max_error_rad = float(max_error)
            goal_handle.publish_feedback(feedback)

        self._publish_status(f"Aubo moveJoint dry-run accepted: {label}")
        for state in ("checking_state", "motion_started", "waiting_arrival"):
            if goal_handle.is_cancel_requested:
                publish_feedback("canceled", 0.0)
                goal_handle.canceled()
                return self._result(False, f"dry-run canceled: {label}", 0.0)
            publish_feedback(state, 0.0)
            time.sleep(0.05)

        if goal_handle.is_cancel_requested:
            publish_feedback("canceled", 0.0)
            goal_handle.canceled()
            return self._result(False, f"dry-run canceled: {label}", 0.0)

        publish_feedback("completed", 0.0)
        goal_handle.succeed()
        return self._result(True, "dry-run completed", 0.0)

    def _config_from_goal(self, goal: AuboMoveJoint.Goal) -> MoveJointConfig:
        speed = (
            float(goal.speed_rad_sec)
            if goal.speed_rad_sec > 0.0
            else float(self.get_parameter("default_speed_rad_sec").value)
        )
        accel = (
            float(goal.accel_rad_sec2)
            if goal.accel_rad_sec2 > 0.0
            else float(self.get_parameter("default_accel_rad_sec2").value)
        )
        tolerance = (
            float(goal.goal_tolerance_rad)
            if goal.goal_tolerance_rad > 0.0
            else float(self.get_parameter("default_goal_tolerance_rad").value)
        )
        timeout = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else float(self.get_parameter("default_timeout_sec").value)
        )
        return MoveJointConfig(
            ip=str(self.get_parameter("robot_ip").value),
            port=int(self.get_parameter("rpc_port").value),
            rpc_timeout=max(float(self.get_parameter("rpc_timeout_sec").value), 0.1),
            speed=max(speed, 0.01),
            accel=max(accel, 0.05),
            blend_radius=max(float(goal.blend_radius), 0.0),
            duration=max(float(goal.duration_sec), 0.0),
            tolerance=max(tolerance, 0.001),
            exec_timeout=max(timeout, 0.5),
            arrival_timeout_padding=max(
                float(self.get_parameter("arrival_timeout_padding_sec").value), 0.0
            ),
            owner_path=Path(str(self.get_parameter("control_owner_path").value)),
            owner_name=str(self.get_parameter("control_owner_name").value).strip()
            or "aubo_move_joint_action_server",
            teach_flag_path=Path(str(self.get_parameter("teach_flag_path").value)),
            gate_settle_sec=max(float(self.get_parameter("gate_settle_sec").value), 0.0),
        )

    def _read_final_error_best_effort(self, config: MoveJointConfig, target: list[float]) -> float:
        try:
            return float(read_final_error(config, target))
        except Exception:
            return -1.0

    def _stop_joint_best_effort(self, config: MoveJointConfig, reason: str) -> None:
        try:
            with AuboDirectJsonRpc(config.ip, config.port, config.rpc_timeout) as rpc:
                stop_joint(
                    rpc,
                    config.accel,
                    reason,
                    warn_only=True,
                    status=lambda text, warn: self._publish_status(text, warn=warn),
                )
        except Exception as exc:
            self._publish_status(f"Aubo SDK stopJoint failed during {reason}: {exc}", warn=True)

    def _result(self, success: bool, message: str, final_error: float) -> AuboMoveJoint.Result:
        result = AuboMoveJoint.Result()
        result.success = bool(success)
        result.message = message
        result.final_error_rad = float(final_error)
        return result

    def _publish_status(self, text: str, *, warn: bool = False) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main() -> None:
    rclpy.init()
    node = AuboMoveJointActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
