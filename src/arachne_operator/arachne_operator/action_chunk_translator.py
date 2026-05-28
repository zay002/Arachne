from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arachne_operator.sequence_executor import ARM_JOINTS, ARM_PRESETS


@dataclass(frozen=True)
class ChunkStep:
    base_linear_x: float = 0.0
    base_angular_z: float = 0.0
    arm_positions: tuple[float, ...] | None = None
    arm_duration: float = 1.0
    gripper_command: str | None = None
    duration: float = 0.2


class ActionChunkTranslator(Node):
    """Translate external VLA/WAM action chunks into Arachne ROS command topics.

    The input is JSON on a std_msgs/String topic to keep the bridge dependency-light.
    It accepts either one step, a list of steps, or {"steps": [...]}.
    """

    def __init__(self) -> None:
        super().__init__("arachne_action_chunk_translator")
        self.declare_parameter("input_topic", "/arachne/vla/action_chunk")
        self.declare_parameter("status_topic", "/arachne/vla/translator/status")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory")
        self.declare_parameter(
            "legacy_arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("default_step_duration", 0.25)
        self.declare_parameter("default_arm_duration", 1.0)
        self.declare_parameter("max_linear_velocity", 0.8)
        self.declare_parameter("max_angular_velocity", 1.4)
        self.declare_parameter("max_arm_delta", 0.25)
        self.declare_parameter("array_arm_mode", "delta")

        self.default_step_duration = float(self.get_parameter("default_step_duration").value)
        self.default_arm_duration = float(self.get_parameter("default_arm_duration").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.max_arm_delta = float(self.get_parameter("max_arm_delta").value)
        self.array_arm_mode = str(self.get_parameter("array_arm_mode").value).lower()

        input_topic = self.get_parameter("input_topic").value
        status_topic = self.get_parameter("status_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        arm_topic = self.get_parameter("arm_trajectory_topic").value
        legacy_arm_topic = self.get_parameter("legacy_arm_trajectory_topic").value
        gripper_topic = self.get_parameter("gripper_command_topic").value

        self.current_arm = dict(zip(ARM_JOINTS, ARM_PRESETS["home"]))
        self.pending_steps: deque[ChunkStep] = deque()
        self.active_step: ChunkStep | None = None
        self.active_until_ns = 0
        self.active_twist = Twist()

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.arm_publishers = [
            self.create_publisher(JointTrajectory, arm_topic, 10),
            self.create_publisher(JointTrajectory, legacy_arm_topic, 10),
        ]
        self.gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(String, input_topic, self._chunk_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)
        self.create_service(Trigger, "/arachne/vla/translator/stop", self._stop_service)
        self.create_timer(0.05, self._tick)

        self._status(
            "ready: JSON action chunks on "
            f"{input_topic}; array schema [vx, wz, j1..j6, gripper]"
        )

    def _chunk_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if text.lower() == "stop":
            self._stop("external stop")
            return

        try:
            payload = json.loads(text)
            steps = self._parse_payload(payload)
        except Exception as exc:
            self._status(f"invalid action chunk: {exc}", warn=True)
            return

        self._stop("replace active chunk", publish_status=False)
        self.pending_steps = deque(steps)
        self._status(f"accepted action chunk: steps={len(steps)}")

    def _joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.current_arm:
                self.current_arm[name] = float(position)

    def _stop_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, text = self._stop("service stop")
        response.success = ok
        response.message = text
        return response

    def _parse_payload(self, payload: Any) -> list[ChunkStep]:
        if isinstance(payload, dict) and "steps" in payload:
            raw_steps = payload["steps"]
        elif isinstance(payload, dict) and "chunks" in payload:
            raw_steps = payload["chunks"]
        elif isinstance(payload, list) and payload and all(isinstance(item, dict) for item in payload):
            raw_steps = payload
        else:
            raw_steps = [payload]

        if not isinstance(raw_steps, list):
            raise ValueError("steps/chunks must be a list")

        steps = [self._parse_step(raw_step) for raw_step in raw_steps]
        if not steps:
            raise ValueError("chunk has no steps")
        return steps

    def _parse_step(self, raw: Any) -> ChunkStep:
        if isinstance(raw, list):
            return self._parse_action_array(raw, {})
        if not isinstance(raw, dict):
            raise ValueError(f"step must be object or action array, got {type(raw).__name__}")

        if "action" in raw:
            action = raw["action"]
            if not isinstance(action, list):
                raise ValueError("action must be a list")
            return self._parse_action_array(action, raw)

        duration = self._positive_float(raw.get("duration"), self.default_step_duration)
        base_linear_x, base_angular_z = self._parse_base(raw.get("base", raw))
        arm_positions, arm_duration = self._parse_arm(raw)
        gripper_command = self._parse_gripper(raw.get("gripper"))
        return ChunkStep(
            base_linear_x=base_linear_x,
            base_angular_z=base_angular_z,
            arm_positions=arm_positions,
            arm_duration=arm_duration,
            gripper_command=gripper_command,
            duration=duration,
        )

    def _parse_action_array(self, action: list[Any], raw: dict[str, Any]) -> ChunkStep:
        if len(action) < 2:
            raise ValueError("action array requires at least [linear_x, angular_z]")

        duration = self._positive_float(raw.get("duration"), self.default_step_duration)
        arm_duration = self._positive_float(
            raw.get("arm_duration", raw.get("duration")), self.default_arm_duration
        )
        base_linear_x = self._clamp_float(action[0], -self.max_linear_velocity, self.max_linear_velocity)
        base_angular_z = self._clamp_float(action[1], -self.max_angular_velocity, self.max_angular_velocity)

        arm_positions = None
        if len(action) >= 8:
            arm_values = [float(value) for value in action[2:8]]
            if str(raw.get("arm_mode", self.array_arm_mode)).lower() == "absolute":
                arm_positions = tuple(arm_values)
            else:
                arm_positions = self._delta_positions(arm_values)

        gripper_command = None
        if len(action) >= 9:
            gripper_command = "close" if float(action[8]) > 0.5 else "open"
        if "gripper" in raw:
            gripper_command = self._parse_gripper(raw.get("gripper"))

        return ChunkStep(
            base_linear_x=base_linear_x,
            base_angular_z=base_angular_z,
            arm_positions=arm_positions,
            arm_duration=arm_duration,
            gripper_command=gripper_command,
            duration=duration,
        )

    def _parse_base(self, raw: Any) -> tuple[float, float]:
        if isinstance(raw, list):
            if len(raw) < 2:
                raise ValueError("base array requires [linear_x, angular_z]")
            return (
                self._clamp_float(raw[0], -self.max_linear_velocity, self.max_linear_velocity),
                self._clamp_float(raw[1], -self.max_angular_velocity, self.max_angular_velocity),
            )
        if isinstance(raw, dict):
            linear = raw.get("linear_x", raw.get("vx", raw.get("linear", 0.0)))
            angular = raw.get("angular_z", raw.get("wz", raw.get("yaw_rate", 0.0)))
            return (
                self._clamp_float(linear, -self.max_linear_velocity, self.max_linear_velocity),
                self._clamp_float(angular, -self.max_angular_velocity, self.max_angular_velocity),
            )
        return 0.0, 0.0

    def _parse_arm(self, raw: dict[str, Any]) -> tuple[tuple[float, ...] | None, float]:
        arm = raw.get("arm", raw)
        arm_duration = self._positive_float(
            raw.get("arm_duration", raw.get("duration")), self.default_arm_duration
        )

        if isinstance(arm, str):
            return self._preset_positions(arm), arm_duration
        if not isinstance(arm, dict):
            return None, arm_duration

        if "duration" in arm:
            arm_duration = self._positive_float(arm.get("duration"), arm_duration)
        if "preset" in arm:
            return self._preset_positions(str(arm["preset"])), arm_duration
        if "positions" in arm:
            values = self._six_values(arm["positions"], "arm.positions")
            if str(arm.get("mode", "absolute")).lower() == "delta":
                return self._delta_positions(values), arm_duration
            return tuple(values), arm_duration
        if "joints" in arm:
            return self._joint_dict_positions(arm["joints"], absolute=True), arm_duration
        if "delta" in arm:
            values = arm["delta"]
            if isinstance(values, dict):
                return self._joint_dict_positions(values, absolute=False), arm_duration
            return self._delta_positions(self._six_values(values, "arm.delta")), arm_duration
        if "deltas" in arm:
            values = arm["deltas"]
            if isinstance(values, dict):
                return self._joint_dict_positions(values, absolute=False), arm_duration
            return self._delta_positions(self._six_values(values, "arm.deltas")), arm_duration

        if "arm_joints" in raw:
            return tuple(self._six_values(raw["arm_joints"], "arm_joints")), arm_duration
        if "arm_delta" in raw:
            return self._delta_positions(self._six_values(raw["arm_delta"], "arm_delta")), arm_duration
        return None, arm_duration

    def _parse_gripper(self, raw: Any) -> str | None:
        if raw is None:
            return None
        if isinstance(raw, dict):
            raw = raw.get("command", raw.get("state", raw.get("value")))
        if isinstance(raw, (int, float)):
            return "close" if float(raw) > 0.5 else "open"
        command = str(raw).strip().lower()
        if command in ("open", "close", "stop"):
            return command
        raise ValueError(f"unsupported gripper command: {raw!r}")

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.active_step is None and self.pending_steps:
            self._start_step(self.pending_steps.popleft())

        if self.active_step is None:
            return

        if now_ns <= self.active_until_ns:
            self.cmd_vel_pub.publish(self.active_twist)
            return

        self.cmd_vel_pub.publish(Twist())
        self.active_step = None
        if not self.pending_steps:
            self._status("action chunk complete")

    def _start_step(self, step: ChunkStep) -> None:
        self.active_step = step
        self.active_until_ns = self.get_clock().now().nanoseconds + int(max(step.duration, 0.05) * 1e9)
        self.active_twist = Twist()
        self.active_twist.linear.x = step.base_linear_x
        self.active_twist.angular.z = step.base_angular_z
        self.cmd_vel_pub.publish(self.active_twist)

        if step.arm_positions is not None:
            self._publish_arm(step.arm_positions, step.arm_duration)
        if step.gripper_command is not None:
            self._publish_gripper(step.gripper_command)

        self._status(
            "step start: "
            f"base=({step.base_linear_x:.2f},{step.base_angular_z:.2f}) "
            f"arm={'yes' if step.arm_positions is not None else 'no'} "
            f"gripper={step.gripper_command or 'none'} duration={step.duration:.2f}"
        )

    def _publish_arm(self, positions: tuple[float, ...], duration: float) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(ARM_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        for publisher in self.arm_publishers:
            publisher.publish(trajectory)
        self.current_arm.update(dict(zip(ARM_JOINTS, positions)))

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _stop(self, reason: str, publish_status: bool = True) -> tuple[bool, str]:
        self.pending_steps.clear()
        self.active_step = None
        self.active_until_ns = 0
        self.active_twist = Twist()
        self.cmd_vel_pub.publish(Twist())
        if publish_status:
            return self._status(f"translator stopped: {reason}")
        return True, reason

    def _preset_positions(self, preset: str) -> tuple[float, ...]:
        if preset not in ARM_PRESETS:
            raise ValueError(f"unknown arm preset: {preset}")
        return tuple(ARM_PRESETS[preset])

    def _joint_dict_positions(self, raw: Any, absolute: bool) -> tuple[float, ...]:
        if not isinstance(raw, dict):
            raise ValueError("joint command must be a dict")
        positions = [self.current_arm[name] for name in ARM_JOINTS]
        for index, name in enumerate(ARM_JOINTS):
            if name not in raw:
                continue
            value = float(raw[name])
            if absolute:
                positions[index] = value
            else:
                positions[index] += self._clamp(value, -self.max_arm_delta, self.max_arm_delta)
        return tuple(positions)

    def _delta_positions(self, deltas: list[float]) -> tuple[float, ...]:
        return tuple(
            self.current_arm[name] + self._clamp(delta, -self.max_arm_delta, self.max_arm_delta)
            for name, delta in zip(ARM_JOINTS, deltas)
        )

    def _six_values(self, raw: Any, label: str) -> list[float]:
        if not isinstance(raw, list) or len(raw) != len(ARM_JOINTS):
            raise ValueError(f"{label} must be a list of {len(ARM_JOINTS)} values")
        return [float(value) for value in raw]

    def _positive_float(self, value: Any, default: float) -> float:
        if value is None:
            return default
        return max(float(value), 0.0)

    def _clamp_float(self, value: Any, lower: float, upper: float) -> float:
        return self._clamp(float(value), lower, upper)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _status(self, text: str, warn: bool = False) -> tuple[bool, str]:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
            return False, text
        self.get_logger().info(text)
        return True, text


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActionChunkTranslator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
