from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics


AUBO_JOINT_ALIASES = (
    ("shoulder_joint", ("shoulder_joint", "aubo_shoulder_joint")),
    ("upperArm_joint", ("upperArm_joint", "aubo_upperArm_joint", "upper_arm_joint")),
    ("foreArm_joint", ("foreArm_joint", "aubo_foreArm_joint", "fore_arm_joint")),
    ("wrist1_joint", ("wrist1_joint", "aubo_wrist1_joint")),
    ("wrist2_joint", ("wrist2_joint", "aubo_wrist2_joint")),
    ("wrist3_joint", ("wrist3_joint", "aubo_wrist3_joint")),
)


TOOL_SCHEMAS = [
    {
        "name": "get_robot_state",
        "description": "Return cached base, arm, gripper, safety, and task state.",
        "danger": "read_only",
        "schema": {"tool": "get_robot_state"},
    },
    {
        "name": "safe_stop",
        "description": "Publish zero base and arm velocity, stop gripper, and stop task servers.",
        "danger": "stop",
        "schema": {"tool": "safe_stop"},
    },
    {
        "name": "base_velocity",
        "description": "Bounded timed /cmd_vel command for small teach-like base nudges.",
        "danger": "motion",
        "schema": {
            "tool": "base_velocity",
            "linear_x": 0.0,
            "angular_z": 0.0,
            "duration_sec": 0.2,
        },
    },
    {
        "name": "base_relative",
        "description": "Forward relative base motion to grasp_task_server base primitive.",
        "danger": "motion",
        "schema": {"tool": "base_relative", "distance_m": 0.1},
    },
    {
        "name": "base_turn",
        "description": "Relative yaw motion through grasp_task_server base primitive.",
        "danger": "motion",
        "schema": {"tool": "base_turn", "angle_deg": 10.0},
    },
    {
        "name": "arm_cartesian_jog",
        "description": "Move the end effector along base-frame x/y/z using damped IK velocity.",
        "danger": "motion",
        "schema": {"tool": "arm_cartesian_jog", "axis": "z", "distance_m": 0.01},
    },
    {
        "name": "arm_joint_jog",
        "description": "Rotate one Aubo joint by a small relative angle.",
        "danger": "motion",
        "schema": {"tool": "arm_joint_jog", "joint": "wrist3_joint", "delta_rad": 0.03},
    },
    {
        "name": "arm_stop",
        "description": "Publish zero joint velocity several times.",
        "danger": "stop",
        "schema": {"tool": "arm_stop"},
    },
    {
        "name": "gripper",
        "description": "Command the gripper to open, close, or stop.",
        "danger": "motion",
        "schema": {"tool": "gripper", "command": "open"},
    },
    {
        "name": "aubo_teach",
        "description": "Request Aubo teach/freedrive mode through the existing teach command topic.",
        "danger": "mode_change",
        "schema": {"tool": "aubo_teach", "enabled": False},
    },
]


def _yaw_from_odom(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _axis_vector(axis: str) -> np.ndarray:
    axes = {
        "x": np.array([1.0, 0.0, 0.0], dtype=float),
        "y": np.array([0.0, 1.0, 0.0], dtype=float),
        "z": np.array([0.0, 0.0, 1.0], dtype=float),
    }
    if axis not in axes:
        raise ValueError(f"axis must be x, y, or z; got {axis!r}")
    return axes[axis]


@dataclass
class CommandResult:
    ok: bool
    message: str
    data: dict[str, Any] = field(default_factory=dict)


class AgentBridge(Node):
    """Bounded teach-control bridge for an external agent.

    The bridge exposes a small JSON tool surface. It deliberately does not load
    API keys or talk to an LLM provider; external agent processes call this node
    after they have made a task decision.
    """

    def __init__(self) -> None:
        super().__init__("arachne_agent_bridge")

        self.declare_parameter("command_topic", "/arachne/agent/command")
        self.declare_parameter("status_topic", "/arachne/agent/status")
        self.declare_parameter("event_topic", "/arachne/agent/event")
        self.declare_parameter("tools_topic", "/arachne/agent/tools")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("arm_velocity_command_topic", "/arachne/aubo/joint_velocity_command")
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("aubo_teach_command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("base_task_command_topic", "/arachne/grasp_task/base_command")
        self.declare_parameter("grasp_task_state_topic", "/arachne/grasp_task/state")
        self.declare_parameter("base_task_state_topic", "/arachne/grasp_task/base_state")
        self.declare_parameter("aubo_status_topic", "/arachne/hardware/aubo_status")
        self.declare_parameter("base_status_topic", "/arachne/hardware/base_status")
        self.declare_parameter("gripper_status_topic", "/arachne/hardware/gripper_status")
        self.declare_parameter("safety_state_topic", "/arachne/safety/state")

        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("confirm_agent_motion", False)
        self.declare_parameter("allow_mode_change", False)
        self.declare_parameter("require_fresh_joint_states", True)
        self.declare_parameter("fresh_state_timeout_sec", 2.0)
        self.declare_parameter("status_publish_period_sec", 0.5)
        self.declare_parameter("command_deadman_sec", 2.0)
        self.declare_parameter("max_base_linear_x", 0.20)
        self.declare_parameter("max_base_angular_z", 0.35)
        self.declare_parameter("max_base_command_duration_sec", 2.0)
        self.declare_parameter("max_base_relative_distance_m", 0.30)
        self.declare_parameter("max_base_relative_yaw_deg", 30.0)
        self.declare_parameter("arm_cartesian_speed_m_sec", 0.015)
        self.declare_parameter("arm_joint_speed_rad_sec", 0.06)
        self.declare_parameter("max_arm_cartesian_step_m", 0.03)
        self.declare_parameter("max_arm_joint_step_rad", 0.12)
        self.declare_parameter("arm_velocity_publish_rate", 80.0)
        self.declare_parameter("arm_velocity_damping", 0.08)
        self.declare_parameter("arm_velocity_max_joint_speed_rad_sec", 0.18)

        self.lock = threading.RLock()
        self.kinematics = AuboI5Kinematics()
        self.current_arm: dict[str, float] = {}
        self.latest: dict[str, tuple[float, Any]] = {}
        self.last_result = CommandResult(ok=True, message="ready")
        self.active_command = ""
        self.active_until = 0.0
        self.active_base_velocity: tuple[float, float] | None = None
        self.active_arm_velocity: list[float] | None = None

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.arm_velocity_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("arm_velocity_command_topic").value),
            10,
        )
        self.gripper_pub = self.create_publisher(
            String, str(self.get_parameter("gripper_command_topic").value), 10
        )
        self.aubo_teach_pub = self.create_publisher(
            String, str(self.get_parameter("aubo_teach_command_topic").value), 10
        )
        self.base_task_pub = self.create_publisher(
            String, str(self.get_parameter("base_task_command_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10
        )
        self.tools_pub = self.create_publisher(
            String, str(self.get_parameter("tools_topic").value), 1
        )

        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._command_cb,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            lambda msg: self._cache("odom", self._odom_snapshot(msg)),
            10,
        )
        for key, topic_name in (
            ("aubo_status", "aubo_status_topic"),
            ("base_status", "base_status_topic"),
            ("gripper_status", "gripper_status_topic"),
            ("safety_state", "safety_state_topic"),
            ("grasp_task_state", "grasp_task_state_topic"),
            ("base_task_state", "base_task_state_topic"),
        ):
            self.create_subscription(
                String,
                str(self.get_parameter(topic_name).value),
                lambda msg, cache_key=key: self._cache(cache_key, msg.data),
                10,
            )

        self.create_service(Trigger, "/arachne/agent/status", self._status_service)
        self.create_service(Trigger, "/arachne/agent/tools", self._tools_service)
        self.create_service(Trigger, "/arachne/agent/safe_stop", self._stop_service)

        status_period = max(float(self.get_parameter("status_publish_period_sec").value), 0.1)
        self.create_timer(status_period, self._publish_status)
        self.create_timer(status_period, self._publish_tools)
        velocity_period = 1.0 / max(float(self.get_parameter("arm_velocity_publish_rate").value), 1.0)
        self.create_timer(velocity_period, self._publish_active_motion)
        self._event("ready", {"tools": [tool["name"] for tool in TOOL_SCHEMAS]})

    def _command_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            payload: Any = json.loads(text)
        except json.JSONDecodeError:
            payload = {"tool": text}
        try:
            result = self._handle_command(payload)
        except Exception as exc:
            result = CommandResult(False, f"agent command rejected: {exc}")
        with self.lock:
            self.last_result = result
        self._event("command_result", {"result": asdict(result), "payload": self._redacted(payload)})
        self._publish_status()

    def _handle_command(self, payload: Any) -> CommandResult:
        if isinstance(payload, str):
            payload = {"tool": payload}
        if not isinstance(payload, dict):
            raise ValueError("command must be a JSON object or a tool string")
        tool = str(payload.get("tool", payload.get("command", ""))).strip().lower()
        if not tool:
            raise ValueError("missing tool")
        tool = {
            "state": "get_robot_state",
            "status": "get_robot_state",
            "stop": "safe_stop",
            "base_move": "base_velocity",
            "base_cmd_vel": "base_velocity",
            "drive_relative": "base_relative",
            "turn_relative": "base_turn",
            "arm_xyz": "arm_cartesian_jog",
            "tcp_jog": "arm_cartesian_jog",
            "joint_jog": "arm_joint_jog",
            "open_gripper": "gripper",
            "close_gripper": "gripper",
            "teach": "aubo_teach",
        }.get(tool, tool)

        if tool == "get_robot_state":
            return CommandResult(True, "robot state", {"state": self._state_snapshot()})
        if tool == "safe_stop":
            return self._safe_stop("agent safe_stop")
        if tool == "arm_stop":
            self._publish_arm_velocity_zero(repeat=4)
            return CommandResult(True, "arm velocity stopped")
        self._require_motion_allowed(tool)

        if tool == "base_velocity":
            return self._base_velocity(payload)
        if tool == "base_relative":
            return self._base_relative(payload)
        if tool == "base_turn":
            return self._base_turn(payload)
        if tool == "arm_cartesian_jog":
            return self._arm_cartesian_jog(payload)
        if tool == "arm_joint_jog":
            return self._arm_joint_jog(payload)
        if tool == "gripper":
            return self._gripper(payload)
        if tool == "aubo_teach":
            return self._aubo_teach(payload)
        raise ValueError(f"unknown agent tool: {tool}")

    def _require_motion_allowed(self, tool: str) -> None:
        if not bool(self.get_parameter("motion_enabled").value):
            raise PermissionError(f"{tool} requires motion_enabled:=true")
        if not bool(self.get_parameter("confirm_agent_motion").value):
            raise PermissionError(f"{tool} requires confirm_agent_motion:=true")

    def _base_velocity(self, payload: dict[str, Any]) -> CommandResult:
        linear_x = self._clamp(
            float(payload.get("linear_x", payload.get("vx", 0.0))),
            -float(self.get_parameter("max_base_linear_x").value),
            float(self.get_parameter("max_base_linear_x").value),
        )
        angular_z = self._clamp(
            float(payload.get("angular_z", payload.get("wz", 0.0))),
            -float(self.get_parameter("max_base_angular_z").value),
            float(self.get_parameter("max_base_angular_z").value),
        )
        duration = self._bounded_duration(payload.get("duration_sec", payload.get("duration", 0.2)))
        with self.lock:
            self.active_command = "base_velocity"
            self.active_until = time.monotonic() + duration
            self.active_base_velocity = (linear_x, angular_z)
            self.active_arm_velocity = None
        self._publish_base_velocity(linear_x, angular_z)
        return CommandResult(
            True,
            f"base velocity active for {duration:.2f}s",
            {"linear_x": linear_x, "angular_z": angular_z, "duration_sec": duration},
        )

    def _base_relative(self, payload: dict[str, Any]) -> CommandResult:
        limit = float(self.get_parameter("max_base_relative_distance_m").value)
        distance = self._clamp(
            float(payload.get("distance_m", payload.get("distance", 0.0))),
            -limit,
            limit,
        )
        command = {"command": "drive_relative", "distance_m": distance}
        self._publish_json(self.base_task_pub, command)
        return CommandResult(True, "base relative command forwarded", command)

    def _base_turn(self, payload: dict[str, Any]) -> CommandResult:
        limit = float(self.get_parameter("max_base_relative_yaw_deg").value)
        if "angle_rad" in payload:
            angle_deg = math.degrees(float(payload["angle_rad"]))
        else:
            angle_deg = float(payload.get("angle_deg", payload.get("angle", 0.0)))
        angle_deg = self._clamp(angle_deg, -limit, limit)
        command = {"command": "turn_relative", "angle_deg": angle_deg}
        self._publish_json(self.base_task_pub, command)
        return CommandResult(True, "base turn command forwarded", command)

    def _arm_cartesian_jog(self, payload: dict[str, Any]) -> CommandResult:
        current = self._current_arm(required=True)
        vector = self._cartesian_vector(payload)
        distance = float(np.linalg.norm(vector))
        limit = float(self.get_parameter("max_arm_cartesian_step_m").value)
        if distance > limit:
            vector = vector * (limit / distance)
            distance = limit
        speed = self._positive(
            payload.get("speed_m_s", self.get_parameter("arm_cartesian_speed_m_sec").value),
            fallback=float(self.get_parameter("arm_cartesian_speed_m_sec").value),
        )
        duration = self._bounded_duration(payload.get("duration_sec", distance / max(speed, 1e-4)))
        twist = np.zeros(6, dtype=float)
        twist[:3] = vector / max(duration, 1e-3)
        velocity = self._joint_velocity_from_twist(np.array(current, dtype=float), twist)
        self._activate_arm_velocity(velocity, duration, "arm_cartesian_jog")
        return CommandResult(
            True,
            f"arm Cartesian jog active for {duration:.2f}s",
            {"vector_m": [float(value) for value in vector], "duration_sec": duration},
        )

    def _arm_joint_jog(self, payload: dict[str, Any]) -> CommandResult:
        current = self._current_arm(required=True)
        index = self._joint_index(payload.get("joint", payload.get("index", 0)))
        limit = float(self.get_parameter("max_arm_joint_step_rad").value)
        delta = self._clamp(
            float(payload.get("delta_rad", payload.get("delta", 0.0))),
            -limit,
            limit,
        )
        speed = self._positive(
            payload.get("speed_rad_s", self.get_parameter("arm_joint_speed_rad_sec").value),
            fallback=float(self.get_parameter("arm_joint_speed_rad_sec").value),
        )
        duration = self._bounded_duration(payload.get("duration_sec", abs(delta) / max(speed, 1e-4)))
        velocity = [0.0 for _ in current]
        velocity[index] = delta / max(duration, 1e-3)
        self._activate_arm_velocity(velocity, duration, "arm_joint_jog")
        return CommandResult(
            True,
            f"arm joint jog active for {duration:.2f}s",
            {
                "joint": AUBO_JOINT_ALIASES[index][0],
                "delta_rad": delta,
                "duration_sec": duration,
            },
        )

    def _gripper(self, payload: dict[str, Any]) -> CommandResult:
        command = str(payload.get("command", payload.get("state", ""))).strip().lower()
        if not command and str(payload.get("tool", "")).lower() == "open_gripper":
            command = "open"
        elif not command and str(payload.get("tool", "")).lower() == "close_gripper":
            command = "close"
        if command not in ("open", "close", "stop"):
            raise ValueError("gripper command must be open, close, or stop")
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        return CommandResult(True, f"gripper {command}", {"command": command})

    def _aubo_teach(self, payload: dict[str, Any]) -> CommandResult:
        if not bool(self.get_parameter("allow_mode_change").value):
            raise PermissionError("aubo_teach requires allow_mode_change:=true")
        enabled = bool(payload.get("enabled", payload.get("value", False)))
        msg = String()
        msg.data = "teach_on" if enabled else "teach_off"
        self.aubo_teach_pub.publish(msg)
        return CommandResult(True, f"aubo {msg.data}", {"enabled": enabled})

    def _safe_stop(self, reason: str) -> CommandResult:
        with self.lock:
            self.active_command = ""
            self.active_until = 0.0
            self.active_base_velocity = None
            self.active_arm_velocity = None
        self._publish_base_velocity(0.0, 0.0)
        self._publish_arm_velocity_zero(repeat=4)
        self._publish_string(self.gripper_pub, "stop")
        self._publish_json(self.base_task_pub, {"command": "stop"})
        return CommandResult(True, f"safe stop: {reason}")

    def _cartesian_vector(self, payload: dict[str, Any]) -> np.ndarray:
        if "vector" in payload:
            raw = payload["vector"]
            if not isinstance(raw, list) or len(raw) != 3:
                raise ValueError("vector must be [x, y, z]")
            return np.array([float(value) for value in raw], dtype=float)
        if any(key in payload for key in ("x", "y", "z")):
            return np.array(
                [
                    float(payload.get("x", 0.0)),
                    float(payload.get("y", 0.0)),
                    float(payload.get("z", 0.0)),
                ],
                dtype=float,
            )
        axis = str(payload.get("axis", "")).strip().lower()
        sign = float(payload.get("sign", 1.0))
        distance = float(payload.get("distance_m", payload.get("distance", 0.0)))
        if abs(distance) < 1e-9:
            distance = float(self.get_parameter("max_arm_cartesian_step_m").value)
        return _axis_vector(axis) * math.copysign(abs(distance), sign)

    def _joint_index(self, raw: Any) -> int:
        if isinstance(raw, str):
            text = raw.strip()
            if text.upper().startswith("J") and text[1:].isdigit():
                return self._joint_index(int(text[1:]) - 1)
            for index, (canonical, aliases) in enumerate(AUBO_JOINT_ALIASES):
                if text in aliases or text == canonical:
                    return index
            if text.isdigit() or (text.startswith("-") and text[1:].isdigit()):
                return self._joint_index(int(text))
            raise ValueError(f"unknown joint: {raw!r}")
        index = int(raw)
        if index < 0:
            raise ValueError(f"joint index must be 0-5 or J1-J6; got {raw!r}")
        if index >= len(AUBO_JOINT_ALIASES):
            raise ValueError(f"joint index out of range: {raw!r}")
        return index

    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        found: dict[str, float] = {}
        for canonical, aliases in AUBO_JOINT_ALIASES:
            for alias in aliases:
                if alias in positions:
                    found[canonical] = float(positions[alias])
                    break
        with self.lock:
            self.latest["joint_states"] = (time.monotonic(), {"names": list(msg.name)})
            if len(found) == len(AUBO_JOINT_ALIASES):
                self.current_arm = found
                self.latest["aubo_joints"] = (time.monotonic(), dict(found))

    def _cache(self, key: str, value: Any) -> None:
        with self.lock:
            self.latest[key] = (time.monotonic(), value)

    def _odom_snapshot(self, msg: Odometry) -> dict[str, float]:
        pose = msg.pose.pose
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(_yaw_from_odom(msg)),
        }

    def _current_arm(self, *, required: bool) -> list[float]:
        with self.lock:
            values = [self.current_arm.get(name) for name, _aliases in AUBO_JOINT_ALIASES]
            stamp_item = self.latest.get("aubo_joints")
        if not all(value is not None for value in values):
            if required:
                raise RuntimeError("no complete Aubo joint state")
            return []
        if bool(self.get_parameter("require_fresh_joint_states").value):
            if stamp_item is None:
                raise RuntimeError("no fresh Aubo joint state")
            age = time.monotonic() - stamp_item[0]
            if age > float(self.get_parameter("fresh_state_timeout_sec").value):
                raise RuntimeError(f"Aubo joint state is stale: {age:.2f}s")
        return [float(value) for value in values if value is not None]

    def _joint_velocity_from_twist(self, q: np.ndarray, twist: np.ndarray) -> list[float]:
        jacobian = self._numeric_twist_jacobian(q)
        damping = max(float(self.get_parameter("arm_velocity_damping").value), 1e-4)
        lhs = jacobian @ jacobian.T + (damping**2) * np.eye(6)
        qdot = jacobian.T @ np.linalg.solve(lhs, twist)
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        peak = float(np.max(np.abs(qdot))) if qdot.size else 0.0
        if peak > max_speed:
            qdot = qdot * (max_speed / peak)
        return [float(value) for value in qdot]

    def _numeric_twist_jacobian(self, q: np.ndarray) -> np.ndarray:
        eps = 1e-5
        base_transform = self.kinematics.fk(q)
        base_position = base_transform[:3, 3]
        base_rotation = base_transform[:3, :3]
        jacobian = np.zeros((6, len(q)))
        for index in range(len(q)):
            q_eps = np.array(q, dtype=float)
            q_eps[index] += eps
            next_transform = self.kinematics.fk(q_eps)
            jacobian[:3, index] = (next_transform[:3, 3] - base_position) / eps
            jacobian[3:, index] = (
                self.kinematics._rotation_vector(next_transform[:3, :3] @ base_rotation.T)
                / eps
            )
        return jacobian

    def _activate_arm_velocity(self, velocity: list[float], duration: float, label: str) -> None:
        if len(velocity) != len(AUBO_JOINT_ALIASES):
            raise ValueError("arm velocity must contain 6 values")
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        clamped = [self._clamp(float(value), -max_speed, max_speed) for value in velocity]
        with self.lock:
            self.active_command = label
            self.active_until = time.monotonic() + duration
            self.active_arm_velocity = clamped
            self.active_base_velocity = None
        self._publish_arm_velocity(clamped)

    def _publish_active_motion(self) -> None:
        now = time.monotonic()
        with self.lock:
            active_until = self.active_until
            base_velocity = self.active_base_velocity
            arm_velocity = list(self.active_arm_velocity) if self.active_arm_velocity else None
            expired = active_until > 0.0 and now > active_until
            if expired:
                self.active_command = ""
                self.active_until = 0.0
                self.active_base_velocity = None
                self.active_arm_velocity = None
        if expired:
            self._publish_base_velocity(0.0, 0.0)
            self._publish_arm_velocity_zero(repeat=2)
            return
        if base_velocity is not None:
            self._publish_base_velocity(base_velocity[0], base_velocity[1])
        if arm_velocity is not None:
            self._publish_arm_velocity(arm_velocity)

    def _publish_base_velocity(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def _publish_arm_velocity(self, velocity: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in velocity]
        self.arm_velocity_pub.publish(msg)

    def _publish_arm_velocity_zero(self, *, repeat: int) -> None:
        for _ in range(max(int(repeat), 1)):
            self._publish_arm_velocity([0.0 for _ in AUBO_JOINT_ALIASES])

    def _status_service(self, _request, response):
        response.success = True
        response.message = json.dumps(self._state_snapshot(), ensure_ascii=False, sort_keys=True)
        return response

    def _tools_service(self, _request, response):
        response.success = True
        response.message = json.dumps({"tools": TOOL_SCHEMAS}, ensure_ascii=False, sort_keys=True)
        return response

    def _stop_service(self, _request, response):
        result = self._safe_stop("service")
        response.success = result.ok
        response.message = result.message
        return response

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._state_snapshot(), ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(msg)

    def _publish_tools(self) -> None:
        msg = String()
        msg.data = json.dumps({"tools": TOOL_SCHEMAS}, ensure_ascii=False, sort_keys=True)
        self.tools_pub.publish(msg)

    def _state_snapshot(self) -> dict[str, Any]:
        with self.lock:
            latest = {
                key: {"age_sec": round(time.monotonic() - stamp, 3), "value": value}
                for key, (stamp, value) in self.latest.items()
            }
            last_result = asdict(self.last_result)
            active = {
                "command": self.active_command,
                "remaining_sec": max(0.0, self.active_until - time.monotonic()),
                "base_velocity": list(self.active_base_velocity)
                if self.active_base_velocity is not None
                else [],
                "arm_velocity": list(self.active_arm_velocity)
                if self.active_arm_velocity is not None
                else [],
            }
            arm = dict(self.current_arm)
        return {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "motion_enabled": bool(self.get_parameter("motion_enabled").value),
            "confirm_agent_motion": bool(self.get_parameter("confirm_agent_motion").value),
            "allow_mode_change": bool(self.get_parameter("allow_mode_change").value),
            "active": active,
            "arm_joints": arm,
            "latest": latest,
            "last_result": last_result,
        }

    def _event(self, kind: str, data: dict[str, Any]) -> None:
        event = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            **data,
        }
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False, sort_keys=True)
        self.event_pub.publish(msg)
        if kind in ("ready", "command_result"):
            self.get_logger().info(msg.data)

    def _bounded_duration(self, raw: Any) -> float:
        duration = float(raw)
        if not math.isfinite(duration):
            raise ValueError("duration must be finite")
        return self._clamp(duration, 0.05, float(self.get_parameter("max_base_command_duration_sec").value))

    def _positive(self, raw: Any, *, fallback: float) -> float:
        try:
            value = float(raw)
        except Exception:
            return fallback
        if not math.isfinite(value) or value <= 0.0:
            return fallback
        return value

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        publisher.publish(msg)

    def _publish_string(self, publisher, payload: str) -> None:
        msg = String()
        msg.data = payload
        publisher.publish(msg)

    def _redacted(self, payload: Any) -> Any:
        if isinstance(payload, dict):
            result = {}
            for key, value in payload.items():
                lowered = str(key).lower()
                if any(token in lowered for token in ("key", "token", "secret", "password")):
                    result[key] = "<redacted>"
                else:
                    result[key] = self._redacted(value)
            return result
        if isinstance(payload, list):
            return [self._redacted(value) for value in payload]
        return payload


def main() -> None:
    rclpy.init()
    node = AgentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._safe_stop("shutdown")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
