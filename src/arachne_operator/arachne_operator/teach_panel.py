from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass, fields, field
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics


DEFAULT_REAL_ARM_JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)

DEFAULT_ARM_HOME_JOINTS_DEG = "-88.28,3.40,116.60,103.48,88.33,-0.13"


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class TeachWaypoint:
    label: str
    stamp: str
    base_pose: list[float] = field(default_factory=list)
    arm_joints: list[float] = field(default_factory=list)
    tool_position: list[float] = field(default_factory=list)
    gripper: str = "open"
    kind: str = "pose"
    wait_sec: float = 0.0
    base_motion: list[dict] = field(default_factory=list)


def _angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _yaw_from_odom(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_names(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def _waypoint_from_dict(data: dict) -> TeachWaypoint:
    known = {item.name for item in fields(TeachWaypoint)}
    filtered = {key: value for key, value in data.items() if key in known}
    return TeachWaypoint(**filtered)


class TeachPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("arachne_teach_panel")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "arm_follow_joint_trajectory_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter(
            "legacy_arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory"
        )
        self.declare_parameter("arm_state_joint_names", ",".join(DEFAULT_REAL_ARM_JOINTS))
        self.declare_parameter("arm_command_joint_names", ",".join(DEFAULT_REAL_ARM_JOINTS))
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("base_linear_speed", 0.08)
        self.declare_parameter("base_angular_speed", 0.30)
        self.declare_parameter("base_replay_linear_speed", 0.20)
        self.declare_parameter("base_replay_angular_speed", 0.24)
        self.declare_parameter("base_position_tolerance", 0.02)
        self.declare_parameter("base_yaw_tolerance_deg", 2.0)
        self.declare_parameter("base_manual_publish_rate", 12.0)
        self.declare_parameter("base_motion_max_segment_sec", 20.0)
        self.declare_parameter("arm_jog_step_m", 0.006)
        self.declare_parameter("arm_jog_duration_sec", 0.30)
        self.declare_parameter("arm_rotate_step_rad", math.radians(1.5))
        self.declare_parameter("arm_rotate_duration_sec", 0.30)
        self.declare_parameter("arm_joint_step_rad", math.radians(1.0))
        self.declare_parameter("arm_hold_period_sec", 0.30)
        self.declare_parameter("arm_waypoint_duration_sec", 3.75)
        self.declare_parameter("arm_home_joints_deg", DEFAULT_ARM_HOME_JOINTS_DEG)
        self.declare_parameter("arm_goal_tolerance", 0.04)
        self.declare_parameter("arm_position_tolerance", 0.006)
        self.declare_parameter("arm_ik_damping", 0.08)
        self.declare_parameter("arm_ik_max_iterations", 180)
        self.declare_parameter("arm_ik_max_step", 0.05)
        self.declare_parameter("arm_jog_max_joint_delta", 0.25)
        self.declare_parameter("arm_target_max_joint_delta", 1.2)
        self.declare_parameter("aubo_teach_command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("aubo_teach_exit_wait_sec", 8.0)
        self.declare_parameter("replay_settle_sec", 0.2)
        self.declare_parameter("recording_dir", "recordings/teach")

        self.arm_state_joint_names = _parse_names(str(self.get_parameter("arm_state_joint_names").value))
        self.arm_command_joint_names = _parse_names(
            str(self.get_parameter("arm_command_joint_names").value)
        )
        if len(self.arm_state_joint_names) != 6 or len(self.arm_command_joint_names) != 6:
            raise ValueError("arm_state_joint_names and arm_command_joint_names must list 6 joints")

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        gripper_topic = str(self.get_parameter("gripper_command_topic").value)
        aubo_teach_topic = str(self.get_parameter("aubo_teach_command_topic").value)
        arm_topics = []
        for topic in (
            str(self.get_parameter("arm_trajectory_topic").value),
            str(self.get_parameter("legacy_arm_trajectory_topic").value),
        ):
            if topic and topic not in arm_topics:
                arm_topics.append(topic)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self.aubo_teach_pub = self.create_publisher(String, aubo_teach_topic, 10)
        self.arm_publishers = [self.create_publisher(JointTrajectory, topic, 10) for topic in arm_topics]
        self.status_pub = self.create_publisher(String, "/arachne/teach/status", 10)
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_follow_joint_trajectory_action").value),
        )

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)
        self.create_subscription(String, "/arachne/hardware/base_status", self._status_callback("Base"), 10)
        self.create_subscription(String, "/arachne/hardware/aubo_status", self._status_callback("Aubo"), 10)
        self.create_subscription(
            String, "/arachne/hardware/gripper_status", self._gripper_status_callback, 10
        )

        self.kinematics = AuboI5Kinematics()
        self.lock = threading.Lock()
        self.manual_arm_command_lock = threading.Lock()
        self.base_pose: Pose2D | None = None
        self.base_motion_segments: list[dict] = []
        self.active_base_motion: dict | None = None
        self.manual_base_velocity: tuple[float, float] | None = None
        self.current_arm: dict[str, float] = {}
        self.tool_position: tuple[float, float, float] | None = None
        self.gripper_state = "open"
        self.hardware_status = {"Base": "waiting", "Aubo": "waiting", "Gripper": "waiting"}
        self.aubo_teach_gate_active = False
        self.aubo_teach_ready_event = threading.Event()
        self.aubo_teach_ready_event.set()
        self.last_status = "ready"
        self.log_lines: list[str] = []
        self.cancel_event = threading.Event()
        self.replay_thread: threading.Thread | None = None
        self._active_goal_handle = None
        publish_rate = max(float(self.get_parameter("base_manual_publish_rate").value), 1.0)
        self.create_timer(1.0 / publish_rate, self._publish_manual_base_velocity)
        self._status("ready")

    def _odom_callback(self, msg: Odometry) -> None:
        pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, _yaw_from_odom(msg))
        with self.lock:
            self.base_pose = pose

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        updates: dict[str, float] = {}
        for name in self.arm_state_joint_names:
            candidates = [name]
            if name.startswith("aubo_"):
                candidates.append(name.removeprefix("aubo_"))
            else:
                candidates.append(f"aubo_{name}")
            for candidate in candidates:
                if candidate in positions:
                    updates[name] = float(positions[candidate])
                    break

        with self.lock:
            self.current_arm.update(updates)
            vector = self._current_arm_vector_locked()
            if vector is not None:
                position = self.kinematics.fk(np.array(vector, dtype=float))[:3, 3]
                self.tool_position = tuple(float(value) for value in position)

    def _status_callback(self, key: str):
        def callback(msg: String) -> None:
            with self.lock:
                self.hardware_status[key] = msg.data
                if key == "Aubo":
                    self._update_aubo_teach_state_locked(msg.data)

        return callback

    def _update_aubo_teach_state_locked(self, status: str) -> None:
        data = status.strip().lower()
        if "teach on active" in data or "keeping ros teach gate active" in data:
            self.aubo_teach_gate_active = True
            self.aubo_teach_ready_event.clear()
        elif "teach off complete" in data:
            self.aubo_teach_gate_active = False
            self.aubo_teach_ready_event.set()

    def _gripper_status_callback(self, msg: String) -> None:
        data = msg.data.strip()
        first = data.split(":", 1)[0].strip().lower()
        with self.lock:
            self.hardware_status["Gripper"] = data
            if first in ("open", "close", "stop"):
                self.gripper_state = first

    def snapshot(self) -> dict[str, str]:
        with self.lock:
            base = self.base_pose
            tool = self.tool_position
            arm_ready = self._current_arm_vector_locked() is not None
            return {
                "base": (
                    f"x={base.x:.3f} y={base.y:.3f} yaw={math.degrees(base.yaw):.1f}deg"
                    if base
                    else "waiting"
                ),
                "tool": (
                    f"x={tool[0]:.3f} y={tool[1]:.3f} z={tool[2]:.3f}"
                    if tool
                    else "waiting"
                ),
                "arm": "ready" if arm_ready else "waiting",
                "gripper": self.gripper_state,
                "teach": "on" if self.aubo_teach_gate_active else "off",
                "status": self.last_status,
                **self.hardware_status,
            }

    def joint_snapshot(self) -> list[tuple[str, float]] | None:
        with self.lock:
            vector = self._current_arm_vector_locked()
            if vector is None:
                return None
            return list(zip(self.arm_state_joint_names, vector))

    def tool_snapshot(self) -> tuple[float, float, float] | None:
        with self.lock:
            return tuple(self.tool_position) if self.tool_position is not None else None

    def log_snapshot(self) -> list[str]:
        with self.lock:
            return list(self.log_lines)

    def recording_dir(self) -> Path:
        directory = Path(str(self.get_parameter("recording_dir").value))
        if not directory.is_absolute():
            directory = Path.cwd() / directory
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def set_base_velocity(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def drive_base_manual(self, direction: str) -> None:
        linear = float(self.get_parameter("base_linear_speed").value)
        angular = float(self.get_parameter("base_angular_speed").value)
        mapping = {
            "forward": (linear, 0.0),
            "back": (-linear, 0.0),
            "left": (0.0, angular),
            "right": (0.0, -angular),
            "stop": (0.0, 0.0),
        }
        vx, wz = mapping.get(direction, (0.0, 0.0))
        self._track_base_motion(direction, vx, wz)
        with self.lock:
            self.manual_base_velocity = None if direction == "stop" else (vx, wz)
        self.set_base_velocity(vx, wz)

    def stop_all(self) -> None:
        self.cancel_event.set()
        self.drive_base_manual("stop")
        self.publish_gripper("stop")
        self.stop_arm_motion()
        self._status("stop requested")

    def stop_arm_motion(self) -> None:
        self.cancel_event.set()
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - best effort stop path.
                self.get_logger().warning(f"arm cancel failed: {exc}")
            self._active_goal_handle = None
        self._status("arm stop requested")

    def hold_arm_current(self) -> None:
        self.stop_arm_motion()
        current = self._current_arm_vector()
        if current is None:
            return
        self.cancel_event.clear()
        self._send_arm_positions(current, 0.25, "hold current", wait=False)

    def publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        with self.lock:
            if command in ("open", "close", "stop"):
                self.gripper_state = command
        self._status(f"gripper {command}")

    def set_aubo_teach(self, enabled: bool) -> None:
        if enabled:
            self.cancel_event.set()
            self.drive_base_manual("stop")
            goal_handle = self._active_goal_handle
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:  # pragma: no cover - best effort stop path.
                    self.get_logger().warning(f"arm cancel before teach failed: {exc}")
        else:
            self.cancel_event.clear()
        with self.lock:
            self.aubo_teach_gate_active = enabled
            if enabled:
                self.aubo_teach_ready_event.clear()
        command = "teach_on" if enabled else "teach_off"
        self._publish_aubo_teach_command(command)
        self._status(f"aubo {command}")

    def _publish_aubo_teach_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.aubo_teach_pub.publish(msg)

    def _aubo_teach_gate_may_be_active(self) -> bool:
        with self.lock:
            if self.aubo_teach_gate_active:
                return True
            status = self.hardware_status.get("Aubo", "").lower()
        return "teach on active" in status or "keeping ros teach gate active" in status

    def _ensure_aubo_motion_ready(self) -> bool:
        if not self._aubo_teach_gate_may_be_active():
            return True
        timeout = max(float(self.get_parameter("aubo_teach_exit_wait_sec").value), 0.0)
        self._status("aubo teach_off before replay")
        self._publish_aubo_teach_command("teach_off")
        if self.aubo_teach_ready_event.wait(timeout):
            return True
        with self.lock:
            status = self.hardware_status.get("Aubo", "unknown")
        self._status(f"aubo teach_off timeout before replay: {status}", warn=True)
        return False

    def jog_arm(self, axis: str, sign: float) -> None:
        self.cancel_event.clear()
        self._start_manual_arm_worker(lambda: self._jog_arm_worker(axis, sign))

    def _track_base_motion(self, direction: str, linear_x: float, angular_z: float) -> None:
        now = time.monotonic()
        with self.lock:
            if direction == "stop" or (abs(linear_x) < 1e-9 and abs(angular_z) < 1e-9):
                self._close_base_motion_locked(now)
                return
            self._close_base_motion_locked(now)
            self.active_base_motion = {
                "command": direction,
                "linear_x": float(linear_x),
                "angular_z": float(angular_z),
                "_start_pose": self._base_pose_values_locked(),
                "start_stamp": datetime.now().isoformat(timespec="seconds"),
                "_start_monotonic": now,
            }

    def _publish_manual_base_velocity(self) -> None:
        with self.lock:
            velocity = self.manual_base_velocity
        if velocity is None:
            return
        self.set_base_velocity(velocity[0], velocity[1])

    def _close_base_motion_locked(self, now: float) -> None:
        if self.active_base_motion is None:
            return
        active = dict(self.active_base_motion)
        start = float(active.get("_start_monotonic", now))
        duration = max(0.0, now - start)
        if duration >= 0.05:
            end_pose = self._base_pose_values_locked()
            segment = self._make_relative_base_segment(active, end_pose, duration)
            segment["end_stamp"] = datetime.now().isoformat(timespec="seconds")
            self.base_motion_segments.append(segment)
            self.last_status = self._describe_base_segment(segment)
        self.active_base_motion = None

    def _base_pose_values_locked(self) -> list[float]:
        if self.base_pose is None:
            return []
        return [self.base_pose.x, self.base_pose.y, self.base_pose.yaw]

    def _make_relative_base_segment(
        self, active: dict, end_pose: list[float], duration: float
    ) -> dict:
        command = str(active.get("command", "stop"))
        linear_x = float(active.get("linear_x", 0.0))
        angular_z = float(active.get("angular_z", 0.0))
        start_pose = active.get("_start_pose", [])
        source = "timed"
        signed_distance = linear_x * duration
        signed_angle = angular_z * duration

        if len(start_pose) == 3 and len(end_pose) == 3:
            dx = float(end_pose[0]) - float(start_pose[0])
            dy = float(end_pose[1]) - float(start_pose[1])
            start_yaw = float(start_pose[2])
            signed_distance = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            signed_angle = _angle_diff(float(end_pose[2]), start_yaw)
            source = "odom"

        if command in ("forward", "back"):
            if abs(signed_distance) < 1e-5:
                signed_distance = linear_x * duration
                source = "timed"
            action = "forward" if signed_distance >= 0.0 else "back"
            return {
                "type": "linear",
                "action": action,
                "distance_m": abs(float(signed_distance)),
                "signed_distance_m": float(signed_distance),
                "duration_sec": float(duration),
                "linear_x": linear_x,
                "source": source,
                "start_stamp": active.get("start_stamp", ""),
            }

        if command in ("left", "right"):
            if abs(signed_angle) < 1e-5:
                signed_angle = angular_z * duration
                source = "timed"
            action = "left" if signed_angle >= 0.0 else "right"
            return {
                "type": "angular",
                "action": action,
                "angle_rad": abs(float(signed_angle)),
                "signed_angle_rad": float(signed_angle),
                "duration_sec": float(duration),
                "angular_z": angular_z,
                "source": source,
                "start_stamp": active.get("start_stamp", ""),
            }

        return {
            "type": "timed",
            "action": command,
            "duration_sec": float(duration),
            "linear_x": linear_x,
            "angular_z": angular_z,
            "source": source,
            "start_stamp": active.get("start_stamp", ""),
        }

    def _describe_base_segment(self, segment: dict) -> str:
        action = str(segment.get("action", "base"))
        if segment.get("type") == "linear":
            return f"base recorded: {action} {float(segment.get('distance_m', 0.0)):.3f} m"
        if segment.get("type") == "angular":
            angle = math.degrees(float(segment.get("angle_rad", 0.0)))
            return f"base recorded: {action} {angle:.1f} deg"
        return f"base recorded: {action} {float(segment.get('duration_sec', 0.0)):.1f} s"

    def _jog_arm_worker(self, axis: str, sign: float) -> None:
        q_start = self._current_arm_vector()
        if q_start is None:
            self._status("arm jog skipped: no joint state", warn=True)
            return
        directions = {
            "x": np.array([1.0, 0.0, 0.0], dtype=float),
            "y": np.array([0.0, 1.0, 0.0], dtype=float),
            "z": np.array([0.0, 0.0, 1.0], dtype=float),
        }
        direction = directions[axis] * float(sign)
        step = float(self.get_parameter("arm_jog_step_m").value)
        target = self.kinematics.fk(np.array(q_start, dtype=float))[:3, 3] + direction * step
        ok, q_target, error, iterations = self.kinematics.solve_position(
            np.array(q_start, dtype=float),
            target,
            tolerance=float(self.get_parameter("arm_position_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_target - np.array(q_start, dtype=float))))
        if not ok:
            self._status(f"arm jog IK failed: error={error:.4f} iterations={iterations}", warn=True)
            return
        if max_delta > float(self.get_parameter("arm_jog_max_joint_delta").value):
            self._status(f"arm jog blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        self._send_arm_positions(
            [float(value) for value in q_target],
            float(self.get_parameter("arm_jog_duration_sec").value),
            f"jog {axis}",
            wait=False,
        )

    def jog_arm_rotation(self, axis: str, sign: float) -> None:
        self.cancel_event.clear()
        self._start_manual_arm_worker(lambda: self._jog_arm_rotation_worker(axis, sign))

    def _jog_arm_rotation_worker(self, axis: str, sign: float) -> None:
        q_start = self._current_arm_vector()
        if q_start is None:
            self._status("arm rotation jog skipped: no joint state", warn=True)
            return
        wrist_indices = {"rx": 3, "ry": 4, "rz": 5}
        if axis not in wrist_indices:
            self._status(f"arm rotation jog skipped: unknown axis {axis}", warn=True)
            return

        step = float(self.get_parameter("arm_rotate_step_rad").value)
        q_target = list(q_start)
        q_target[wrist_indices[axis]] += float(sign) * step
        max_delta = max(abs(target - start) for target, start in zip(q_target, q_start))
        if max_delta > float(self.get_parameter("arm_jog_max_joint_delta").value):
            self._status(f"arm rotation jog blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        self._send_arm_positions(
            q_target,
            float(self.get_parameter("arm_rotate_duration_sec").value),
            f"rotate {axis}",
            wait=False,
        )

    def jog_arm_joint(self, index: int, sign: float) -> None:
        self.cancel_event.clear()
        self._start_manual_arm_worker(lambda: self._jog_arm_joint_worker(index, sign))

    def _jog_arm_joint_worker(self, index: int, sign: float) -> None:
        q_start = self._current_arm_vector()
        if q_start is None:
            self._status("joint jog skipped: no joint state", warn=True)
            return
        if index < 0 or index >= len(q_start):
            self._status(f"joint jog skipped: invalid joint {index + 1}", warn=True)
            return
        step = float(self.get_parameter("arm_joint_step_rad").value)
        q_target = list(q_start)
        q_target[index] += float(sign) * step
        max_delta = max(abs(target - start) for target, start in zip(q_target, q_start))
        if max_delta > float(self.get_parameter("arm_jog_max_joint_delta").value):
            self._status(f"joint jog blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        self._send_arm_positions(
            q_target,
            float(self.get_parameter("arm_rotate_duration_sec").value),
            f"jog J{index + 1}",
            wait=False,
        )

    def move_arm_to_joints_degrees(self, degrees: list[float]) -> None:
        self.cancel_event.clear()
        self._start_worker(lambda: self._move_arm_to_joints_worker(degrees))

    def home_joints_degrees(self) -> list[float]:
        text = str(self.get_parameter("arm_home_joints_deg").value)
        values = [float(item.strip()) for item in text.replace(";", ",").split(",") if item.strip()]
        if len(values) != 6:
            raise ValueError("arm_home_joints_deg must contain 6 comma-separated degrees")
        return values

    def move_arm_home(self) -> None:
        try:
            target = self.home_joints_degrees()
        except Exception as exc:
            self._status(f"home skipped: {exc}", warn=True)
            return
        self.move_arm_to_joints_degrees(target)

    def _move_arm_to_joints_worker(self, degrees: list[float]) -> None:
        if len(degrees) != 6:
            self._status("joint target skipped: expected 6 joint angles", warn=True)
            return
        target = [math.radians(float(value)) for value in degrees]
        self._send_arm_positions(
            target,
            float(self.get_parameter("arm_waypoint_duration_sec").value),
            "joint target",
            wait=False,
        )

    def move_tool_to_position(self, position: list[float]) -> None:
        self.cancel_event.clear()
        self._start_worker(lambda: self._move_tool_to_position_worker(position))

    def _move_tool_to_position_worker(self, position: list[float]) -> None:
        if len(position) != 3:
            self._status("TCP target skipped: expected x,y,z", warn=True)
            return
        q_start = self._current_arm_vector()
        if q_start is None:
            self._status("TCP target skipped: no joint state", warn=True)
            return
        target = np.array([float(value) for value in position], dtype=float)
        ok, q_target, error, iterations = self.kinematics.solve_position(
            np.array(q_start, dtype=float),
            target,
            tolerance=float(self.get_parameter("arm_position_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_target - np.array(q_start, dtype=float))))
        if not ok:
            self._status(f"TCP target IK failed: error={error:.4f} iterations={iterations}", warn=True)
            return
        if max_delta > float(self.get_parameter("arm_target_max_joint_delta").value):
            self._status(f"TCP target blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        self._send_arm_positions(
            [float(value) for value in q_target],
            float(self.get_parameter("arm_waypoint_duration_sec").value),
            "TCP target",
            wait=False,
        )

    def record_waypoint(self, label: str) -> TeachWaypoint:
        with self.lock:
            self._close_base_motion_locked(time.monotonic())
            base = self.base_pose
            arm = self._current_arm_vector_locked()
            tool = self.tool_position
            gripper = self.gripper_state
            if base is None:
                raise RuntimeError("missing /odom")
            if arm is None or tool is None:
                raise RuntimeError("missing Aubo /joint_states")
            base_motion = [dict(item) for item in self.base_motion_segments]
            self.base_motion_segments.clear()
        clean_label = label.strip() or f"wp_{datetime.now().strftime('%H%M%S')}"
        waypoint = TeachWaypoint(
            label=clean_label,
            stamp=datetime.now().isoformat(timespec="seconds"),
            base_pose=[],
            arm_joints=[float(value) for value in arm],
            tool_position=[float(value) for value in tool],
            gripper=gripper,
            kind="pose",
            base_motion=base_motion,
        )
        self._status(f"recorded {clean_label}")
        return waypoint

    def record_wait(self, label: str, seconds: float) -> TeachWaypoint:
        wait_sec = max(float(seconds), 0.0)
        clean_label = label.strip() or f"wait_{datetime.now().strftime('%H%M%S')}"
        waypoint = TeachWaypoint(
            label=clean_label,
            stamp=datetime.now().isoformat(timespec="seconds"),
            kind="wait",
            wait_sec=wait_sec,
        )
        self._status(f"recorded wait {clean_label}: {wait_sec:.1f}s")
        return waypoint

    def replay(self, waypoints: list[TeachWaypoint]) -> None:
        if not waypoints:
            self._status("replay skipped: no waypoints", warn=True)
            return
        if self.replay_thread is not None and self.replay_thread.is_alive():
            self._status("replay already running", warn=True)
            return
        self.drive_base_manual("stop")
        self.cancel_event.clear()
        self.replay_thread = threading.Thread(
            target=self._replay_worker,
            args=([TeachWaypoint(**asdict(item)) for item in waypoints],),
            daemon=True,
        )
        self.replay_thread.start()

    def _replay_worker(self, waypoints: list[TeachWaypoint]) -> None:
        self._status(f"replay started: {len(waypoints)} waypoints")
        try:
            if not self._ensure_aubo_motion_ready():
                raise RuntimeError("Aubo teach mode is still active")
            for index, waypoint in enumerate(waypoints, start=1):
                if self.cancel_event.is_set():
                    break
                self._status(f"replay {index}/{len(waypoints)}: {waypoint.label}")
                if waypoint.kind == "wait":
                    self.set_base_velocity(0.0, 0.0)
                    self._status(f"wait {waypoint.wait_sec:.1f}s: {waypoint.label}")
                    self._sleep(float(waypoint.wait_sec))
                    continue

                if waypoint.base_motion:
                    self._replay_base_motion(waypoint.base_motion, waypoint.label)
                if waypoint.gripper in ("open", "close"):
                    self.publish_gripper(waypoint.gripper)
                if not self._send_arm_positions(
                    waypoint.arm_joints,
                    float(self.get_parameter("arm_waypoint_duration_sec").value),
                    waypoint.label,
                    wait=True,
                ):
                    raise RuntimeError(f"arm waypoint failed: {waypoint.label}")
                self._sleep(float(self.get_parameter("replay_settle_sec").value))
            self.set_base_velocity(0.0, 0.0)
            self._status("replay complete" if not self.cancel_event.is_set() else "replay stopped")
        except Exception as exc:
            self.set_base_velocity(0.0, 0.0)
            self._status(f"replay failed: {exc}", warn=True)

    def _replay_base_motion(self, segments: list[dict], label: str) -> None:
        max_duration = float(self.get_parameter("base_motion_max_segment_sec").value)
        for index, segment in enumerate(segments, start=1):
            if self.cancel_event.is_set():
                break
            normalized = self._normalize_base_motion_segment(segment)
            motion_type = normalized.get("type")
            if motion_type == "linear":
                distance = float(normalized.get("signed_distance_m", 0.0))
                self._status(
                    f"base motion {index}/{len(segments)} for {label}: "
                    f"{normalized.get('action', 'linear')} {abs(distance):.3f}m"
                )
                segment_speed = abs(float(normalized.get("linear_x", 0.0)))
                self._drive_distance(
                    distance,
                    speed_override=segment_speed if segment_speed > 0.0 else None,
                )
            elif motion_type == "angular":
                angle = float(normalized.get("signed_angle_rad", 0.0))
                self._status(
                    f"base motion {index}/{len(segments)} for {label}: "
                    f"{normalized.get('action', 'angular')} {abs(math.degrees(angle)):.1f}deg"
                )
                self._turn_relative(angle)
            else:
                duration = max(0.0, min(float(normalized.get("duration_sec", 0.0)), max_duration))
                linear_x = float(normalized.get("linear_x", 0.0))
                angular_z = float(normalized.get("angular_z", 0.0))
                self._status(
                    f"base motion {index}/{len(segments)} for {label}: "
                    f"vx={linear_x:.3f} wz={angular_z:.3f} t={duration:.1f}s"
                )
                deadline = time.monotonic() + duration
                while not self.cancel_event.is_set() and time.monotonic() < deadline:
                    self.set_base_velocity(linear_x, angular_z)
                    time.sleep(0.05)
            self.set_base_velocity(0.0, 0.0)
            self._sleep(0.1)

    def _normalize_base_motion_segment(self, segment: dict) -> dict:
        if segment.get("type") == "linear":
            normalized = dict(segment)
            if "signed_distance_m" not in normalized:
                action = str(normalized.get("action", "forward"))
                distance = abs(float(normalized.get("distance_m", 0.0)))
                normalized["signed_distance_m"] = -distance if action == "back" else distance
            normalized["distance_m"] = abs(float(normalized.get("signed_distance_m", 0.0)))
            normalized["action"] = (
                "forward" if float(normalized.get("signed_distance_m", 0.0)) >= 0.0 else "back"
            )
            return normalized
        if segment.get("type") == "angular":
            normalized = dict(segment)
            if "signed_angle_rad" not in normalized:
                action = str(normalized.get("action", "left"))
                angle = abs(float(normalized.get("angle_rad", 0.0)))
                normalized["signed_angle_rad"] = -angle if action == "right" else angle
            normalized["angle_rad"] = abs(float(normalized.get("signed_angle_rad", 0.0)))
            normalized["action"] = (
                "left" if float(normalized.get("signed_angle_rad", 0.0)) >= 0.0 else "right"
            )
            return normalized
        if segment.get("type") == "timed":
            return dict(segment)

        command = str(segment.get("command", ""))
        duration = float(segment.get("duration_sec", 0.0))
        linear_x = float(segment.get("linear_x", 0.0))
        angular_z = float(segment.get("angular_z", 0.0))
        start_pose = segment.get("start_pose", [])
        end_pose = segment.get("end_pose", [])

        active = {
            "command": command,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "_start_pose": start_pose,
            "start_stamp": segment.get("start_stamp", ""),
        }
        return self._make_relative_base_segment(active, end_pose, duration)

    def _drive_base_to_pose(self, target: Pose2D) -> None:
        current = self._current_base_pose()
        dx = target.x - current.x
        dy = target.y - current.y
        distance = math.hypot(dx, dy)
        if distance > float(self.get_parameter("base_position_tolerance").value):
            heading = math.atan2(dy, dx)
            direction = 1.0
            if abs(_angle_diff(heading, current.yaw)) > math.pi / 2.0:
                heading = math.atan2(math.sin(heading + math.pi), math.cos(heading + math.pi))
                direction = -1.0
            self._turn_to_yaw(heading)
            self._drive_distance(direction * distance)
        self._turn_to_yaw(target.yaw)
        self.set_base_velocity(0.0, 0.0)

    def _turn_relative(self, angle: float) -> None:
        current = self._current_base_pose()
        target = math.atan2(math.sin(current.yaw + angle), math.cos(current.yaw + angle))
        self._turn_to_yaw(target)

    def _turn_to_yaw(self, target_yaw: float) -> None:
        tolerance = math.radians(float(self.get_parameter("base_yaw_tolerance_deg").value))
        speed = abs(float(self.get_parameter("base_replay_angular_speed").value))
        initial_error = abs(_angle_diff(target_yaw, self._current_base_pose().yaw))
        deadline = time.monotonic() + max(12.0, initial_error / max(speed, 1e-3) + 8.0)
        while not self.cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose()
            error = _angle_diff(target_yaw, current.yaw)
            if abs(error) <= tolerance:
                break
            twist = Twist()
            twist.angular.z = math.copysign(speed, error)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.set_base_velocity(0.0, 0.0)

    def _drive_distance(self, distance: float, speed_override: float | None = None) -> None:
        start = self._current_base_pose()
        heading = np.array([math.cos(start.yaw), math.sin(start.yaw)], dtype=float)
        tolerance = float(self.get_parameter("base_position_tolerance").value)
        speed = (
            abs(float(speed_override))
            if speed_override is not None
            else abs(float(self.get_parameter("base_replay_linear_speed").value))
        )
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.monotonic() + abs(distance) / max(speed, 1e-3) + 8.0
        while not self.cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose()
            offset = np.array([current.x - start.x, current.y - start.y], dtype=float)
            progress = float(offset @ heading)
            if sign * progress >= abs(distance) - tolerance:
                break
            twist = Twist()
            twist.linear.x = sign * speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.set_base_velocity(0.0, 0.0)

    def _send_arm_positions(
        self, positions: list[float], duration: float, label: str, *, wait: bool
    ) -> bool:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_command_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        if self.arm_action_client.wait_for_server(timeout_sec=0.25):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal_future = self.arm_action_client.send_goal_async(goal)
            if not self._wait_future(goal_future, 3.0):
                self._status(f"arm action timeout: {label}", warn=True)
                return False
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._status(f"arm action rejected: {label}", warn=True)
                return False
            self._active_goal_handle = goal_handle
            if not wait:
                self._status(f"arm command sent: {label}")
                return True
            result_future = goal_handle.get_result_async()
            ok = self._wait_future(result_future, duration + 8.0)
            self._active_goal_handle = None
            if not ok:
                self._status(f"arm action result timeout: {label}", warn=True)
                return False
            result = result_future.result()
            error_code = result.result.error_code if result else -999
            if error_code != 0:
                self._status(f"arm action failed: {label} code={error_code}", warn=True)
                return False
            if not self._wait_for_arm_target(
                [float(value) for value in positions],
                float(self.get_parameter("arm_goal_tolerance").value),
                max(2.0, duration + 2.0),
            ):
                self._status(f"arm feedback did not reach target: {label}", warn=True)
                return False
            return True

        for publisher in self.arm_publishers:
            publisher.publish(trajectory)
        self._status(f"arm trajectory published: {label}")
        if wait:
            self._sleep(duration)
        return True

    def _wait_for_arm_target(
        self, target: list[float], tolerance: float, timeout_sec: float
    ) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        best_error = float("inf")
        while time.monotonic() < deadline and not self.cancel_event.is_set():
            current = self._current_arm_vector()
            if current is not None and len(current) == len(target):
                error = max(abs(a - b) for a, b in zip(current, target))
                best_error = min(best_error, error)
                if error <= tolerance:
                    return True
            time.sleep(0.05)
        self.get_logger().warning(f"arm target feedback timeout: best_error={best_error:.4f}")
        return False

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline and not self.cancel_event.is_set():
            time.sleep(0.02)
        return future.done()

    def _current_base_pose(self) -> Pose2D:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not self.cancel_event.is_set():
            with self.lock:
                if self.base_pose is not None:
                    return Pose2D(self.base_pose.x, self.base_pose.y, self.base_pose.yaw)
            time.sleep(0.05)
        raise TimeoutError("missing /odom")

    def _current_arm_vector(self) -> list[float] | None:
        with self.lock:
            vector = self._current_arm_vector_locked()
            return list(vector) if vector is not None else None

    def _current_arm_vector_locked(self) -> list[float] | None:
        if not all(name in self.current_arm for name in self.arm_state_joint_names):
            return None
        return [self.current_arm[name] for name in self.arm_state_joint_names]

    def _sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(seconds, 0.0)
        while time.monotonic() < deadline and not self.cancel_event.is_set():
            time.sleep(0.05)

    def _start_worker(self, target) -> None:
        threading.Thread(target=target, daemon=True).start()

    def _start_manual_arm_worker(self, target) -> None:
        def runner() -> None:
            if not self.manual_arm_command_lock.acquire(blocking=False):
                return
            try:
                target()
            finally:
                self.manual_arm_command_lock.release()

        self._start_worker(runner)

    def clear_base_motion_history(self) -> None:
        with self.lock:
            self.base_motion_segments.clear()
            self.active_base_motion = None
            self.manual_base_velocity = None

    def pending_base_motion_count(self) -> int:
        with self.lock:
            return len(self.base_motion_segments) + (1 if self.active_base_motion is not None else 0)

    def update_motion_settings(
        self,
        *,
        base_linear_speed: float,
        base_angular_speed: float,
        arm_jog_step_m: float,
        arm_rotate_step_deg: float,
        arm_joint_step_deg: float,
        arm_hold_period_sec: float,
        waypoint_duration_sec: float,
        arm_home_joints_deg: str,
    ) -> None:
        if base_linear_speed <= 0.0 or base_angular_speed <= 0.0:
            raise ValueError("base speeds must be positive")
        if arm_jog_step_m <= 0.0 or arm_rotate_step_deg <= 0.0:
            raise ValueError("arm jog steps must be positive")
        if arm_joint_step_deg <= 0.0:
            raise ValueError("joint jog step must be positive")
        if arm_hold_period_sec <= 0.0:
            raise ValueError("hold period must be positive")
        if waypoint_duration_sec <= 0.0:
            raise ValueError("waypoint duration must be positive")
        home_values = [
            float(item.strip())
            for item in str(arm_home_joints_deg).replace(";", ",").split(",")
            if item.strip()
        ]
        if len(home_values) != 6:
            raise ValueError("home joints must contain 6 comma-separated degrees")
        self.set_parameters(
            [
                Parameter("base_linear_speed", Parameter.Type.DOUBLE, float(base_linear_speed)),
                Parameter("base_angular_speed", Parameter.Type.DOUBLE, float(base_angular_speed)),
                Parameter("arm_jog_step_m", Parameter.Type.DOUBLE, float(arm_jog_step_m)),
                Parameter(
                    "arm_rotate_step_rad",
                    Parameter.Type.DOUBLE,
                    math.radians(float(arm_rotate_step_deg)),
                ),
                Parameter(
                    "arm_joint_step_rad",
                    Parameter.Type.DOUBLE,
                    math.radians(float(arm_joint_step_deg)),
                ),
                Parameter(
                    "arm_hold_period_sec",
                    Parameter.Type.DOUBLE,
                    float(arm_hold_period_sec),
                ),
                Parameter(
                    "arm_waypoint_duration_sec",
                    Parameter.Type.DOUBLE,
                    float(waypoint_duration_sec),
                ),
                Parameter(
                    "arm_home_joints_deg",
                    Parameter.Type.STRING,
                    ",".join(f"{value:.2f}" for value in home_values),
                ),
            ]
        )
        self._status(
            "motion settings applied: "
            f"base={base_linear_speed:.3f}m/s {base_angular_speed:.3f}rad/s, "
            f"tcp={arm_jog_step_m:.3f}m, rot={arm_rotate_step_deg:.1f}deg, "
            f"joint={arm_joint_step_deg:.1f}deg, hold={arm_hold_period_sec:.2f}s"
        )

    def _status(self, text: str, *, warn: bool = False) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.last_status = text
            self.log_lines.append(f"{stamp} {'WARN' if warn else 'INFO'} {text}")
            self.log_lines = self.log_lines[-300:]
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


class TeachPanelApp:
    def __init__(self, node: TeachPanelNode) -> None:
        self.node = node
        self.waypoints: list[TeachWaypoint] = []
        self.root = tk.Tk()
        self.root.title("Arachne Scope")
        self.root.geometry("1180x760")
        self.root.minsize(980, 640)
        self.status_vars: dict[str, tk.StringVar] = {}
        self.label_var = tk.StringVar(value="wp_1")
        self.wait_var = tk.StringVar(value="2.0")
        self.file_var = tk.StringVar(value="unsaved")
        self.program_var = tk.StringVar(value="0 nodes")
        self.base_linear_var = tk.StringVar(value=f"{float(node.get_parameter('base_linear_speed').value):.3f}")
        self.base_angular_var = tk.StringVar(
            value=f"{float(node.get_parameter('base_angular_speed').value):.3f}"
        )
        self.arm_step_var = tk.StringVar(value=f"{float(node.get_parameter('arm_jog_step_m').value):.3f}")
        self.arm_rotate_var = tk.StringVar(
            value=f"{math.degrees(float(node.get_parameter('arm_rotate_step_rad').value)):.1f}"
        )
        self.arm_joint_step_var = tk.StringVar(
            value=f"{math.degrees(float(node.get_parameter('arm_joint_step_rad').value)):.1f}"
        )
        self.arm_hold_period_var = tk.StringVar(
            value=f"{float(node.get_parameter('arm_hold_period_sec').value):.2f}"
        )
        self.waypoint_duration_var = tk.StringVar(
            value=f"{float(node.get_parameter('arm_waypoint_duration_sec').value):.2f}"
        )
        self.home_joints_var = tk.StringVar(
            value=str(node.get_parameter("arm_home_joints_deg").value)
        )
        self.joint_target_vars = [tk.StringVar(value="") for _ in range(6)]
        self.tool_target_vars = {axis: tk.StringVar(value="") for axis in ("x", "y", "z")}
        self.move_status_vars: dict[str, tk.StringVar] = {}
        self.move_joint_vars: list[tk.StringVar] = []
        self._arm_hold_after: str | None = None
        self._arm_hold_callback = None
        self.listbox: tk.Listbox | None = None
        self.log_text: tk.Text | None = None
        self.joint_tree: ttk.Treeview | None = None
        self._build()
        self._refresh()

    def _build(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Top.TFrame", background="#202833")
        style.configure("TopTitle.TLabel", background="#202833", foreground="#f6f8fb", font=("TkDefaultFont", 16, "bold"))
        style.configure("Top.TLabel", background="#202833", foreground="#f6f8fb")
        style.configure("State.TLabel", font=("TkDefaultFont", 10, "bold"))
        style.configure("Danger.TButton", foreground="#8a1f11")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self._build_top_bar()

        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew")
        self._build_home_tab()
        self._build_move_tab()
        self._build_program_tab()
        self._build_config_tab()
        self._build_log_tab()

    def _build_top_bar(self) -> None:
        top = ttk.Frame(self.root, style="Top.TFrame", padding=(12, 8))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(4, weight=1)
        ttk.Label(top, text="Arachne Scope", style="TopTitle.TLabel").grid(
            row=0, column=0, rowspan=2, sticky="w", padx=(0, 18)
        )
        for column, key in enumerate(("Aubo", "Base", "Gripper"), start=1):
            var = tk.StringVar(value="waiting")
            self.status_vars[key] = var
            ttk.Label(top, text=key, style="Top.TLabel").grid(row=0, column=column, sticky="w", padx=8)
            ttk.Label(top, textvariable=var, style="Top.TLabel", width=22).grid(
                row=1, column=column, sticky="w", padx=8
            )
        status_var = tk.StringVar(value="ready")
        self.status_vars["status"] = status_var
        ttk.Label(top, textvariable=status_var, style="Top.TLabel").grid(
            row=0, column=4, rowspan=2, sticky="ew", padx=(16, 8)
        )
        ttk.Button(top, text="Home", command=self.node.move_arm_home).grid(
            row=0, column=5, rowspan=2, padx=4
        )
        ttk.Button(top, text="Run", command=self._play).grid(row=0, column=6, rowspan=2, padx=4)
        ttk.Button(top, text="Stop", command=self.node.stop_all, style="Danger.TButton").grid(
            row=0, column=7, rowspan=2, padx=4
        )

    def _build_home_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Home")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)

        overview = ttk.LabelFrame(tab, text="Robot Status")
        overview.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        overview.columnconfigure(1, weight=1)
        for row, key in enumerate(("base", "tool", "arm", "gripper", "teach", "program")):
            ttk.Label(overview, text=key, style="State.TLabel", width=10).grid(
                row=row, column=0, sticky="w", padx=6, pady=4
            )
            var = tk.StringVar(value="waiting")
            self.status_vars[key] = var
            ttk.Label(overview, textvariable=var).grid(row=row, column=1, sticky="ew", padx=6, pady=4)

        quick = ttk.LabelFrame(tab, text="Quick Control")
        quick.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        for index, (text, command) in enumerate(
            (
                ("Teach On", lambda: self.node.set_aubo_teach(True)),
                ("Teach Off", lambda: self.node.set_aubo_teach(False)),
                ("Home", self.node.move_arm_home),
                ("Record", self._record),
                ("Replay", self._play),
                ("Open", lambda: self.node.publish_gripper("open")),
                ("Close", lambda: self.node.publish_gripper("close")),
                ("Stop All", self.node.stop_all),
            )
        ):
            ttk.Button(quick, text=text, command=command).grid(
                row=index // 3, column=index % 3, sticky="ew", padx=5, pady=5
            )
        for column in range(3):
            quick.columnconfigure(column, weight=1)

        joints = ttk.LabelFrame(tab, text="Monitor and Joint")
        joints.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        joints.rowconfigure(0, weight=1)
        joints.columnconfigure(0, weight=1)
        self.joint_tree = ttk.Treeview(joints, columns=("rad", "deg"), show="headings", height=8)
        self.joint_tree.heading("rad", text="rad")
        self.joint_tree.heading("deg", text="deg")
        self.joint_tree.column("rad", width=120, anchor="e")
        self.joint_tree.column("deg", width=120, anchor="e")
        self.joint_tree.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        live_log = ttk.LabelFrame(tab, text="Variables and Log")
        live_log.grid(row=1, column=1, sticky="nsew")
        live_log.rowconfigure(0, weight=1)
        live_log.columnconfigure(0, weight=1)
        text = tk.Text(live_log, height=10, width=60, state="disabled", wrap="word")
        text.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.home_log_text = text

    def _build_move_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Move")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)
        self._build_move_monitor(tab)
        left = ttk.Frame(tab)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        right = ttk.Frame(tab)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        self._build_arm_controls(left)
        self._build_joint_target_controls(left)
        target = ttk.Frame(right)
        target.grid(row=0, column=0, sticky="ew")
        hardware = ttk.Frame(right)
        hardware.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        self._build_tool_target_controls(target)
        self._build_base_controls(hardware)
        self._build_gripper_controls(hardware)

    def _build_move_monitor(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Realtime")
        frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        for column in range(4):
            frame.columnconfigure(column, weight=1)
        for column, key in enumerate(("base", "tool", "arm", "gripper")):
            ttk.Label(frame, text=key, style="State.TLabel").grid(
                row=0, column=column, sticky="w", padx=6, pady=(4, 0)
            )
            var = tk.StringVar(value="waiting")
            self.move_status_vars[key] = var
            ttk.Label(frame, textvariable=var).grid(row=1, column=column, sticky="w", padx=6, pady=(0, 4))
        for index in range(6):
            var = tk.StringVar(value=f"J{index + 1}: waiting")
            self.move_joint_vars.append(var)
            ttk.Label(frame, textvariable=var, width=23).grid(
                row=2 + index // 3, column=index % 3, sticky="w", padx=6, pady=2
            )

    def _build_program_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Program")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(2, weight=1)

        editor = ttk.LabelFrame(tab, text="Node Editor")
        editor.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        editor.columnconfigure(1, weight=1)
        ttk.Label(editor, text="Label").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(editor, textvariable=self.label_var).grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        ttk.Button(editor, text="Waypoint", command=self._record).grid(row=0, column=2, padx=5, pady=5)
        ttk.Label(editor, text="Wait s").grid(row=0, column=3, padx=5, pady=5)
        ttk.Entry(editor, textvariable=self.wait_var, width=8).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(editor, text="Wait", command=self._add_wait).grid(row=0, column=5, padx=5, pady=5)

        toolbar = ttk.Frame(tab)
        toolbar.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        for index, (text, command) in enumerate(
            (
                ("Update", self._update_selected),
                ("Duplicate", self._duplicate_selected),
                ("Delete", self._delete_selected),
                ("Clear", self._clear),
                ("Reset", self._reset),
                ("Save", self._save),
                ("Load", self._load),
                ("Run", self._play),
                ("Stop", self.node.stop_all),
            )
        ):
            ttk.Button(toolbar, text=text, command=command).grid(row=0, column=index, padx=3)

        program = ttk.LabelFrame(tab, text="Program Tree")
        program.grid(row=2, column=0, sticky="nsew")
        program.rowconfigure(0, weight=1)
        program.columnconfigure(0, weight=1)
        self.listbox = tk.Listbox(program, height=18, selectmode=tk.EXTENDED)
        self.listbox.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        scroll = ttk.Scrollbar(program, command=self.listbox.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.listbox.configure(yscrollcommand=scroll.set)
        ttk.Label(tab, textvariable=self.file_var).grid(row=3, column=0, sticky="w", pady=(6, 0))

    def _build_config_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Configure")
        tab.columnconfigure(0, weight=1)
        motion = ttk.LabelFrame(tab, text="Motion Parameters")
        motion.grid(row=0, column=0, sticky="ew")
        fields = (
            ("Base linear m/s", self.base_linear_var),
            ("Base angular rad/s", self.base_angular_var),
            ("TCP step m", self.arm_step_var),
            ("Wrist step deg", self.arm_rotate_var),
            ("Joint step deg", self.arm_joint_step_var),
            ("Hold period s", self.arm_hold_period_var),
            ("Waypoint duration s", self.waypoint_duration_var),
        )
        for row, (label, var) in enumerate(fields):
            ttk.Label(motion, text=label, width=20).grid(row=row, column=0, sticky="w", padx=6, pady=5)
            ttk.Entry(motion, textvariable=var, width=12).grid(row=row, column=1, sticky="w", padx=6, pady=5)
        home_row = len(fields)
        ttk.Label(motion, text="Home joints deg", width=20).grid(
            row=home_row, column=0, sticky="w", padx=6, pady=5
        )
        ttk.Entry(motion, textvariable=self.home_joints_var, width=52).grid(
            row=home_row, column=1, sticky="ew", padx=6, pady=5
        )
        ttk.Button(motion, text="Apply", command=self._apply_motion_settings).grid(
            row=home_row + 1, column=0, columnspan=2, sticky="ew", padx=6, pady=(8, 6)
        )
        motion.columnconfigure(1, weight=1)

        payload = ttk.LabelFrame(tab, text="Aubo Payload")
        payload.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(payload, text="Startup payload is configured by scripts/real_full_teach.sh.").grid(
            row=0, column=0, sticky="w", padx=6, pady=4
        )
        ttk.Label(payload, text="Current Jetson default: 2.5kg, CoG 0,0,0.").grid(
            row=1, column=0, sticky="w", padx=6, pady=4
        )

    def _build_log_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Log")
        tab.rowconfigure(0, weight=1)
        tab.columnconfigure(0, weight=1)
        self.log_text = tk.Text(tab, height=24, state="disabled", wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(tab, command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _build_base_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Base")
        frame.grid(row=0, column=0, sticky="ew")
        for column in range(3):
            frame.columnconfigure(column, weight=1)
        buttons = {
            "Forward": ("forward", 0, 1),
            "Back": ("back", 2, 1),
            "Left": ("left", 1, 0),
            "Right": ("right", 1, 2),
            "Stop": ("stop", 1, 1),
        }
        for text, (direction, row, column) in buttons.items():
            button = ttk.Button(frame, text=text)
            button.grid(row=row, column=column, padx=4, pady=4, sticky="ew")
            if direction == "stop":
                button.configure(command=lambda: self.node.drive_base_manual("stop"))
            else:
                button.bind("<ButtonPress-1>", lambda _event, d=direction: self._base_press(d))
                button.bind("<ButtonRelease-1>", lambda _event: self._base_release())

    def _build_arm_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Aubo Move / HandGuide")
        frame.grid(row=0, column=0, sticky="ew")
        for column in range(3):
            frame.columnconfigure(column, weight=1)
        for row, axis in enumerate(("x", "y", "z")):
            ttk.Label(frame, text=axis.upper()).grid(row=row, column=0, padx=4, pady=4)
            self._make_arm_hold_button(frame, "-", lambda a=axis: self.node.jog_arm(a, -1.0)).grid(
                row=row, column=1, padx=4, pady=4, sticky="ew"
            )
            self._make_arm_hold_button(frame, "+", lambda a=axis: self.node.jog_arm(a, 1.0)).grid(
                row=row, column=2, padx=4, pady=4, sticky="ew"
            )
        for row, axis in enumerate(("rx", "ry", "rz"), start=3):
            ttk.Label(frame, text=axis.upper()).grid(row=row, column=0, padx=4, pady=4)
            self._make_arm_hold_button(
                frame, "-", lambda a=axis: self.node.jog_arm_rotation(a, -1.0)
            ).grid(row=row, column=1, padx=4, pady=4, sticky="ew")
            self._make_arm_hold_button(
                frame, "+", lambda a=axis: self.node.jog_arm_rotation(a, 1.0)
            ).grid(row=row, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(frame, text="Teach On", command=lambda: self.node.set_aubo_teach(True)).grid(
            row=6, column=0, columnspan=2, padx=4, pady=(8, 4), sticky="ew"
        )
        ttk.Button(frame, text="Teach Off", command=lambda: self.node.set_aubo_teach(False)).grid(
            row=6, column=2, padx=4, pady=(8, 4), sticky="ew"
        )

    def _build_joint_target_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Joint Jog / Target")
        frame.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for column in range(5):
            frame.columnconfigure(column, weight=1 if column in (2, 4) else 0)
        ttk.Label(frame, text="Joint").grid(row=0, column=0, padx=4, pady=3)
        ttk.Label(frame, text="Hold").grid(row=0, column=1, columnspan=2, padx=4, pady=3)
        ttk.Label(frame, text="Target deg").grid(row=0, column=3, padx=4, pady=3)
        for index, name in enumerate(self.node.arm_state_joint_names):
            row = index + 1
            label = name.replace("_joint", "").replace("upperArm", "upper").replace("foreArm", "fore")
            ttk.Label(frame, text=f"J{index + 1} {label}").grid(row=row, column=0, sticky="w", padx=4, pady=2)
            self._make_arm_hold_button(
                frame, "-", lambda i=index: self.node.jog_arm_joint(i, -1.0)
            ).grid(row=row, column=1, padx=2, pady=2, sticky="ew")
            self._make_arm_hold_button(
                frame, "+", lambda i=index: self.node.jog_arm_joint(i, 1.0)
            ).grid(row=row, column=2, padx=2, pady=2, sticky="ew")
            ttk.Entry(frame, textvariable=self.joint_target_vars[index], width=9).grid(
                row=row, column=3, padx=4, pady=2
            )
        ttk.Button(frame, text="Use Current", command=self._fill_current_joints).grid(
            row=7, column=0, columnspan=2, sticky="ew", padx=4, pady=(6, 4)
        )
        ttk.Button(frame, text="Use Home", command=self._fill_home_joints).grid(
            row=7, column=2, sticky="ew", padx=4, pady=(6, 4)
        )
        ttk.Button(frame, text="Move Joints", command=self._move_to_joint_targets).grid(
            row=7, column=3, columnspan=2, sticky="ew", padx=4, pady=(6, 4)
        )
        ttk.Button(frame, text="Home Pose", command=self.node.move_arm_home).grid(
            row=8, column=0, columnspan=5, sticky="ew", padx=4, pady=(0, 4)
        )

    def _build_tool_target_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="TCP Target")
        frame.grid(row=0, column=0, sticky="ew")
        for column in range(3):
            frame.columnconfigure(column, weight=1)
        for column, axis in enumerate(("x", "y", "z")):
            ttk.Label(frame, text=f"{axis.upper()} m").grid(row=0, column=column, padx=4, pady=(4, 0))
            ttk.Entry(frame, textvariable=self.tool_target_vars[axis], width=10).grid(
                row=1, column=column, padx=4, pady=(0, 4), sticky="ew"
            )
        ttk.Button(frame, text="Use Current TCP", command=self._fill_current_tool).grid(
            row=2, column=0, columnspan=2, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(frame, text="Move TCP", command=self._move_to_tool_target).grid(
            row=2, column=2, padx=4, pady=4, sticky="ew"
        )

    def _make_arm_hold_button(self, parent: ttk.Frame, text: str, callback):
        button = ttk.Button(parent, text=text)
        button.bind("<ButtonPress-1>", lambda _event: self._arm_hold_start(callback))
        button.bind("<ButtonRelease-1>", lambda _event: self._arm_hold_release())
        button.bind("<Leave>", lambda _event: self._arm_hold_release())
        return button

    def _build_gripper_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Gripper")
        frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(frame, text="Open", command=lambda: self.node.publish_gripper("open")).grid(
            row=0, column=0, padx=4, pady=4
        )
        ttk.Button(frame, text="Close", command=lambda: self.node.publish_gripper("close")).grid(
            row=0, column=1, padx=4, pady=4
        )
        ttk.Button(frame, text="Stop All", command=self.node.stop_all).grid(
            row=1, column=0, columnspan=2, padx=4, pady=4, sticky="ew"
        )

    def _refresh(self) -> None:
        snapshot = self.node.snapshot()
        snapshot["program"] = f"{len(self.waypoints)} nodes"
        for key, var in self.status_vars.items():
            var.set(snapshot.get(key, "waiting"))
        for key, var in self.move_status_vars.items():
            var.set(snapshot.get(key, "waiting"))
        self._refresh_move_joint_values()
        self._refresh_joint_tree()
        self._refresh_logs()
        self.root.after(100, self._refresh)

    def _refresh_move_joint_values(self) -> None:
        joints = self.node.joint_snapshot()
        if joints is None:
            for index, var in enumerate(self.move_joint_vars):
                var.set(f"J{index + 1}: waiting")
            return
        for index, (name, value) in enumerate(joints):
            if index >= len(self.move_joint_vars):
                break
            short = name.replace("_joint", "")
            self.move_joint_vars[index].set(f"J{index + 1} {short}: {math.degrees(value):.1f} deg")

    def _refresh_joint_tree(self) -> None:
        if self.joint_tree is None:
            return
        for item in self.joint_tree.get_children():
            self.joint_tree.delete(item)
        joints = self.node.joint_snapshot()
        if joints is None:
            self.joint_tree.insert("", tk.END, values=("waiting", "waiting"))
            return
        for name, value in joints:
            self.joint_tree.insert(
                "",
                tk.END,
                values=(f"{name}: {value:.4f}", f"{math.degrees(value):.1f}"),
            )

    def _refresh_logs(self) -> None:
        lines = "\n".join(self.node.log_snapshot()[-120:])
        for text in (getattr(self, "home_log_text", None), self.log_text):
            if text is None:
                continue
            current = text.get("1.0", tk.END).strip()
            if current == lines:
                continue
            text.configure(state="normal")
            text.delete("1.0", tk.END)
            text.insert(tk.END, lines)
            text.see(tk.END)
            text.configure(state="disabled")

    def _apply_motion_settings(self) -> None:
        try:
            self.node.update_motion_settings(
                base_linear_speed=float(self.base_linear_var.get()),
                base_angular_speed=float(self.base_angular_var.get()),
                arm_jog_step_m=float(self.arm_step_var.get()),
                arm_rotate_step_deg=float(self.arm_rotate_var.get()),
                arm_joint_step_deg=float(self.arm_joint_step_var.get()),
                arm_hold_period_sec=float(self.arm_hold_period_var.get()),
                waypoint_duration_sec=float(self.waypoint_duration_var.get()),
                arm_home_joints_deg=self.home_joints_var.get(),
            )
        except Exception as exc:
            messagebox.showerror("Apply failed", str(exc))

    def _arm_hold_start(self, callback) -> None:
        self._arm_hold_release(cancel_arm=False)
        self._arm_hold_callback = callback

        def tick() -> None:
            if self._arm_hold_callback is None:
                return
            self._arm_hold_callback()
            interval_sec = max(
                float(self.node.get_parameter("arm_hold_period_sec").value),
                float(self.node.get_parameter("arm_jog_duration_sec").value),
                float(self.node.get_parameter("arm_rotate_duration_sec").value),
            )
            interval = max(120, int(interval_sec * 1000))
            self._arm_hold_after = self.root.after(interval, tick)

        tick()

    def _arm_hold_release(self, *, cancel_arm: bool = True) -> None:
        was_active = self._arm_hold_after is not None or self._arm_hold_callback is not None
        if self._arm_hold_after is not None:
            self.root.after_cancel(self._arm_hold_after)
            self._arm_hold_after = None
        self._arm_hold_callback = None
        if cancel_arm and was_active:
            self.node.hold_arm_current()

    def _fill_current_joints(self) -> None:
        joints = self.node.joint_snapshot()
        if joints is None:
            messagebox.showerror("Joint target", "No Aubo joint state is available.")
            return
        for var, (_name, value) in zip(self.joint_target_vars, joints):
            var.set(f"{math.degrees(value):.2f}")

    def _fill_home_joints(self) -> None:
        try:
            joints = self.node.home_joints_degrees()
        except Exception as exc:
            messagebox.showerror("Home target", str(exc))
            return
        for var, value in zip(self.joint_target_vars, joints):
            var.set(f"{value:.2f}")

    def _move_to_joint_targets(self) -> None:
        try:
            targets = [float(var.get()) for var in self.joint_target_vars]
        except ValueError:
            messagebox.showerror("Move Joints", "All joint target angles must be numeric degrees.")
            return
        if len(targets) != 6:
            messagebox.showerror("Move Joints", "Expected 6 joint target angles.")
            return
        self.node.move_arm_to_joints_degrees(targets)

    def _fill_current_tool(self) -> None:
        tool = self.node.tool_snapshot()
        if tool is None:
            messagebox.showerror("TCP target", "No Aubo TCP position is available.")
            return
        for axis, value in zip(("x", "y", "z"), tool):
            self.tool_target_vars[axis].set(f"{value:.4f}")

    def _move_to_tool_target(self) -> None:
        try:
            target = [float(self.tool_target_vars[axis].get()) for axis in ("x", "y", "z")]
        except ValueError:
            messagebox.showerror("Move TCP", "TCP target x/y/z must be numeric meters.")
            return
        self.node.move_tool_to_position(target)

    def _base_press(self, direction: str) -> None:
        self.node.drive_base_manual(direction)

    def _base_release(self) -> None:
        self.node.drive_base_manual("stop")
        if self.node.pending_base_motion_count() > 0:
            self._record()

    def _record(self) -> None:
        try:
            waypoint = self.node.record_waypoint(self.label_var.get())
        except Exception as exc:
            messagebox.showerror("Record failed", str(exc))
            return
        self.waypoints.append(waypoint)
        self.label_var.set(f"wp_{len(self.waypoints) + 1}")
        self._refresh_waypoints()

    def _add_wait(self) -> None:
        try:
            seconds = float(self.wait_var.get())
        except ValueError:
            messagebox.showerror("Wait failed", "Wait seconds must be numeric.")
            return
        if seconds < 0.0:
            messagebox.showerror("Wait failed", "Wait seconds must be non-negative.")
            return
        waypoint = self.node.record_wait(self.label_var.get(), seconds)
        self.waypoints.append(waypoint)
        self.label_var.set(f"wp_{len(self.waypoints) + 1}")
        self._refresh_waypoints()

    def _delete_selected(self) -> None:
        selected = list(self.listbox.curselection())
        for index in reversed(selected):
            del self.waypoints[index]
        self._refresh_waypoints()

    def _update_selected(self) -> None:
        selected = list(self.listbox.curselection())
        if len(selected) != 1:
            messagebox.showinfo("Update WP", "Select exactly one waypoint to update.")
            return
        index = selected[0]
        source = self.waypoints[index]
        try:
            if source.kind == "wait":
                seconds = float(self.wait_var.get())
                if seconds < 0.0:
                    raise ValueError("Wait seconds must be non-negative.")
                waypoint = self.node.record_wait(source.label, seconds)
            else:
                waypoint = self.node.record_waypoint(source.label)
        except Exception as exc:
            messagebox.showerror("Update failed", str(exc))
            return
        self.waypoints[index] = waypoint
        self._refresh_waypoints()
        self.listbox.selection_set(index)
        self.listbox.see(index)

    def _duplicate_selected(self) -> None:
        selected = list(self.listbox.curselection())
        if not selected:
            messagebox.showinfo("Duplicate", "Select one or more waypoints first.")
            return
        base_label = self.label_var.get().strip()
        for offset, source_index in enumerate(selected):
            source = self.waypoints[source_index]
            waypoint = _waypoint_from_dict(asdict(source))
            if base_label:
                waypoint.label = base_label if len(selected) == 1 else f"{base_label}_{offset + 1}"
            else:
                waypoint.label = f"{source.label}_copy"
            waypoint.stamp = datetime.now().isoformat(timespec="seconds")
            self.waypoints.append(waypoint)
        self.label_var.set(f"wp_{len(self.waypoints) + 1}")
        self._refresh_waypoints()

    def _clear(self) -> None:
        self.waypoints.clear()
        self.node.clear_base_motion_history()
        self.label_var.set("wp_1")
        self._refresh_waypoints()

    def _reset(self) -> None:
        self.node.stop_all()
        self.waypoints.clear()
        self.node.clear_base_motion_history()
        self.label_var.set("wp_1")
        self.wait_var.set("2.0")
        self.file_var.set("unsaved")
        self._refresh_waypoints()

    def _save(self) -> None:
        default = self.node.recording_dir() / f"arachne_teach_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        path = filedialog.asksaveasfilename(
            initialfile=default.name,
            initialdir=str(default.parent),
            defaultextension=".json",
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        payload = {
            "format": "arachne_teach_v2",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "arm_command_joint_names": self.node.arm_command_joint_names,
            "waypoints": [asdict(item) for item in self.waypoints],
        }
        Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self.file_var.set(path)

    def _load(self) -> None:
        path = filedialog.askopenfilename(
            initialdir=str(self.node.recording_dir()),
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        self.waypoints = [_waypoint_from_dict(item) for item in payload.get("waypoints", [])]
        self.file_var.set(path)
        self.label_var.set(f"wp_{len(self.waypoints) + 1}")
        self._refresh_waypoints()

    def _play(self) -> None:
        self.node.replay(self.waypoints)

    def _refresh_waypoints(self) -> None:
        self.listbox.delete(0, tk.END)
        for index, waypoint in enumerate(self.waypoints, start=1):
            if waypoint.kind == "wait":
                self.listbox.insert(tk.END, f"{index:02d} {waypoint.label} | wait={waypoint.wait_sec:.1f}s")
                continue
            tool = waypoint.tool_position
            moves = self._base_motion_summary(waypoint.base_motion)
            tool_text = (
                f"tool=({tool[0]:.2f},{tool[1]:.2f},{tool[2]:.2f})"
                if len(tool) == 3
                else "tool=unknown"
            )
            self.listbox.insert(
                tk.END,
                (
                    f"{index:02d} {waypoint.label} | base={moves} "
                    f"{tool_text} gripper={waypoint.gripper}"
                ),
            )

    def _base_motion_summary(self, segments: list[dict]) -> str:
        if not segments:
            return "none"
        parts: list[str] = []
        for segment in segments[:3]:
            normalized = self.node._normalize_base_motion_segment(segment)
            action = normalized.get("action", "?")
            if normalized.get("type") == "linear":
                parts.append(f"{action} {float(normalized.get('distance_m', 0.0)):.2f}m")
            elif normalized.get("type") == "angular":
                angle = math.degrees(float(normalized.get("angle_rad", 0.0)))
                parts.append(f"{action} {angle:.0f}deg")
            else:
                parts.append(f"{action} {float(normalized.get('duration_sec', 0.0)):.1f}s")
        if len(segments) > 3:
            parts.append(f"+{len(segments) - 3}")
        return ", ".join(parts)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = TeachPanelNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        TeachPanelApp(node).run()
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
