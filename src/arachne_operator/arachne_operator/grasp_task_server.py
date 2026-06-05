from __future__ import annotations

import json
import math
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


AUBO_JOINT_ALIASES = (
    ("shoulder_joint", ("shoulder_joint", "aubo_shoulder_joint")),
    ("upperArm_joint", ("upperArm_joint", "aubo_upperArm_joint", "upper_arm_joint")),
    ("foreArm_joint", ("foreArm_joint", "aubo_foreArm_joint", "fore_arm_joint")),
    ("wrist1_joint", ("wrist1_joint", "aubo_wrist1_joint")),
    ("wrist2_joint", ("wrist2_joint", "aubo_wrist2_joint")),
    ("wrist3_joint", ("wrist3_joint", "aubo_wrist3_joint")),
)


TERMINAL_STATES = {"succeeded", "failed", "canceled"}


def _angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _yaw_from_odom(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class PreflightResult:
    ok: bool
    checks: list[dict[str, Any]] = field(default_factory=list)

    def add(self, name: str, ok: bool, message: str, *, required: bool = True) -> None:
        self.checks.append(
            {
                "name": name,
                "ok": bool(ok),
                "required": bool(required),
                "message": message,
            }
        )
        if required and not ok:
            self.ok = False


@dataclass
class TaskSnapshot:
    task_id: str
    state: str
    message: str
    run_dir: str
    started_at: str
    finished_at: str
    returncode: int | None
    grasp_preview_log_dir: str
    preflight: list[dict[str, Any]]
    base: dict[str, Any]


class GraspTaskServer(Node):
    """Guarded wrapper around the existing grasp_preview real execution demo.

    This node intentionally reuses scripts/vision/grasp_preview_real_sync.sh
    instead of duplicating detection, MoveIt planning, and Aubo SDK execution.
    It adds a task-level state machine, preflight checks, process control, and a
    stable log bundle per run.
    """

    def __init__(self) -> None:
        super().__init__("arachne_grasp_task_server")

        self.declare_parameter("workspace_root", "")
        self.declare_parameter("runner_script", "scripts/vision/grasp_preview_real_sync.sh")
        self.declare_parameter("log_root", "log/grasp_tasks")
        self.declare_parameter("execute_real", False)
        self.declare_parameter("confirm_execute_real", False)
        self.declare_parameter("with_rviz", False)
        self.declare_parameter("classes", "bottle")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device_id", 0)
        self.declare_parameter("real_execute_backend", "sdk_move_joint")
        self.declare_parameter("real_return_home", True)
        self.declare_parameter("real_sdk_move_speed", 0.25)
        self.declare_parameter("real_sdk_move_accel", 0.45)
        self.declare_parameter("grasp_base_offset", "0.06,0.09,-0.10")
        self.declare_parameter("extra_args", "")
        self.declare_parameter("preflight_timeout_sec", 2.0)
        self.declare_parameter("status_publish_period_sec", 0.5)
        self.declare_parameter("require_safety_state_machine", False)
        self.declare_parameter("set_safety_autonomous_on_start", True)
        self.declare_parameter("set_safety_manual_on_finish", True)
        self.declare_parameter("require_aubo_status", True)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("require_gripper_status", True)
        self.declare_parameter("require_odom", False)
        self.declare_parameter("require_camera_topics", False)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("aubo_status_topic", "/arachne/hardware/aubo_status")
        self.declare_parameter("gripper_status_topic", "/arachne/hardware/gripper_status")
        self.declare_parameter("base_status_topic", "/arachne/hardware/base_status")
        self.declare_parameter("safety_state_topic", "/arachne/safety/state")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("base_command_topic", "/arachne/grasp_task/base_command")
        self.declare_parameter("base_state_topic", "/arachne/grasp_task/base_state")
        self.declare_parameter("base_linear_speed", 0.08)
        self.declare_parameter("base_angular_speed", 0.30)
        self.declare_parameter("base_replay_linear_speed", 0.20)
        self.declare_parameter("base_replay_angular_speed", 0.24)
        self.declare_parameter("base_position_tolerance", 0.02)
        self.declare_parameter("base_yaw_tolerance_deg", 2.0)
        self.declare_parameter("base_manual_publish_rate", 12.0)
        self.declare_parameter("base_motion_max_segment_sec", 20.0)
        self.declare_parameter("base_pose_timeout_sec", 3.0)
        self.declare_parameter("allow_base_commands_during_grasp", False)

        self.root = self._resolve_workspace_root()
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.base_cancel_event = threading.Event()
        self.process: subprocess.Popen[str] | None = None
        self.worker: threading.Thread | None = None
        self.base_worker: threading.Thread | None = None
        self.latest: dict[str, tuple[float, Any]] = {}
        self.state = "idle"
        self.message = "ready"
        self.task_id = ""
        self.run_dir: Path | None = None
        self.started_at = ""
        self.finished_at = ""
        self.returncode: int | None = None
        self.preflight_checks: list[dict[str, Any]] = []
        self.grasp_preview_log_dir = ""
        self.manual_base_velocity: tuple[float, float] | None = None
        self.active_base_motion: dict[str, Any] | None = None
        self.base_motion_segments: list[dict[str, Any]] = []
        self.base_state = "idle"
        self.base_message = "ready"
        self.base_started_at = ""
        self.base_finished_at = ""
        self.base_active_command: dict[str, Any] = {}
        self.base_latest_result: dict[str, Any] = {}

        self.state_pub = self.create_publisher(String, "/arachne/grasp_task/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/grasp_task/event", 10)
        self.base_state_pub = self.create_publisher(
            String, str(self.get_parameter("base_state_topic").value), 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_service(Trigger, "/arachne/grasp_task/start", self._start_cb)
        self.create_service(Trigger, "/arachne/grasp_task/cancel", self._cancel_cb)
        self.create_service(Trigger, "/arachne/grasp_task/status", self._status_cb)
        self.create_service(Trigger, "/arachne/grasp_task/preflight", self._preflight_cb)
        self.create_service(Trigger, "/arachne/grasp_task/base_stop", self._base_stop_cb)
        self.create_service(Trigger, "/arachne/grasp_task/base_status", self._base_status_cb)

        self.set_autonomous_client = self.create_client(
            Trigger, "/arachne/safety/set_autonomous"
        )
        self.set_manual_client = self.create_client(Trigger, "/arachne/safety/set_manual")

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._cache_joint_states,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            lambda msg: self._cache("odom", msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("aubo_status_topic").value),
            lambda msg: self._cache("aubo_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_status_topic").value),
            lambda msg: self._cache("gripper_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_status_topic").value),
            lambda msg: self._cache("base_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("safety_state_topic").value),
            lambda msg: self._cache("safety_state", msg.data),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            lambda msg: self._cache("color_image", msg.header.stamp),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            lambda msg: self._cache("depth_image", msg.header.stamp),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_command_topic").value),
            self._base_command_cb,
            10,
        )

        period = max(float(self.get_parameter("status_publish_period_sec").value), 0.1)
        self.create_timer(period, self._publish_state)
        self.create_timer(period, self._publish_base_state)
        base_rate = max(float(self.get_parameter("base_manual_publish_rate").value), 1.0)
        self.create_timer(1.0 / base_rate, self._publish_manual_base_velocity)
        self._event("server_ready", {"workspace_root": str(self.root)})

    def _resolve_workspace_root(self) -> Path:
        configured = str(self.get_parameter("workspace_root").value).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        env_root = os.environ.get("ARACHNE_ROOT_DIR", "").strip()
        if env_root:
            return Path(env_root).expanduser().resolve()
        cwd = Path.cwd().resolve()
        if (cwd / "scripts" / "vision" / "grasp_preview_real_sync.sh").exists():
            return cwd
        return Path(__file__).resolve().parents[3]

    def _cache(self, key: str, value: Any) -> None:
        with self.lock:
            self.latest[key] = (time.monotonic(), value)

    def _cache_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        found: dict[str, float] = {}
        for canonical, aliases in AUBO_JOINT_ALIASES:
            for alias in aliases:
                if alias in positions:
                    found[canonical] = float(positions[alias])
                    break
        with self.lock:
            self.latest["joint_states"] = (time.monotonic(), msg)
            if len(found) == len(AUBO_JOINT_ALIASES):
                self.latest["aubo_joints"] = (time.monotonic(), found)

    def _start_cb(self, _request, response):
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy or self.state not in ("idle", *TERMINAL_STATES):
                response.success = False
                response.message = self._snapshot_json()
                return response
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run_task, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _cancel_cb(self, _request, response):
        self.cancel_event.set()
        self._terminate_process("cancel requested")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request, response):
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request, response):
        result = self._run_preflight(set_autonomous=False)
        with self.lock:
            self.preflight_checks = result.checks
        response.success = result.ok
        response.message = json.dumps(
            {"ok": result.ok, "checks": result.checks},
            ensure_ascii=False,
            sort_keys=True,
        )
        return response

    def _base_stop_cb(self, _request, response):
        ok, message = self._stop_base("service stop")
        response.success = ok
        response.message = message
        return response

    def _base_status_cb(self, _request, response):
        response.success = True
        response.message = self._base_snapshot_json()
        return response

    def _base_command_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            payload: Any = json.loads(text)
        except json.JSONDecodeError:
            payload = text
        try:
            ok, message = self._handle_base_command(payload)
        except Exception as exc:
            ok = False
            message = f"base command rejected: {exc}"
            self._set_base_state("failed", message)
        self._event("base_command", {"ok": ok, "message": message, "payload": payload})

    def request_base_motion(self, payload: dict[str, Any]) -> tuple[bool, str]:
        """Internal entry for future autonomous task policies.

        The ROS String topic uses the same parser for smoke tests and external
        bridges, but the intended production path is for task logic in this node
        to call this method after detection/planning decides a base motion is
        needed.
        """
        return self._handle_base_command(payload)

    def _handle_base_command(self, payload: Any) -> tuple[bool, str]:
        if isinstance(payload, str):
            payload = {"command": payload}
        if not isinstance(payload, dict):
            raise ValueError("base command must be a string or JSON object")

        raw_command = str(payload.get("command", payload.get("type", ""))).strip().lower()
        command = raw_command
        if not command:
            raise ValueError("missing base command")
        if raw_command == "turn_left" and "direction" not in payload:
            payload = dict(payload, direction="left")
        elif raw_command == "turn_right" and "direction" not in payload:
            payload = dict(payload, direction="right")
        command = {
            "backward": "back",
            "reverse": "back",
            "turn_left": "turn_relative",
            "turn_right": "turn_relative",
            "drive": "drive_relative",
            "move": "drive_relative",
            "velocity": "timed",
            "cmd_vel": "timed",
            "segments": "replay_segments",
        }.get(command, command)

        if command == "stop":
            return self._stop_base("command stop")

        if self._grasp_task_active() and not bool(
            self.get_parameter("allow_base_commands_during_grasp").value
        ):
            return False, "base command rejected: grasp task is active"

        if command in ("forward", "back", "left", "right", "manual", "jog"):
            direction = str(payload.get("direction", command)).strip().lower()
            if direction in ("backward", "reverse"):
                direction = "back"
            self._drive_base_manual(direction)
            return True, self._base_message_snapshot()

        if command == "drive_relative":
            distance = self._payload_float(payload, "distance_m", "distance", default=0.0)
            return self._start_base_worker(
                "drive_relative",
                payload,
                lambda: self._drive_distance(distance),
            )

        if command == "turn_relative":
            if "angle_rad" in payload:
                angle = float(payload["angle_rad"])
            elif "angle_deg" in payload:
                angle = math.radians(float(payload["angle_deg"]))
            else:
                sign = -1.0 if str(payload.get("direction", "")).lower() == "right" else 1.0
                angle = sign * math.radians(float(payload.get("angle", 0.0)))
            return self._start_base_worker(
                "turn_relative",
                payload,
                lambda: self._turn_relative(angle),
            )

        if command == "timed":
            linear_x = self._payload_float(payload, "linear_x", "vx", "linear", default=0.0)
            angular_z = self._payload_float(payload, "angular_z", "wz", "yaw_rate", default=0.0)
            duration = self._payload_float(payload, "duration_sec", "duration", default=0.0)
            return self._start_base_worker(
                "timed",
                payload,
                lambda: self._run_timed_base_motion(linear_x, angular_z, duration),
            )

        if command in ("replay_segments", "replay"):
            segments = payload.get("segments", payload.get("base_motion", []))
            if not isinstance(segments, list):
                raise ValueError("segments must be a list")
            return self._start_base_worker(
                "replay_segments",
                payload,
                lambda: self._replay_base_motion(segments, label=str(payload.get("label", "base"))),
            )

        if command == "segment":
            segment = payload.get("segment", payload)
            if not isinstance(segment, dict):
                raise ValueError("segment must be an object")
            return self._start_base_worker(
                "segment",
                payload,
                lambda: self._replay_base_motion([segment], label=str(payload.get("label", "base"))),
            )

        raise ValueError(f"unsupported base command: {command}")

    def _grasp_task_active(self) -> bool:
        with self.lock:
            worker_active = self.worker is not None and self.worker.is_alive()
            return worker_active or self.state in ("preflight", "running")

    def _payload_float(self, payload: dict[str, Any], *names: str, default: float) -> float:
        for name in names:
            if name in payload:
                return float(payload[name])
        return float(default)

    def _start_base_worker(self, label: str, payload: dict[str, Any], target) -> tuple[bool, str]:
        with self.lock:
            busy = self.base_worker is not None and self.base_worker.is_alive()
            if busy:
                return False, self._base_message_snapshot()
            self.base_cancel_event.clear()
            self.manual_base_velocity = None
            self._close_base_motion_locked(time.monotonic())
            self.base_state = "running"
            self.base_message = f"base {label} started"
            self.base_started_at = datetime.now().isoformat(timespec="seconds")
            self.base_finished_at = ""
            self.base_active_command = dict(payload)
            self.base_latest_result = {}
            self.base_worker = threading.Thread(
                target=self._base_worker_runner,
                args=(label, target),
                daemon=True,
            )
            self.base_worker.start()
        self._event("base_state", {"state": "running", "message": f"base {label} started"})
        self._publish_base_state()
        return True, f"base {label} started"

    def _base_worker_runner(self, label: str, target) -> None:
        try:
            target()
            if self.base_cancel_event.is_set():
                self._set_base_state("canceled", f"base {label} canceled")
            else:
                self._set_base_state("succeeded", f"base {label} complete")
        except Exception as exc:
            if self.base_cancel_event.is_set():
                self._set_base_state("canceled", f"base {label} canceled")
            else:
                self._set_base_state("failed", f"base {label} failed: {exc}")
        finally:
            self._publish_base_stop()

    def _drive_base_manual(self, direction: str) -> None:
        linear = float(self.get_parameter("base_linear_speed").value)
        angular = float(self.get_parameter("base_angular_speed").value)
        mapping = {
            "forward": (linear, 0.0),
            "back": (-linear, 0.0),
            "left": (0.0, angular),
            "right": (0.0, -angular),
            "stop": (0.0, 0.0),
        }
        if direction not in mapping:
            raise ValueError(f"unsupported manual base direction: {direction}")
        vx, wz = mapping[direction]
        self._track_base_motion(direction, vx, wz)
        with self.lock:
            self.manual_base_velocity = None if direction == "stop" else (vx, wz)
            if direction == "stop":
                self.base_state = "idle"
                self.base_message = "base manual stop"
                self.base_finished_at = datetime.now().isoformat(timespec="seconds")
            else:
                self.base_cancel_event.clear()
                self.base_state = "manual"
                self.base_message = f"base manual {direction}"
                self.base_started_at = self.base_started_at or datetime.now().isoformat(timespec="seconds")
                self.base_finished_at = ""
                self.base_active_command = {"command": "manual", "direction": direction}
        self.set_base_velocity(vx, wz)
        self._event("base_state", {"state": self.base_state, "message": self.base_message})
        self._publish_base_state()

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
            self.base_latest_result = {"recorded_segment": segment}
            self.base_message = self._describe_base_segment(segment)
        self.active_base_motion = None

    def _base_pose_values_locked(self) -> list[float]:
        item = self.latest.get("odom")
        if item is None or not isinstance(item[1], Odometry):
            return []
        pose = item[1].pose.pose
        return [pose.position.x, pose.position.y, _yaw_from_odom(item[1])]

    def _make_relative_base_segment(
        self, active: dict[str, Any], end_pose: list[float], duration: float
    ) -> dict[str, Any]:
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

    def _describe_base_segment(self, segment: dict[str, Any]) -> str:
        action = str(segment.get("action", "base"))
        if segment.get("type") == "linear":
            return f"base recorded: {action} {float(segment.get('distance_m', 0.0)):.3f} m"
        if segment.get("type") == "angular":
            angle = math.degrees(float(segment.get("angle_rad", 0.0)))
            return f"base recorded: {action} {angle:.1f} deg"
        return f"base recorded: {action} {float(segment.get('duration_sec', 0.0)):.1f} s"

    def _publish_manual_base_velocity(self) -> None:
        with self.lock:
            velocity = self.manual_base_velocity
        if velocity is None:
            return
        self.set_base_velocity(velocity[0], velocity[1])

    def set_base_velocity(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def _publish_base_stop(self) -> None:
        self.set_base_velocity(0.0, 0.0)

    def _stop_base(self, reason: str) -> tuple[bool, str]:
        self.base_cancel_event.set()
        with self.lock:
            self.manual_base_velocity = None
            self._close_base_motion_locked(time.monotonic())
        self._publish_base_stop()
        with self.lock:
            running = self.base_worker is not None and self.base_worker.is_alive()
        if not running:
            self._set_base_state("idle", f"base stopped: {reason}")
        else:
            self._set_base_state("canceled", f"base stopping: {reason}")
        return True, self._base_snapshot_json()

    def _run_timed_base_motion(self, linear_x: float, angular_z: float, duration: float) -> None:
        max_duration = float(self.get_parameter("base_motion_max_segment_sec").value)
        duration = max(0.0, min(float(duration), max_duration))
        deadline = time.monotonic() + duration
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            self.set_base_velocity(linear_x, angular_z)
            time.sleep(0.05)

    def _drive_distance(self, distance: float, speed_override: float | None = None) -> None:
        start = self._current_base_pose()
        heading_x = math.cos(start.yaw)
        heading_y = math.sin(start.yaw)
        tolerance = float(self.get_parameter("base_position_tolerance").value)
        speed = (
            abs(float(speed_override))
            if speed_override is not None
            else abs(float(self.get_parameter("base_replay_linear_speed").value))
        )
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.monotonic() + abs(distance) / max(speed, 1e-3) + 8.0
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose(timeout=0.1)
            progress = (current.x - start.x) * heading_x + (current.y - start.y) * heading_y
            if sign * progress >= abs(distance) - tolerance:
                self.base_latest_result = {
                    "type": "linear",
                    "target_m": float(distance),
                    "progress_m": float(progress),
                }
                return
            self.set_base_velocity(sign * speed, 0.0)
            time.sleep(0.05)
        if self.base_cancel_event.is_set():
            return
        raise TimeoutError(f"base distance timeout target={distance:.3f}m")

    def _turn_relative(self, angle: float) -> None:
        current = self._current_base_pose()
        target = math.atan2(math.sin(current.yaw + angle), math.cos(current.yaw + angle))
        self._turn_to_yaw(target)

    def _turn_to_yaw(self, target_yaw: float) -> None:
        tolerance = math.radians(float(self.get_parameter("base_yaw_tolerance_deg").value))
        speed = abs(float(self.get_parameter("base_replay_angular_speed").value))
        initial_error = abs(_angle_diff(target_yaw, self._current_base_pose().yaw))
        deadline = time.monotonic() + max(12.0, initial_error / max(speed, 1e-3) + 8.0)
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose(timeout=0.1)
            error = _angle_diff(target_yaw, current.yaw)
            if abs(error) <= tolerance:
                self.base_latest_result = {
                    "type": "angular",
                    "target_yaw_rad": float(target_yaw),
                    "error_rad": float(error),
                }
                return
            self.set_base_velocity(0.0, math.copysign(speed, error))
            time.sleep(0.05)
        if self.base_cancel_event.is_set():
            return
        raise TimeoutError(f"base yaw timeout target={math.degrees(target_yaw):.2f}deg")

    def _replay_base_motion(self, segments: list[dict[str, Any]], label: str) -> None:
        max_duration = float(self.get_parameter("base_motion_max_segment_sec").value)
        for index, segment in enumerate(segments, start=1):
            if self.base_cancel_event.is_set():
                break
            normalized = self._normalize_base_motion_segment(segment)
            motion_type = normalized.get("type")
            self._set_base_state(
                "running",
                f"base segment {index}/{len(segments)} for {label}: {motion_type}",
            )
            if motion_type == "linear":
                distance = float(normalized.get("signed_distance_m", 0.0))
                speed = abs(float(normalized.get("linear_x", 0.0)))
                self._drive_distance(distance, speed_override=speed if speed > 0.0 else None)
            elif motion_type == "angular":
                self._turn_relative(float(normalized.get("signed_angle_rad", 0.0)))
            else:
                duration = max(0.0, min(float(normalized.get("duration_sec", 0.0)), max_duration))
                linear_x = float(normalized.get("linear_x", 0.0))
                angular_z = float(normalized.get("angular_z", 0.0))
                self._run_timed_base_motion(linear_x, angular_z, duration)
            self._publish_base_stop()
            time.sleep(0.1)

    def _normalize_base_motion_segment(self, segment: dict[str, Any]) -> dict[str, Any]:
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
        active = {
            "command": command,
            "linear_x": float(segment.get("linear_x", 0.0)),
            "angular_z": float(segment.get("angular_z", 0.0)),
            "_start_pose": segment.get("start_pose", []),
            "start_stamp": segment.get("start_stamp", ""),
        }
        return self._make_relative_base_segment(
            active,
            segment.get("end_pose", []),
            float(segment.get("duration_sec", 0.0)),
        )

    def _current_base_pose(self, timeout: float | None = None) -> Pose2D:
        if timeout is None:
            timeout = float(self.get_parameter("base_pose_timeout_sec").value)
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() <= deadline:
            with self.lock:
                item = self.latest.get("odom")
            if item is not None and isinstance(item[1], Odometry):
                msg = item[1]
                pose = msg.pose.pose
                return Pose2D(pose.position.x, pose.position.y, _yaw_from_odom(msg))
            time.sleep(0.02)
        raise TimeoutError("missing /odom")

    def _set_base_state(self, state: str, message: str) -> None:
        with self.lock:
            self.base_state = state
            self.base_message = message
            if state in ("idle", "succeeded", "failed", "canceled"):
                self.base_finished_at = datetime.now().isoformat(timespec="seconds")
        self._event("base_state", {"state": state, "message": message})
        self._publish_base_state()

    def _base_message_snapshot(self) -> str:
        with self.lock:
            return self.base_message

    def _base_snapshot_dict(self) -> dict[str, Any]:
        with self.lock:
            pose_values = self._base_pose_values_locked()
            active_command = dict(self.base_active_command)
            result = dict(self.base_latest_result)
            segments = [dict(item) for item in self.base_motion_segments]
            worker_busy = self.base_worker is not None and self.base_worker.is_alive()
            manual = self.manual_base_velocity
            return {
                "state": self.base_state,
                "message": self.base_message,
                "started_at": self.base_started_at,
                "finished_at": self.base_finished_at,
                "worker_busy": worker_busy,
                "manual_velocity": list(manual) if manual is not None else [],
                "active_command": active_command,
                "latest_result": result,
                "recorded_segments": segments,
                "pose": pose_values,
            }

    def _base_snapshot_json(self) -> str:
        return json.dumps(self._base_snapshot_dict(), ensure_ascii=False, sort_keys=True)

    def _publish_base_state(self) -> None:
        msg = String()
        msg.data = self._base_snapshot_json()
        self.base_state_pub.publish(msg)

    def _run_task(self) -> None:
        task_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        run_dir = self._log_root() / task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.task_id = task_id
            self.run_dir = run_dir
            self.started_at = datetime.now().isoformat(timespec="seconds")
            self.finished_at = ""
            self.returncode = None
            self.grasp_preview_log_dir = ""
            self.preflight_checks = []
        self._set_state("preflight", "checking real-hardware readiness")
        self._write_json(run_dir / "task_request.json", self._task_request())

        result = self._run_preflight(
            set_autonomous=bool(self.get_parameter("set_safety_autonomous_on_start").value)
        )
        with self.lock:
            self.preflight_checks = result.checks
        self._write_json(run_dir / "preflight.json", {"ok": result.ok, "checks": result.checks})
        if not result.ok:
            self._finish_task("failed", "preflight failed", returncode=None)
            return

        command, env = self._runner_command()
        self._write_json(
            run_dir / "runner.json",
            {
                "command": command,
                "environment_overrides": self._logged_environment(env),
            },
        )
        self._set_state("running", "grasp_preview pipeline started")
        process_log = run_dir / "process.log"
        try:
            with process_log.open("w", encoding="utf-8") as log_file:
                self.process = subprocess.Popen(
                    command,
                    cwd=str(self.root),
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
                if self.process.stdout is not None:
                    for line in self.process.stdout:
                        log_file.write(line)
                        log_file.flush()
                        self._parse_process_line(line)
                        if self.cancel_event.is_set():
                            self._terminate_process("cancel requested")
                returncode = self.process.wait()
        except FileNotFoundError as exc:
            self._event("runner_error", {"error": str(exc)})
            self._finish_task("failed", f"runner missing: {exc}", returncode=None)
            return
        except Exception as exc:  # pragma: no cover - defensive around live process I/O.
            self._event("runner_error", {"error": str(exc)})
            self._finish_task("failed", f"runner error: {exc}", returncode=None)
            return
        finally:
            self.process = None

        if self.cancel_event.is_set():
            self._finish_task("canceled", "task canceled", returncode=returncode)
        elif returncode == 0:
            self._finish_task("succeeded", "grasp task complete", returncode=returncode)
        else:
            self._finish_task(
                "failed",
                f"grasp task failed with return code {returncode}",
                returncode=returncode,
            )

    def _run_preflight(self, *, set_autonomous: bool) -> PreflightResult:
        timeout = max(float(self.get_parameter("preflight_timeout_sec").value), 0.1)
        result = PreflightResult(ok=True)

        runner = self._runner_path()
        result.add("workspace_root", self.root.exists(), str(self.root), required=True)
        result.add("runner_script", runner.exists() and os.access(runner, os.X_OK), str(runner), required=True)
        result.add(
            "install_setup",
            (self.root / "install" / "setup.bash").exists(),
            str(self.root / "install" / "setup.bash"),
            required=True,
        )
        execute_real = bool(self.get_parameter("execute_real").value)
        confirmed = bool(self.get_parameter("confirm_execute_real").value)
        result.add(
            "real_execution_confirmation",
            (not execute_real) or confirmed,
            "execute_real requires confirm_execute_real:=true",
            required=True,
        )

        if set_autonomous:
            required = bool(self.get_parameter("require_safety_state_machine").value)
            ok, message = self._call_trigger(self.set_autonomous_client, timeout)
            result.add("safety_set_autonomous", ok, message, required=required)

        self._wait_for_fresh_inputs(timeout)
        self._add_cached_check(
            result,
            "aubo_status",
            required=bool(self.get_parameter("require_aubo_status").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: "not reachable" not in str(value).lower(),
        )
        self._add_cached_check(
            result,
            "aubo_joints",
            required=bool(self.get_parameter("require_joint_states").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "gripper_status",
            required=bool(self.get_parameter("require_gripper_status").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "odom",
            required=bool(self.get_parameter("require_odom").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "safety_state",
            required=bool(self.get_parameter("require_safety_state_machine").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: not str(value).lower().startswith(("disabled", "estop", "fault")),
        )
        if bool(self.get_parameter("require_camera_topics").value):
            self._add_cached_check(result, "color_image", required=True, max_age=max(timeout + 1.0, 2.0))
            self._add_cached_check(result, "depth_image", required=True, max_age=max(timeout + 1.0, 2.0))
        return result

    def _wait_for_fresh_inputs(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest:
                    return
            time.sleep(0.05)

    def _add_cached_check(
        self,
        result: PreflightResult,
        name: str,
        *,
        required: bool,
        max_age: float,
        validator=None,
    ) -> None:
        with self.lock:
            item = self.latest.get(name)
        if item is None:
            result.add(name, False, "no recent message", required=required)
            return
        stamp, value = item
        age = time.monotonic() - stamp
        ok = age <= max_age
        message = f"age={age:.2f}s value={self._short_value(value)}"
        if ok and validator is not None:
            try:
                ok = bool(validator(value))
            except Exception as exc:
                ok = False
                message += f" validator_error={exc}"
        result.add(name, ok, message, required=required)

    def _call_trigger(self, client, timeout: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"service unavailable: {getattr(client, 'srv_name', 'trigger')}"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return False, f"service timeout: {getattr(client, 'srv_name', 'trigger')}"
        response = future.result()
        if response is None:
            return False, f"empty response: {getattr(client, 'srv_name', 'trigger')}"
        return bool(response.success), str(response.message)

    def _runner_command(self) -> tuple[list[str], dict[str, str]]:
        command = [str(self._runner_path())]
        if bool(self.get_parameter("execute_real").value):
            command.append("--execute-real")
        extra = shlex.split(str(self.get_parameter("extra_args").value))
        if extra:
            command.append("--")
            command.extend(extra)

        env = os.environ.copy()
        env["ARACHNE_GRASP_WITH_RVIZ"] = "true" if bool(self.get_parameter("with_rviz").value) else "false"
        env["ARACHNE_GRASP_CLASSES"] = str(self.get_parameter("classes").value)
        env["ARACHNE_GRASP_CONF"] = str(self.get_parameter("confidence").value)
        env["ARACHNE_GRASP_DEVICE_ID"] = str(self.get_parameter("device_id").value)
        env["ARACHNE_GRASP_REAL_EXECUTE_BACKEND"] = str(
            self.get_parameter("real_execute_backend").value
        )
        env["ARACHNE_GRASP_REAL_RETURN_HOME"] = (
            "true" if bool(self.get_parameter("real_return_home").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_SPEED"] = str(
            self.get_parameter("real_sdk_move_speed").value
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL"] = str(
            self.get_parameter("real_sdk_move_accel").value
        )
        env["ARACHNE_GRASP_BASE_OFFSET"] = str(self.get_parameter("grasp_base_offset").value)
        if bool(self.get_parameter("confirm_execute_real").value):
            env["ARACHNE_CONFIRM_GRASP_EXECUTE_REAL"] = "YES"
        return command, env

    def _runner_path(self) -> Path:
        configured = Path(str(self.get_parameter("runner_script").value))
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _log_root(self) -> Path:
        configured = Path(str(self.get_parameter("log_root").value)).expanduser()
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _task_request(self) -> dict[str, Any]:
        names = (
            "execute_real",
            "confirm_execute_real",
            "with_rviz",
            "classes",
            "confidence",
            "device_id",
            "real_execute_backend",
            "real_return_home",
            "real_sdk_move_speed",
            "real_sdk_move_accel",
            "grasp_base_offset",
            "extra_args",
        )
        return {name: self.get_parameter(name).value for name in names}

    def _logged_environment(self, env: dict[str, str]) -> dict[str, str]:
        keys = sorted(key for key in env if key.startswith("ARACHNE_GRASP_"))
        keys.append("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL")
        return {key: env.get(key, "") for key in keys}

    def _parse_process_line(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return
        match = re.search(r"Grasp preview logs:\s*(.+)$", stripped)
        if match:
            with self.lock:
                self.grasp_preview_log_dir = match.group(1).strip()
            self._event("grasp_preview_log_dir", {"path": match.group(1).strip()})
        if "REAL arm SDK moveJoint sequence complete" in stripped:
            self._event("arm_sequence_complete", {"line": stripped})
        elif "REAL execution is armed" in stripped:
            self._event("real_execution_armed", {"line": stripped})
        elif "grasp task failed" in stripped.lower() or "failed" in stripped.lower():
            self._event("runner_warning", {"line": stripped})

    def _finish_task(self, state: str, message: str, *, returncode: int | None) -> None:
        with self.lock:
            self.returncode = returncode
            self.finished_at = datetime.now().isoformat(timespec="seconds")
        if bool(self.get_parameter("set_safety_manual_on_finish").value):
            self._call_trigger(self.set_manual_client, timeout=0.8)
        self._set_state(state, message)
        run_dir = self.run_dir
        if run_dir is not None:
            self._write_json(run_dir / "summary.json", asdict(self._snapshot()))

    def _terminate_process(self, reason: str) -> None:
        with self.lock:
            process = self.process
        if process is None or process.poll() is not None:
            return
        self._event("terminate", {"reason": reason})
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().warning(f"SIGINT failed: {exc}")
        deadline = time.monotonic() + 3.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            except Exception as exc:
                self.get_logger().warning(f"SIGTERM failed: {exc}")

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _event(self, kind: str, data: dict[str, Any]) -> None:
        event = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "task_id": self.task_id,
            **data,
        }
        text = json.dumps(event, ensure_ascii=False, sort_keys=True)
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        run_dir = self.run_dir
        if run_dir is not None:
            try:
                with (run_dir / "events.jsonl").open("a", encoding="utf-8") as handle:
                    handle.write(text + "\n")
            except OSError as exc:
                self.get_logger().warning(f"event log write failed: {exc}")
        if kind == "state":
            self.get_logger().info(f"{data.get('state')}: {data.get('message')}")

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._snapshot_json()
        self.state_pub.publish(msg)

    def _snapshot(self) -> TaskSnapshot:
        with self.lock:
            return TaskSnapshot(
                task_id=self.task_id,
                state=self.state,
                message=self.message,
                run_dir=str(self.run_dir or ""),
                started_at=self.started_at,
                finished_at=self.finished_at,
                returncode=self.returncode,
                grasp_preview_log_dir=self.grasp_preview_log_dir,
                preflight=list(self.preflight_checks),
                base=self._base_snapshot_dict(),
            )

    def _snapshot_json(self) -> str:
        return json.dumps(asdict(self._snapshot()), ensure_ascii=False, sort_keys=True)

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _short_value(self, value: Any) -> str:
        if isinstance(value, dict):
            return json.dumps(value, ensure_ascii=False, sort_keys=True)[:240]
        text = str(value)
        return text if len(text) <= 240 else text[:237] + "..."


def main() -> None:
    rclpy.init()
    node = GraspTaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_event.set()
        node.base_cancel_event.set()
        node._publish_base_stop()
        node._terminate_process("shutdown")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
