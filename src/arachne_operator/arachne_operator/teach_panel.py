from __future__ import annotations

import json
import math
import os
import signal
import shlex
import socket
import subprocess
import threading
import time
from dataclasses import asdict, dataclass, fields, field
from datetime import datetime
from pathlib import Path
from typing import Any
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
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

DEFAULT_ARM_HOME_JOINTS_DEG = "-90.00,11.55,95.09,27.80,96.07,43.79"
DEFAULT_ARM_INSTALL_JOINTS_DEG = DEFAULT_ARM_HOME_JOINTS_DEG
DEFAULT_AUBO_PAYLOAD_MASS_KG = 0.818
DEFAULT_AUBO_PAYLOAD_COG = "0.039927,0.045067,0.143233"
DEFAULT_AUBO_PAYLOAD_AOM = "0,0,0"
DEFAULT_AUBO_PAYLOAD_INERTIA = "0,0,0,0,0,0"
DEFAULT_ARM_BASE_XYZ = "0.22,0.0,0.105"
DEFAULT_ARM_BASE_RPY = "0.0,0.0,1.57079632679"
DEFAULT_REAR_RACK_KEEPOUT_MIN_XYZ = "-0.41,-0.22,0.04"
DEFAULT_REAR_RACK_KEEPOUT_MAX_XYZ = "0.09,0.22,0.82"
DEFAULT_TEACH_CONFIG_PATH = "recordings/teach/teach_panel_config.json"
DEFAULT_AUBO_TEACH_FLAG_PATH = "/tmp/arachne_aubo_teach_mode"
DEFAULT_AUBO_CONTROL_OWNER_PATH = "/tmp/arachne_aubo_control_owner"
MANAGED_SERVICE_COMMAND_PARAMS = {
    "camera": "camera_command",
    "viewer": "camera_view_command",
    "slam": "slam_command",
    "grasp_server": "grasp_server_command",
    "cleanup_server": "cleanup_server_command",
}


def _control_owner_payload(owner: str) -> str:
    return json.dumps(
        {"owner": owner, "pid": os.getpid(), "created_at": time.time()},
        separators=(",", ":"),
    ) + "\n"


def _parse_control_owner(text: str) -> tuple[str, int | None]:
    text = text.strip()
    if not text:
        return "", None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return text.splitlines()[0].strip(), None
    owner = str(data.get("owner", "")).strip()
    pid_value = data.get("pid")
    try:
        pid = int(pid_value) if pid_value is not None else None
    except (TypeError, ValueError):
        pid = None
    return owner, pid


def _pid_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _claim_control_owner(path: Path, owner: str) -> tuple[bool, str]:
    owner = owner.strip() or "unknown"
    for _attempt in range(2):
        try:
            fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError as exc:
                return False, f"unreadable owner file {path}: {exc}"
            active_owner, pid = _parse_control_owner(text)
            if active_owner == owner and pid == os.getpid():
                return True, f"already owned by {owner}"
            if pid is not None and not _pid_alive(pid):
                try:
                    path.unlink(missing_ok=True)
                except OSError as exc:
                    return False, f"stale owner {active_owner or text!r} could not be cleared: {exc}"
                continue
            pid_text = str(pid) if pid is not None else "unknown"
            return False, f"owned by {active_owner or text.strip() or 'unknown'} pid={pid_text}"
        except OSError as exc:
            return False, f"could not create owner file {path}: {exc}"

        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(_control_owner_payload(owner))
        except OSError as exc:
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass
            return False, f"could not write owner file {path}: {exc}"
        return True, f"owned by {owner}"
    return False, f"could not claim owner file {path}"


def _release_control_owner(path: Path, owner: str) -> None:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except (FileNotFoundError, OSError):
        return
    active_owner, pid = _parse_control_owner(text)
    if active_owner != (owner.strip() or "unknown"):
        return
    if pid is not None and pid != os.getpid():
        return
    try:
        path.unlink(missing_ok=True)
    except OSError:
        return


class AuboDirectJsonRpc:
    def __init__(self, ip: str, port: int, timeout: float) -> None:
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def __enter__(self) -> "AuboDirectJsonRpc":
        self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
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
        request = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params or [],
            "id": self.request_id,
        }
        self.sock.sendall(json.dumps(request, separators=(",", ":")).encode("utf-8"))
        response = json.loads(self.sock.recv(8192).decode("utf-8", errors="replace"))
        if response.get("error") not in (None, "", "None", "null"):
            raise RuntimeError(f"{method} failed: {response['error']}")
        return response.get("result")

    def robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        return self.call(f"{self.robot_name}.{suffix}", params)


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


@dataclass(frozen=True)
class KeepoutBox:
    name: str
    minimum: np.ndarray
    maximum: np.ndarray


def _angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _yaw_from_odom(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_names(text: str) -> list[str]:
    disabled = {"", "0", "false", "none", "off", "no"}
    text = str(text).strip()
    if text.lower() in disabled:
        return []
    return [item.strip() for item in text.split(",") if item.strip()]


def _parse_vector(text: str, *, expected: int, label: str = "vector") -> list[float]:
    values = [
        float(item.strip())
        for item in str(text).replace(";", ",").replace(" ", ",").split(",")
        if item.strip()
    ]
    if len(values) != expected:
        raise ValueError(f"{label} must contain {expected} values, got {len(values)}")
    return values


def _parse_vector3(text: str) -> np.ndarray:
    return np.array(_parse_vector(text, expected=3), dtype=float)


def _parse_joint_degrees(text: str, *, label: str) -> list[float]:
    values = [
        float(item.strip())
        for item in str(text).replace(";", ",").split(",")
        if item.strip()
    ]
    if len(values) != 6:
        raise ValueError(f"{label} must contain 6 comma-separated degrees")
    return values


def _format_joint_degrees(values: list[float]) -> str:
    if len(values) != 6:
        raise ValueError("joint pose must contain 6 values")
    return ",".join(f"{float(value):.2f}" for value in values)


def _bool_value(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _transform_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rotation_matrix_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    transform[:3, 3] = xyz
    return transform


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
        self.declare_parameter("base_linear_speed", 0.16)
        self.declare_parameter("base_angular_speed", 0.60)
        self.declare_parameter("base_replay_linear_speed", 0.40)
        self.declare_parameter("base_replay_angular_speed", 0.48)
        self.declare_parameter("base_position_tolerance", 0.02)
        self.declare_parameter("base_yaw_tolerance_deg", 2.0)
        self.declare_parameter("base_manual_publish_rate", 12.0)
        self.declare_parameter("base_motion_max_segment_sec", 20.0)
        self.declare_parameter("base_ignore_spurious_zero_odom", True)
        self.declare_parameter("arm_jog_step_m", 0.008)
        self.declare_parameter("arm_jog_duration_sec", 0.24)
        self.declare_parameter("arm_rotate_step_rad", math.radians(0.7))
        self.declare_parameter("arm_rotate_duration_sec", 0.24)
        self.declare_parameter("arm_joint_step_rad", math.radians(0.4))
        self.declare_parameter("arm_hold_period_sec", 0.10)
        self.declare_parameter("arm_manual_prefer_topic", True)
        self.declare_parameter(
            "arm_velocity_command_topic", "/arachne/aubo/joint_velocity_command"
        )
        self.declare_parameter("arm_velocity_publish_rate", 80.0)
        self.declare_parameter("arm_velocity_watchdog_sec", 0.20)
        self.declare_parameter("arm_joint_jog_speed_rad_sec", 0.08)
        self.declare_parameter("arm_cartesian_jog_speed_m_sec", 0.025)
        self.declare_parameter("arm_cartesian_rotate_speed_rad_sec", 0.08)
        self.declare_parameter("arm_velocity_damping", 0.08)
        self.declare_parameter("arm_velocity_max_joint_speed_rad_sec", 0.25)
        self.declare_parameter("arm_velocity_max_joint_accel_rad_sec2", 1.60)
        self.declare_parameter("arm_velocity_max_joint_jerk_rad_sec3", 24.0)
        self.declare_parameter("arm_velocity_smoothing_tau_sec", 0.08)
        self.declare_parameter("arm_velocity_keepout_predict_sec", 0.35)
        self.declare_parameter("arm_velocity_keepout_check_interval_sec", 0.05)
        self.declare_parameter("arm_velocity_stream_deadman_sec", 0.75)
        self.declare_parameter("arm_waypoint_duration_sec", 3.75)
        self.declare_parameter("arm_replay_backend", "sdk_move_joint")
        self.declare_parameter("aubo_sdk_ip", os.environ.get("AUBO_ROBOT_IP", "192.168.127.128"))
        self.declare_parameter("aubo_sdk_rpc_port", 30004)
        self.declare_parameter("aubo_sdk_rpc_timeout_sec", 3.0)
        self.declare_parameter("aubo_sdk_move_speed_rad_sec", 0.25)
        self.declare_parameter("aubo_sdk_move_accel_rad_sec2", 0.45)
        self.declare_parameter("aubo_sdk_blend_radius", 0.0)
        self.declare_parameter("aubo_sdk_move_duration_sec", 0.0)
        self.declare_parameter("aubo_sdk_goal_tolerance_rad", 0.04)
        self.declare_parameter("aubo_sdk_arrival_timeout_padding_sec", 3.0)
        self.declare_parameter("aubo_sdk_lifecycle_power_timeout_sec", 45.0)
        self.declare_parameter("aubo_sdk_lifecycle_startup_timeout_sec", 45.0)
        self.declare_parameter("aubo_sdk_lifecycle_poll_sec", 0.5)
        self.declare_parameter("aubo_sdk_teach_flag_path", DEFAULT_AUBO_TEACH_FLAG_PATH)
        self.declare_parameter("aubo_sdk_control_owner_path", DEFAULT_AUBO_CONTROL_OWNER_PATH)
        self.declare_parameter("aubo_sdk_control_owner_name", "teach_panel")
        self.declare_parameter("arm_home_joints_deg", DEFAULT_ARM_HOME_JOINTS_DEG)
        self.declare_parameter("arm_install_joints_deg", DEFAULT_ARM_INSTALL_JOINTS_DEG)
        self.declare_parameter("aubo_payload_mass_kg", DEFAULT_AUBO_PAYLOAD_MASS_KG)
        self.declare_parameter("aubo_payload_cog", DEFAULT_AUBO_PAYLOAD_COG)
        self.declare_parameter("aubo_payload_aom", DEFAULT_AUBO_PAYLOAD_AOM)
        self.declare_parameter("aubo_payload_inertia", DEFAULT_AUBO_PAYLOAD_INERTIA)
        self.declare_parameter("arm_goal_tolerance", 0.04)
        self.declare_parameter("arm_position_tolerance", 0.006)
        self.declare_parameter("arm_jog_position_tolerance", 0.0008)
        self.declare_parameter("arm_orientation_tolerance", 0.01)
        self.declare_parameter("arm_jog_orientation_tolerance", 0.004)
        self.declare_parameter("arm_ik_damping", 0.08)
        self.declare_parameter("arm_ik_max_iterations", 180)
        self.declare_parameter("arm_ik_max_step", 0.05)
        self.declare_parameter("arm_jog_max_joint_delta", 0.25)
        self.declare_parameter("arm_target_max_joint_delta", 1.2)
        self.declare_parameter("arm_keepout_enabled", True)
        self.declare_parameter("arm_base_xyz", DEFAULT_ARM_BASE_XYZ)
        self.declare_parameter("arm_base_rpy", DEFAULT_ARM_BASE_RPY)
        self.declare_parameter("rear_rack_keepout_min_xyz", DEFAULT_REAR_RACK_KEEPOUT_MIN_XYZ)
        self.declare_parameter("rear_rack_keepout_max_xyz", DEFAULT_REAR_RACK_KEEPOUT_MAX_XYZ)
        self.declare_parameter("arm_keepout_sample_step_m", 0.035)
        self.declare_parameter("arm_keepout_joint_step_rad", 0.06)
        self.declare_parameter("aubo_teach_command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("aubo_teach_exit_wait_sec", 8.0)
        self.declare_parameter("replay_settle_sec", 0.2)
        self.declare_parameter("gripper_settle_sec", 2.0)
        self.declare_parameter("recording_dir", "recordings/teach")
        self.declare_parameter("teach_config_path", DEFAULT_TEACH_CONFIG_PATH)
        self.declare_parameter("teach_config_autoload", True)
        self.declare_parameter("workspace_root", "")
        self.declare_parameter("runtime_log_root", "log/teach_panel")
        self.declare_parameter(
            "autostart_managed_processes",
            "camera,viewer,grasp_server,cleanup_server",
        )
        self.declare_parameter("service_stop_timeout_sec", 4.0)
        self.declare_parameter(
            "camera_command",
            (
                "ros2 launch arachne_sensors gemini335.launch.py "
                "publish_pointcloud:=false with_color_view:=false with_depth_view:=false "
                "with_tf:=true camera_parent_frame:=tool0 "
                "camera_optical_x:=-0.239469796 camera_optical_y:=0.181459396 "
                "camera_optical_z:=0.190102132 camera_optical_roll:=0.083404947 "
                "camera_optical_pitch:=-0.300045345 camera_optical_yaw:=3.128380060 "
                "projection_flip_x:=true projection_flip_y:=true color_yuv_layout:=YUYV"
            ),
        )
        self.declare_parameter(
            "camera_view_command",
            (
                "${ARACHNE_SYSTEM_PYTHON:-python3} scripts/vision/raw_image_viewer.py "
                "--topic /camera/color/image_raw --window \"Arachne Raw Camera\" --max-fps 15"
            ),
        )
        self.declare_parameter("slam_command", "scripts/hardware/real_lidar_nav.sh")
        self.declare_parameter(
            "grasp_server_command",
            (
                "scripts/vision/grasp_task_server.sh "
                "execute_real:=true confirm_execute_real:=true with_rviz:=false "
                "preview_on_start:=true planning_recovery_base_enabled:=false "
                "require_odom:=false require_camera_topics:=true "
                "require_aubo_status:=false require_gripper_status:=false "
                "max_grasp_attempts:=3 retry_on_gripper_miss:=true"
            ),
        )
        self.declare_parameter(
            "cleanup_server_command",
            (
                "scripts/vision/road_cleanup_task_server.sh "
                "patrol_pattern:=box_entry patrol_box_width_m:=1.0 "
                "patrol_box_height_m:=1.2 patrol_entry_m:=0.3 max_round_trips:=2 "
                "patrol_base_speed_mps:=0.06 base_step_timeout_sec:=32.0 "
                "candidate_min_base_z_m:=-0.08 candidate_max_reach_m:=0.70 "
                "initial_detection_wait_sec:=3.0 patrol_turn_scale:=1.0 "
                "detection_confidence:=0.35 loop:=true"
            ),
        )
        self.declare_parameter("grasp_task_state_topic", "/arachne/grasp_task/state")
        self.declare_parameter("grasp_task_start_service", "/arachne/grasp_task/start")
        self.declare_parameter("grasp_task_stop_service", "/arachne/grasp_task/stop")
        self.declare_parameter("grasp_task_restore_service", "/arachne/grasp_task/restore")
        self.declare_parameter("grasp_task_status_service", "/arachne/grasp_task/status")
        self.declare_parameter("grasp_task_preflight_service", "/arachne/grasp_task/preflight")
        self.declare_parameter("cleanup_task_state_topic", "/arachne/road_cleanup/state")
        self.declare_parameter("cleanup_task_start_service", "/arachne/road_cleanup/start")
        self.declare_parameter("cleanup_task_pause_service", "/arachne/road_cleanup/pause")
        self.declare_parameter(
            "cleanup_task_return_home_service", "/arachne/road_cleanup/return_home"
        )
        self.declare_parameter("cleanup_task_stop_service", "/arachne/road_cleanup/stop")
        self.declare_parameter("cleanup_task_status_service", "/arachne/road_cleanup/status")
        self.declare_parameter("cleanup_task_preflight_service", "/arachne/road_cleanup/preflight")

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
        arm_velocity_qos = QoSProfile(depth=1)
        self.arm_velocity_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("arm_velocity_command_topic").value),
            arm_velocity_qos,
        )
        self.status_pub = self.create_publisher(String, "/arachne/teach/status", 10)
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_follow_joint_trajectory_action").value),
        )
        self.grasp_task_clients = {
            "start": self.create_client(
                Trigger, str(self.get_parameter("grasp_task_start_service").value)
            ),
            "stop": self.create_client(
                Trigger, str(self.get_parameter("grasp_task_stop_service").value)
            ),
            "restore": self.create_client(
                Trigger, str(self.get_parameter("grasp_task_restore_service").value)
            ),
            "status": self.create_client(
                Trigger, str(self.get_parameter("grasp_task_status_service").value)
            ),
            "preflight": self.create_client(
                Trigger, str(self.get_parameter("grasp_task_preflight_service").value)
            ),
        }
        self.cleanup_task_clients = {
            "start": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_start_service").value)
            ),
            "pause": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_pause_service").value)
            ),
            "return_home": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_return_home_service").value)
            ),
            "stop": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_stop_service").value)
            ),
            "status": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_status_service").value)
            ),
            "preflight": self.create_client(
                Trigger, str(self.get_parameter("cleanup_task_preflight_service").value)
            ),
        }

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)
        self.create_subscription(String, "/arachne/hardware/base_status", self._status_callback("Base"), 10)
        self.create_subscription(String, "/arachne/hardware/aubo_status", self._status_callback("Aubo"), 10)
        self.create_subscription(
            String, "/arachne/hardware/gripper_status", self._gripper_status_callback, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("grasp_task_state_topic").value),
            self._grasp_task_state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("cleanup_task_state_topic").value),
            self._cleanup_task_state_callback,
            10,
        )

        self.kinematics = AuboI5Kinematics()
        self.lock = threading.Lock()
        self.manual_arm_command_lock = threading.Lock()
        self.base_pose: Pose2D | None = None
        self.base_motion_segments: list[dict] = []
        self.active_base_motion: dict | None = None
        self.base_motion_recording_enabled = False
        self.manual_base_velocity: tuple[float, float] | None = None
        self.manual_arm_stream_command: dict[str, object] | None = None
        self.manual_arm_stream_deadline = 0.0
        self.manual_arm_velocity: list[float] | None = None
        self.manual_arm_velocity_deadline = 0.0
        self.arm_velocity_filtered = [0.0 for _ in self.arm_command_joint_names]
        self.arm_velocity_accel = [0.0 for _ in self.arm_command_joint_names]
        self.arm_velocity_last_update = time.monotonic()
        self.arm_velocity_command_generation = 0
        self.arm_velocity_keepout_last_check = 0.0
        self.arm_velocity_keepout_last_violation: str | None = None
        self.current_arm: dict[str, float] = {}
        self.tool_position: tuple[float, float, float] | None = None
        self.gripper_state = "open"
        self.hardware_status = {
            "Base": "waiting",
            "Aubo": "waiting",
            "Gripper": "waiting",
            "Grasp": "waiting",
            "Road": "waiting",
        }
        self.aubo_teach_gate_active = False
        self.aubo_teach_ready_event = threading.Event()
        self.aubo_teach_ready_event.set()
        self.last_status = "ready"
        self.log_lines: list[str] = []
        self.cancel_event = threading.Event()
        self.replay_thread: threading.Thread | None = None
        self._active_goal_handle = None
        self._last_manual_arm_stream_status = 0.0
        self.workspace_root = self._resolve_workspace_root()
        self.runtime_log_dir = self._make_runtime_log_dir()
        self.event_log_path = self.runtime_log_dir / "events.jsonl"
        self.managed_processes: dict[str, subprocess.Popen] = {}
        self.managed_process_logs: dict[str, Path] = {}
        self.managed_process_log_handles: dict[str, Any] = {}
        self.last_logged_hardware_status: dict[str, str] = {}
        self.autostart_managed_processes = _parse_names(
            str(self.get_parameter("autostart_managed_processes").value)
        )
        self.autostart_requested = False
        if bool(self.get_parameter("teach_config_autoload").value):
            self.load_teach_config(status=False)
        publish_rate = max(float(self.get_parameter("base_manual_publish_rate").value), 1.0)
        self.create_timer(1.0 / publish_rate, self._publish_manual_base_velocity)
        arm_velocity_rate = max(float(self.get_parameter("arm_velocity_publish_rate").value), 1.0)
        self.create_timer(1.0 / arm_velocity_rate, self._publish_manual_arm_velocity)
        self.create_timer(1.0, self._autostart_managed_processes_once)
        self._status(f"ready; logs={self.runtime_log_dir}")

    def _autostart_managed_processes_once(self) -> None:
        if self.autostart_requested or not self.autostart_managed_processes:
            return
        self.autostart_requested = True
        for name in self.autostart_managed_processes:
            if name not in MANAGED_SERVICE_COMMAND_PARAMS:
                self._status(f"autostart ignored unknown service: {name}", warn=True)
                continue
            self._start_worker(lambda service=name: self._start_managed_process_worker(service))

    def _resolve_workspace_root(self) -> Path:
        configured = str(self.get_parameter("workspace_root").value).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        for parent in Path(__file__).resolve().parents:
            if (parent / "scripts/env/arachne_env.sh").exists():
                return parent
        return Path.cwd().resolve()

    def _workspace_path(self, path: str | Path) -> Path:
        resolved = Path(str(path)).expanduser()
        if not resolved.is_absolute():
            resolved = self.workspace_root / resolved
        return resolved

    def _make_runtime_log_dir(self) -> Path:
        root = self._workspace_path(str(self.get_parameter("runtime_log_root").value))
        directory = root / datetime.now().strftime("%Y%m%d_%H%M%S")
        directory.mkdir(parents=True, exist_ok=True)
        latest = root / "latest"
        try:
            latest.unlink(missing_ok=True)
            latest.symlink_to(directory)
        except OSError:
            pass
        return directory

    def _write_event(self, kind: str, message: str, **fields: Any) -> None:
        path = getattr(self, "event_log_path", None)
        if path is None:
            return
        payload = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "message": str(message),
            **fields,
        }
        try:
            with Path(path).open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
        except OSError as exc:
            self.get_logger().warning(f"teach event log write failed: {exc}")

    def _log_hardware_status_change(self, key: str, value: str) -> None:
        previous = self.last_logged_hardware_status.get(key)
        if previous == value:
            return
        self.last_logged_hardware_status[key] = value
        self._write_event("hardware_status", value, key=key)

    def _odom_callback(self, msg: Odometry) -> None:
        pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, _yaw_from_odom(msg))
        with self.lock:
            if self._looks_like_spurious_zero_odom_locked(pose):
                return
            self.base_pose = pose

    def _looks_like_spurious_zero_odom_locked(self, pose: Pose2D) -> bool:
        if not bool(self.get_parameter("base_ignore_spurious_zero_odom").value):
            return False
        current = self.base_pose
        if current is None:
            return False
        incoming_near_zero = (
            abs(pose.x) < 1e-4 and abs(pose.y) < 1e-4 and abs(pose.yaw) < math.radians(0.05)
        )
        if not incoming_near_zero:
            return False
        current_nonzero = (
            math.hypot(current.x, current.y) > 0.02
            or abs(current.yaw) > math.radians(2.0)
        )
        return current_nonzero

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
            data = msg.data
            with self.lock:
                self.hardware_status[key] = data
                if key == "Aubo":
                    self._update_aubo_teach_state_locked(data)
            self._log_hardware_status_change(key, data)

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
        self._log_hardware_status_change("Gripper", data)

    def _grasp_task_state_callback(self, msg: String) -> None:
        data = msg.data.strip()
        label = data
        try:
            payload = json.loads(data)
            state = str(payload.get("state", "unknown"))
            message = str(payload.get("message", "")).strip()
            label = f"{state}: {message}" if message else state
        except json.JSONDecodeError:
            pass
        with self.lock:
            self.hardware_status["Grasp"] = label
        self._log_hardware_status_change("Grasp", label)

    def _cleanup_task_state_callback(self, msg: String) -> None:
        data = msg.data.strip()
        label = data
        try:
            payload = json.loads(data)
            state = str(payload.get("state", "unknown"))
            message = str(payload.get("message", "")).strip()
            progress = payload.get("progress_m")
            prefix = f"{state}: {message}" if message else state
            if isinstance(progress, (int, float)):
                prefix = f"{prefix} ({float(progress):.2f}m)"
            label = prefix
        except json.JSONDecodeError:
            pass
        with self.lock:
            self.hardware_status["Road"] = label
        self._log_hardware_status_change("Road", label)

    def snapshot(self) -> dict[str, str]:
        with self.lock:
            base = self.base_pose
            tool = self.tool_position
            arm_ready = self._current_arm_vector_locked() is not None
            managed = {
                key: self._managed_process_status_locked(key)
                for key in MANAGED_SERVICE_COMMAND_PARAMS
            }
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
                "grasp_task": self.hardware_status.get("Grasp", "waiting"),
                "road_cleanup": self.hardware_status.get("Road", "waiting"),
                "status": self.last_status,
                "camera": managed["camera"],
                "viewer": managed["viewer"],
                "slam": managed["slam"],
                "grasp_server": managed["grasp_server"],
                "cleanup_server": managed["cleanup_server"],
                "log_dir": str(self.runtime_log_dir),
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
        directory = self._workspace_path(str(self.get_parameter("recording_dir").value))
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def teach_config_path(self, path: str | Path | None = None) -> Path:
        config_path = self._workspace_path(
            str(path if path is not None else self.get_parameter("teach_config_path").value)
        )
        config_path.parent.mkdir(parents=True, exist_ok=True)
        return config_path

    def current_joints_degrees(self) -> list[float] | None:
        current = self._current_arm_vector()
        if current is None:
            return None
        return [math.degrees(float(value)) for value in current]

    def teach_config_payload(self) -> dict:
        return {
            "format": "arachne_teach_config_v1",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "motion": {
                "base_linear_speed": float(self.get_parameter("base_linear_speed").value),
                "base_angular_speed": float(self.get_parameter("base_angular_speed").value),
                "arm_jog_step_m": float(self.get_parameter("arm_jog_step_m").value),
                "arm_rotate_step_deg": math.degrees(
                    float(self.get_parameter("arm_rotate_step_rad").value)
                ),
                "arm_joint_step_deg": math.degrees(
                    float(self.get_parameter("arm_joint_step_rad").value)
                ),
                "arm_hold_period_sec": float(
                    self.get_parameter("arm_hold_period_sec").value
                ),
                "arm_waypoint_duration_sec": float(
                    self.get_parameter("arm_waypoint_duration_sec").value
                ),
                "gripper_settle_sec": float(self.get_parameter("gripper_settle_sec").value),
            },
            "poses": {
                "home_joints_deg": str(self.get_parameter("arm_home_joints_deg").value),
                "install_joints_deg": str(self.get_parameter("arm_install_joints_deg").value),
            },
            "payload": {
                "mass_kg": float(self.get_parameter("aubo_payload_mass_kg").value),
                "cog_m": str(self.get_parameter("aubo_payload_cog").value),
                "aom": str(self.get_parameter("aubo_payload_aom").value),
                "inertia": str(self.get_parameter("aubo_payload_inertia").value),
            },
            "safety": {
                "arm_keepout_enabled": bool(self.get_parameter("arm_keepout_enabled").value),
                "rear_rack_keepout_min_xyz": str(
                    self.get_parameter("rear_rack_keepout_min_xyz").value
                ),
                "rear_rack_keepout_max_xyz": str(
                    self.get_parameter("rear_rack_keepout_max_xyz").value
                ),
                "arm_keepout_sample_step_m": float(
                    self.get_parameter("arm_keepout_sample_step_m").value
                ),
                "arm_keepout_joint_step_rad": float(
                    self.get_parameter("arm_keepout_joint_step_rad").value
                ),
            },
        }

    def apply_teach_config(self, payload: dict) -> None:
        motion = payload.get("motion", {})
        poses = payload.get("poses", {})
        payload_defaults = payload.get("payload", {})
        safety = payload.get("safety", {})

        home_values = _parse_joint_degrees(
            poses.get("home_joints_deg", self.get_parameter("arm_home_joints_deg").value),
            label="home joints",
        )
        install_values = _parse_joint_degrees(
            poses.get(
                "install_joints_deg",
                self.get_parameter("arm_install_joints_deg").value,
            ),
            label="install joints",
        )
        keepout_min = safety.get(
            "rear_rack_keepout_min_xyz",
            self.get_parameter("rear_rack_keepout_min_xyz").value,
        )
        keepout_max = safety.get(
            "rear_rack_keepout_max_xyz",
            self.get_parameter("rear_rack_keepout_max_xyz").value,
        )
        payload_mass = float(
            payload_defaults.get(
                "mass_kg",
                self.get_parameter("aubo_payload_mass_kg").value,
            )
        )
        payload_cog = str(
            payload_defaults.get(
                "cog_m",
                self.get_parameter("aubo_payload_cog").value,
            )
        )
        payload_aom = str(
            payload_defaults.get(
                "aom",
                self.get_parameter("aubo_payload_aom").value,
            )
        )
        payload_inertia = str(
            payload_defaults.get(
                "inertia",
                self.get_parameter("aubo_payload_inertia").value,
            )
        )
        _parse_vector3(str(keepout_min))
        _parse_vector3(str(keepout_max))
        _parse_vector(payload_cog, expected=3, label="payload cog")
        _parse_vector(payload_aom, expected=3, label="payload aom")
        _parse_vector(payload_inertia, expected=6, label="payload inertia")

        self.set_parameters(
            [
                Parameter(
                    "base_linear_speed",
                    Parameter.Type.DOUBLE,
                    float(motion.get("base_linear_speed", self.get_parameter("base_linear_speed").value)),
                ),
                Parameter(
                    "base_angular_speed",
                    Parameter.Type.DOUBLE,
                    float(motion.get("base_angular_speed", self.get_parameter("base_angular_speed").value)),
                ),
                Parameter(
                    "arm_jog_step_m",
                    Parameter.Type.DOUBLE,
                    float(motion.get("arm_jog_step_m", self.get_parameter("arm_jog_step_m").value)),
                ),
                Parameter(
                    "arm_rotate_step_rad",
                    Parameter.Type.DOUBLE,
                    math.radians(
                        float(
                            motion.get(
                                "arm_rotate_step_deg",
                                math.degrees(
                                    float(self.get_parameter("arm_rotate_step_rad").value)
                                ),
                            )
                        )
                    ),
                ),
                Parameter(
                    "arm_joint_step_rad",
                    Parameter.Type.DOUBLE,
                    math.radians(
                        float(
                            motion.get(
                                "arm_joint_step_deg",
                                math.degrees(
                                    float(self.get_parameter("arm_joint_step_rad").value)
                                ),
                            )
                        )
                    ),
                ),
                Parameter(
                    "arm_hold_period_sec",
                    Parameter.Type.DOUBLE,
                    float(
                        motion.get(
                            "arm_hold_period_sec",
                            self.get_parameter("arm_hold_period_sec").value,
                        )
                    ),
                ),
                Parameter(
                    "arm_waypoint_duration_sec",
                    Parameter.Type.DOUBLE,
                    float(
                        motion.get(
                            "arm_waypoint_duration_sec",
                            self.get_parameter("arm_waypoint_duration_sec").value,
                        )
                    ),
                ),
                Parameter(
                    "gripper_settle_sec",
                    Parameter.Type.DOUBLE,
                    float(
                        motion.get(
                            "gripper_settle_sec",
                            self.get_parameter("gripper_settle_sec").value,
                        )
                    ),
                ),
                Parameter(
                    "arm_home_joints_deg",
                    Parameter.Type.STRING,
                    _format_joint_degrees(home_values),
                ),
                Parameter(
                    "arm_install_joints_deg",
                    Parameter.Type.STRING,
                    _format_joint_degrees(install_values),
                ),
                Parameter(
                    "aubo_payload_mass_kg",
                    Parameter.Type.DOUBLE,
                    payload_mass,
                ),
                Parameter(
                    "aubo_payload_cog",
                    Parameter.Type.STRING,
                    payload_cog,
                ),
                Parameter(
                    "aubo_payload_aom",
                    Parameter.Type.STRING,
                    payload_aom,
                ),
                Parameter(
                    "aubo_payload_inertia",
                    Parameter.Type.STRING,
                    payload_inertia,
                ),
                Parameter(
                    "arm_keepout_enabled",
                    Parameter.Type.BOOL,
                    _bool_value(
                        safety.get(
                            "arm_keepout_enabled",
                            self.get_parameter("arm_keepout_enabled").value,
                        )
                    ),
                ),
                Parameter(
                    "rear_rack_keepout_min_xyz",
                    Parameter.Type.STRING,
                    str(keepout_min),
                ),
                Parameter(
                    "rear_rack_keepout_max_xyz",
                    Parameter.Type.STRING,
                    str(keepout_max),
                ),
                Parameter(
                    "arm_keepout_sample_step_m",
                    Parameter.Type.DOUBLE,
                    float(
                        safety.get(
                            "arm_keepout_sample_step_m",
                            self.get_parameter("arm_keepout_sample_step_m").value,
                        )
                    ),
                ),
                Parameter(
                    "arm_keepout_joint_step_rad",
                    Parameter.Type.DOUBLE,
                    float(
                        safety.get(
                            "arm_keepout_joint_step_rad",
                            self.get_parameter("arm_keepout_joint_step_rad").value,
                        )
                    ),
                ),
            ]
        )

    def save_teach_config(self, path: str | Path | None = None, *, status: bool = True) -> Path:
        config_path = self.teach_config_path(path)
        if path is not None:
            self.set_parameters(
                [Parameter("teach_config_path", Parameter.Type.STRING, str(config_path))]
            )
        config_path.write_text(
            json.dumps(self.teach_config_payload(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        if status:
            self._status(f"config saved: {config_path}")
        return config_path

    def load_teach_config(self, path: str | Path | None = None, *, status: bool = True) -> bool:
        config_path = self.teach_config_path(path)
        if path is not None:
            self.set_parameters(
                [Parameter("teach_config_path", Parameter.Type.STRING, str(config_path))]
            )
        if not config_path.exists():
            if status:
                self._status(f"config not found: {config_path}", warn=True)
            return False
        try:
            payload = json.loads(config_path.read_text(encoding="utf-8"))
            self.apply_teach_config(payload)
        except Exception as exc:
            if status:
                self._status(f"config load failed: {exc}", warn=True)
            else:
                self.get_logger().warning(f"config autoload failed: {exc}")
            return False
        if status:
            self._status(f"config loaded: {config_path}")
        return True

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
        self.stop_arm_velocity_hold()
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - best effort stop path.
                self.get_logger().warning(f"arm cancel failed: {exc}")
            self._active_goal_handle = None
        self._status("arm stop requested")

    def hold_arm_current(self) -> None:
        self.cancel_event.set()
        self.stop_arm_velocity_hold()
        self._status("arm hold current requested")

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
            self.stop_arm_velocity_hold()
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

    def command_aubo_lifecycle(self, command: str) -> None:
        command = str(command).strip().lower()
        if command not in {"power_on", "startup", "power_off"}:
            self._status(f"aubo lifecycle ignored unknown command: {command}", warn=True)
            return
        self._start_worker(lambda: self._aubo_lifecycle_worker(command))

    def _aubo_lifecycle_worker(self, command: str) -> None:
        self.drive_base_manual("stop")
        self.stop_arm_velocity_hold()
        if command == "power_off":
            self.cancel_event.set()
        else:
            self.cancel_event.clear()
        ip = str(self.get_parameter("aubo_sdk_ip").value)
        port = int(self.get_parameter("aubo_sdk_rpc_port").value)
        timeout = max(float(self.get_parameter("aubo_sdk_rpc_timeout_sec").value), 0.1)
        power_timeout = max(
            float(self.get_parameter("aubo_sdk_lifecycle_power_timeout_sec").value), 1.0
        )
        startup_timeout = max(
            float(self.get_parameter("aubo_sdk_lifecycle_startup_timeout_sec").value), 1.0
        )
        poll = max(float(self.get_parameter("aubo_sdk_lifecycle_poll_sec").value), 0.05)
        try:
            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                mode = str(rpc.robot_call("RobotState.getRobotModeType"))
                safety = str(rpc.robot_call("RobotState.getSafetyModeType"))
                self._status(f"Aubo lifecycle {command}: mode={mode} safety={safety}")
                if command == "power_on":
                    if mode in {"Idle", "Running"}:
                        self._status(f"Aubo already {mode}")
                        return
                    self._publish_aubo_zero_velocity("before power_on")
                    result = rpc.robot_call("RobotManage.poweron")
                    self._status(f"Aubo power_on result={result}")
                    self._sdk_wait_mode(rpc, {"Idle", "Running"}, power_timeout, poll, "power_on")
                    return

                if command == "startup":
                    self._publish_aubo_zero_velocity("before startup")
                    if mode not in {"Idle", "Running"}:
                        result = rpc.robot_call("RobotManage.poweron")
                        self._status(f"Aubo power_on result={result}")
                        self._sdk_wait_mode(rpc, {"Idle", "Running"}, power_timeout, poll, "power_on")
                    mode = str(rpc.robot_call("RobotState.getRobotModeType"))
                    if mode != "Running":
                        result = rpc.robot_call("RobotManage.startup")
                        self._status(f"Aubo startup result={result}")
                        self._sdk_wait_mode(rpc, {"Running"}, startup_timeout, poll, "startup")
                    self._publish_aubo_zero_velocity("after startup")
                    self._status("Aubo startup complete")
                    return

                self._sdk_stop_joint(rpc, "power_off", warn_only=True)
                result = self._aubo_power_off_call(rpc)
                self._status(f"Aubo power_off result={result}")
        except Exception as exc:
            self._status(f"Aubo lifecycle {command} failed: {exc}", warn=True)

    def _publish_aubo_zero_velocity(self, reason: str) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.arm_command_joint_names)
        self.arm_velocity_pub.publish(msg)
        self._status(f"Aubo zero velocity hold: {reason}")

    def _aubo_power_off_call(self, rpc: AuboDirectJsonRpc) -> Any:
        last_error: Exception | None = None
        for method in ("RobotManage.poweroff", "RobotManage.powerOff"):
            try:
                return rpc.robot_call(method)
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"power_off RPC unavailable: {last_error}")

    def _sdk_wait_mode(
        self,
        rpc: AuboDirectJsonRpc,
        expected: set[str],
        timeout_sec: float,
        poll_sec: float,
        label: str,
    ) -> str:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        last_mode = ""
        last_safety = ""
        while time.monotonic() < deadline and not self.cancel_event.is_set():
            last_mode = str(rpc.robot_call("RobotState.getRobotModeType"))
            last_safety = str(rpc.robot_call("RobotState.getSafetyModeType"))
            if last_mode in expected:
                self._status(f"Aubo {label} reached {last_mode}")
                return last_mode
            time.sleep(max(float(poll_sec), 0.05))
        if self.cancel_event.is_set():
            raise RuntimeError(f"Aubo {label} cancelled")
        raise TimeoutError(
            f"Aubo {label} timeout: mode={last_mode or 'unknown'} safety={last_safety or 'unknown'}"
        )

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

    def call_grasp_task(self, command: str) -> None:
        command = str(command).strip().lower()
        if command not in self.grasp_task_clients:
            self._status(f"grasp task ignored unknown command: {command}", warn=True)
            return
        self._start_worker(lambda: self._call_grasp_task_worker(command))

    def call_cleanup_task(self, command: str) -> None:
        command = str(command).strip().lower()
        if command not in self.cleanup_task_clients:
            self._status(f"road cleanup ignored unknown command: {command}", warn=True)
            return
        self._start_worker(lambda: self._call_cleanup_task_worker(command))

    def visual_grasp_start(self) -> None:
        self._start_worker(self._visual_grasp_start_worker)

    def _visual_grasp_start_worker(self) -> None:
        self.drive_base_manual("stop")
        self.stop_arm_velocity_hold()
        if self._aubo_teach_gate_may_be_active():
            self._publish_aubo_teach_command("teach_off")
            ready = self.aubo_teach_ready_event.wait(
                max(float(self.get_parameter("aubo_teach_exit_wait_sec").value), 0.0)
            )
            if not ready or self._aubo_teach_gate_may_be_active():
                self._status("visual grasp blocked: Aubo teach mode still active", warn=True)
                return

        self._status("visual grasp: starting camera, raw view, and grasp server")
        self._start_managed_process_worker("camera")
        time.sleep(0.8)
        self._start_managed_process_worker("viewer")
        self._start_managed_process_worker("grasp_server")

        preflight = self.grasp_task_clients.get("preflight")
        if preflight is None:
            self._status("visual grasp blocked: missing preflight client", warn=True)
            return
        service_name = getattr(preflight, "srv_name", "preflight")
        if not preflight.wait_for_service(timeout_sec=15.0):
            self._status(f"visual grasp blocked: preflight unavailable: {service_name}", warn=True)
            return

        deadline = time.monotonic() + 30.0
        last_message = ""
        while time.monotonic() < deadline:
            future = preflight.call_async(Trigger.Request())
            if not self._wait_service_future(future, 5.0):
                last_message = f"preflight timeout: {service_name}"
                self._status(f"visual grasp: {last_message}", warn=True)
                time.sleep(0.5)
                continue
            response = future.result()
            success = bool(response.success) if response is not None else False
            message = response.message if response is not None else "empty response"
            if success:
                self._status("visual grasp: preflight ok, starting task")
                self._call_grasp_task_worker("start")
                return

            failures = self._required_preflight_failures(message)
            last_message = self._format_preflight_failures(failures, message)
            failure_names = {str(item.get("name", "")) for item in failures}
            if failure_names and failure_names <= {"color_image", "depth_image"}:
                self._status(f"visual grasp: waiting for camera/depth ({last_message})")
                time.sleep(0.8)
                continue
            self._status(f"visual grasp blocked: {last_message}", warn=True)
            return

        self._status(f"visual grasp blocked: preflight never became ready ({last_message})", warn=True)

    def _call_grasp_task_worker(self, command: str) -> None:
        client = self.grasp_task_clients.get(command)
        if client is None:
            self._status(f"grasp task service missing: {command}", warn=True)
            return
        if command == "start":
            self.drive_base_manual("stop")
            self.stop_arm_velocity_hold()
            if self._aubo_teach_gate_may_be_active():
                self._publish_aubo_teach_command("teach_off")
                ready = self.aubo_teach_ready_event.wait(
                    max(float(self.get_parameter("aubo_teach_exit_wait_sec").value), 0.0)
                )
                if not ready or self._aubo_teach_gate_may_be_active():
                    self._status("grasp task start blocked: Aubo teach mode still active", warn=True)
                    return
        if command in {"stop", "restore"}:
            self.drive_base_manual("stop")
            self.stop_arm_velocity_hold()

        service_name = getattr(client, "srv_name", command)
        self._status(f"grasp task {command} requested")
        if not client.wait_for_service(timeout_sec=1.5):
            self._status(f"grasp task {command} unavailable: {service_name}", warn=True)
            return
        future = client.call_async(Trigger.Request())
        if not self._wait_service_future(future, 8.0):
            self._status(f"grasp task {command} timeout: {service_name}", warn=True)
            return
        response = future.result()
        success = bool(response.success) if response is not None else False
        message = response.message if response is not None else "empty response"
        self._status(
            f"grasp task {command} {'ok' if success else 'failed'}: "
            f"{self._short_grasp_task_message(message)}",
            warn=not success,
        )

    def _call_cleanup_task_worker(self, command: str) -> None:
        client = self.cleanup_task_clients.get(command)
        if client is None:
            self._status(f"road cleanup service missing: {command}", warn=True)
            return
        if command == "start":
            self.drive_base_manual("stop")
            self.stop_arm_velocity_hold()
            if self._aubo_teach_gate_may_be_active():
                self._publish_aubo_teach_command("teach_off")
                ready = self.aubo_teach_ready_event.wait(
                    max(float(self.get_parameter("aubo_teach_exit_wait_sec").value), 0.0)
                )
                if not ready or self._aubo_teach_gate_may_be_active():
                    self._status("road cleanup blocked: Aubo teach mode still active", warn=True)
                    return
            self.start_camera_stack()
            self._start_managed_process_worker("grasp_server")
            self._start_managed_process_worker("cleanup_server")
            self._status("road cleanup: waiting for server startup")
        elif command in {"pause", "return_home", "stop"}:
            self.drive_base_manual("stop")
            self.stop_arm_velocity_hold()

        service_name = getattr(client, "srv_name", command)
        self._status(f"road cleanup {command} requested")
        wait_timeout = 75.0 if command == "start" else 4.0
        if not client.wait_for_service(timeout_sec=wait_timeout):
            self._status(f"road cleanup {command} unavailable: {service_name}", warn=True)
            return
        future = client.call_async(Trigger.Request())
        response_timeout = 12.0 if command == "return_home" else 8.0
        if not self._wait_service_future(future, response_timeout):
            self._status(f"road cleanup {command} timeout: {service_name}", warn=True)
            return
        response = future.result()
        success = bool(response.success) if response is not None else False
        message = response.message if response is not None else "empty response"
        self._status(
            f"road cleanup {command} {'ok' if success else 'failed'}: "
            f"{self._short_grasp_task_message(message)}",
            warn=not success,
        )

    def _wait_service_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _short_grasp_task_message(self, message: str) -> str:
        text = str(message).strip()
        if not text:
            return ""
        try:
            payload = json.loads(text)
            state = str(payload.get("state", "")).strip()
            detail = str(payload.get("message", "")).strip()
            task_id = str(payload.get("task_id", "")).strip()
            parts = [part for part in (state, detail, task_id) if part]
            if parts:
                return " | ".join(parts)
        except json.JSONDecodeError:
            pass
        return text[:180]

    def _required_preflight_failures(self, message: str) -> list[dict[str, Any]]:
        try:
            payload = json.loads(str(message))
        except json.JSONDecodeError:
            return [{"name": "preflight", "message": str(message), "required": True}]
        checks = payload.get("checks", [])
        if not isinstance(checks, list):
            return [{"name": "preflight", "message": str(message), "required": True}]
        failures: list[dict[str, Any]] = []
        for item in checks:
            if not isinstance(item, dict):
                continue
            if bool(item.get("required", True)) and not bool(item.get("ok", False)):
                failures.append(item)
        return failures

    def _format_preflight_failures(
        self, failures: list[dict[str, Any]], fallback: str
    ) -> str:
        if not failures:
            return self._short_grasp_task_message(fallback)
        parts = []
        for item in failures[:4]:
            name = str(item.get("name", "check")).strip()
            message = str(item.get("message", "")).strip()
            parts.append(f"{name}: {message}" if message else name)
        if len(failures) > 4:
            parts.append(f"+{len(failures) - 4} more")
        return "; ".join(parts)

    def jog_arm(self, axis: str, sign: float) -> None:
        self.cancel_event.clear()
        if axis not in ("x", "y", "z"):
            self._status(f"arm velocity jog skipped: unknown axis {axis}", warn=True)
            self.stop_arm_velocity_hold()
            return
        self._set_manual_arm_stream_command(
            {"kind": "cartesian", "axis": axis, "sign": float(sign)},
            f"{axis}{'+' if sign > 0 else '-'}",
        )

    def _track_base_motion(self, direction: str, linear_x: float, angular_z: float) -> None:
        now = time.monotonic()
        with self.lock:
            if not self.base_motion_recording_enabled:
                self._close_base_motion_locked(now)
                return
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

    def set_base_motion_recording(self, enabled: bool) -> None:
        now = time.monotonic()
        with self.lock:
            self._close_base_motion_locked(now)
            self.base_motion_recording_enabled = bool(enabled)
            count = len(self.base_motion_segments)
        state = "on" if enabled else "off"
        self._status(f"program base recording {state}; pending base segments={count}")

    def base_motion_recording_state(self) -> tuple[bool, int]:
        with self.lock:
            return (
                self.base_motion_recording_enabled,
                len(self.base_motion_segments) + (1 if self.active_base_motion is not None else 0),
            )

    def _publish_manual_base_velocity(self) -> None:
        with self.lock:
            velocity = self.manual_base_velocity
        if velocity is None:
            return
        self.set_base_velocity(velocity[0], velocity[1])

    def _publish_manual_arm_velocity(self) -> None:
        now = time.monotonic()
        with self.lock:
            dt = max(0.001, min(now - self.arm_velocity_last_update, 0.10))
            self.arm_velocity_last_update = now
            generation = self.arm_velocity_command_generation
            velocity = list(self.manual_arm_velocity) if self.manual_arm_velocity is not None else None
            expired = velocity is not None and now > self.manual_arm_velocity_deadline
            if expired:
                self.manual_arm_velocity = None
                self.manual_arm_velocity_deadline = 0.0
        stream_velocity = self._manual_arm_stream_velocity(now)
        if stream_velocity is not None:
            velocity, generation = stream_velocity
            expired = False
        if velocity is None:
            return
        if expired:
            self.stop_arm_velocity_hold()
            return
        with self.lock:
            if generation != self.arm_velocity_command_generation:
                return
        self._publish_arm_velocity(self._smooth_arm_velocity(velocity, dt))

    def _publish_arm_velocity(self, velocity: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in velocity]
        self.arm_velocity_pub.publish(msg)

    def _publish_arm_velocity_zero(self) -> None:
        self._publish_arm_velocity([0.0 for _ in self.arm_command_joint_names])

    def _set_manual_arm_stream_command(self, command: dict[str, object], label: str) -> None:
        now = time.monotonic()
        deadman = max(float(self.get_parameter("arm_velocity_stream_deadman_sec").value), 0.08)
        with self.lock:
            self.arm_velocity_command_generation += 1
            self.manual_arm_stream_command = dict(command, label=label)
            self.manual_arm_stream_deadline = now + deadman
            self.manual_arm_velocity = None
            self.manual_arm_velocity_deadline = 0.0
        if now - self._last_manual_arm_stream_status >= 0.8:
            self._status(f"arm velocity stream: {label}")
            self._last_manual_arm_stream_status = now

    def refresh_arm_velocity_stream_deadman(self) -> None:
        deadman = max(float(self.get_parameter("arm_velocity_stream_deadman_sec").value), 0.08)
        with self.lock:
            if self.manual_arm_stream_command is not None:
                self.manual_arm_stream_deadline = time.monotonic() + deadman

    def _manual_arm_stream_velocity(self, now: float) -> tuple[list[float], int] | None:
        with self.lock:
            command = (
                dict(self.manual_arm_stream_command)
                if self.manual_arm_stream_command is not None
                else None
            )
            deadline = self.manual_arm_stream_deadline
            generation = self.arm_velocity_command_generation
        if command is None:
            return None
        if now > deadline:
            self.stop_arm_velocity_hold()
            self._status("arm velocity stream timeout: deadman hold", warn=True)
            return None

        label = str(command.get("label", "stream"))
        kind = str(command.get("kind", ""))
        sign = float(command.get("sign", 0.0))
        current = self._current_arm_vector()
        if current is None:
            self._status(f"arm velocity skipped: no joint state for {label}", warn=True)
            self.stop_arm_velocity_hold()
            return None

        velocity: list[float] | None = None
        if kind == "cartesian":
            axes = {
                "x": np.array([1.0, 0.0, 0.0], dtype=float),
                "y": np.array([0.0, 1.0, 0.0], dtype=float),
                "z": np.array([0.0, 0.0, 1.0], dtype=float),
            }
            axis = str(command.get("axis", ""))
            if axis not in axes:
                self._status(f"arm velocity skipped: unknown axis {axis}", warn=True)
                self.stop_arm_velocity_hold()
                return None
            speed = max(float(self.get_parameter("arm_cartesian_jog_speed_m_sec").value), 0.0)
            twist = np.zeros(6, dtype=float)
            twist[:3] = axes[axis] * sign * speed
            velocity = self._joint_velocity_from_twist(np.array(current, dtype=float), twist)
        elif kind == "rotation":
            axes = {
                "rx": np.array([1.0, 0.0, 0.0], dtype=float),
                "ry": np.array([0.0, 1.0, 0.0], dtype=float),
                "rz": np.array([0.0, 0.0, 1.0], dtype=float),
            }
            axis = str(command.get("axis", ""))
            if axis not in axes:
                self._status(f"arm rotation velocity skipped: unknown axis {axis}", warn=True)
                self.stop_arm_velocity_hold()
                return None
            speed = max(float(self.get_parameter("arm_cartesian_rotate_speed_rad_sec").value), 0.0)
            twist = np.zeros(6, dtype=float)
            twist[3:] = axes[axis] * sign * speed
            velocity = self._joint_velocity_from_twist(np.array(current, dtype=float), twist)
        elif kind == "joint":
            index = int(command.get("index", -1))
            if index < 0 or index >= len(self.arm_command_joint_names):
                self._status(f"joint velocity skipped: invalid joint {index + 1}", warn=True)
                self.stop_arm_velocity_hold()
                return None
            speed = max(float(self.get_parameter("arm_joint_jog_speed_rad_sec").value), 0.0)
            velocity = [0.0 for _ in self.arm_command_joint_names]
            velocity[index] = sign * speed
        else:
            self._status(f"arm velocity skipped: unknown stream kind {kind}", warn=True)
            self.stop_arm_velocity_hold()
            return None

        if velocity is None:
            self.stop_arm_velocity_hold()
            return None
        clamped = self._validated_arm_velocity(velocity, label)
        if clamped is None:
            self.stop_arm_velocity_hold()
            return None
        return clamped, generation

    def _validated_arm_velocity(self, velocity: list[float], label: str) -> list[float] | None:
        if len(velocity) != len(self.arm_command_joint_names):
            self._status(f"arm velocity ignored: invalid length for {label}", warn=True)
            return None
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        clamped = [
            max(-max_speed, min(max_speed, float(value)))
            if math.isfinite(float(value))
            else 0.0
            for value in velocity
        ]
        current = self._current_arm_vector()
        if current is None:
            self._status(f"arm velocity skipped: no joint state for {label}", warn=True)
            return None
        now = time.monotonic()
        keepout_interval = max(
            float(self.get_parameter("arm_velocity_keepout_check_interval_sec").value), 0.0
        )
        with self.lock:
            cached_violation = self.arm_velocity_keepout_last_violation
            next_check_due = now - self.arm_velocity_keepout_last_check >= keepout_interval
        if next_check_due:
            violation = self._arm_velocity_keepout_violation(np.array(current, dtype=float), clamped, label)
            with self.lock:
                self.arm_velocity_keepout_last_check = now
                self.arm_velocity_keepout_last_violation = violation
        else:
            violation = cached_violation
        if violation is not None:
            self._status(f"arm velocity blocked by safety zone: {violation}", warn=True)
            return None
        return clamped

    def _smooth_arm_velocity(self, velocity: list[float], dt: float) -> list[float]:
        target = np.array([float(value) for value in velocity], dtype=float)
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        max_accel = max(
            float(self.get_parameter("arm_velocity_max_joint_accel_rad_sec2").value), 0.05
        )
        max_jerk = max(
            float(self.get_parameter("arm_velocity_max_joint_jerk_rad_sec3").value), 0.1
        )
        tau = max(float(self.get_parameter("arm_velocity_smoothing_tau_sec").value), 0.02)
        target = np.clip(target, -max_speed, max_speed)

        with self.lock:
            current = np.array(self.arm_velocity_filtered, dtype=float)
            accel = np.array(self.arm_velocity_accel, dtype=float)

            desired_accel = np.clip((target - current) / tau, -max_accel, max_accel)
            accel_delta = np.clip(desired_accel - accel, -max_jerk * dt, max_jerk * dt)
            accel = accel + accel_delta
            next_velocity = current + accel * dt

            for index, (start, end, goal) in enumerate(zip(current, next_velocity, target)):
                if (goal - start) * (goal - end) <= 0.0 and abs(goal - start) > 1e-9:
                    next_velocity[index] = goal
                    accel[index] = 0.0

            next_velocity = np.clip(next_velocity, -max_speed, max_speed)
            self.arm_velocity_filtered = [float(value) for value in next_velocity]
            self.arm_velocity_accel = [float(value) for value in accel]
            return list(self.arm_velocity_filtered)

    def _set_manual_arm_velocity(self, velocity: list[float], label: str) -> None:
        clamped = self._validated_arm_velocity(velocity, label)
        if clamped is None:
            self.stop_arm_velocity_hold()
            return
        watchdog = max(float(self.get_parameter("arm_velocity_watchdog_sec").value), 0.05)
        with self.lock:
            self.arm_velocity_command_generation += 1
            self.manual_arm_stream_command = None
            self.manual_arm_velocity = clamped
            self.manual_arm_velocity_deadline = time.monotonic() + watchdog
        now = time.monotonic()
        if now - self._last_manual_arm_stream_status >= 0.8:
            self._status(f"arm velocity jog: {label}")
            self._last_manual_arm_stream_status = now

    def stop_arm_velocity_hold(self) -> None:
        with self.lock:
            self.arm_velocity_command_generation += 1
            self.manual_arm_stream_command = None
            self.manual_arm_stream_deadline = 0.0
            self.manual_arm_velocity = None
            self.manual_arm_velocity_deadline = 0.0
            self.arm_velocity_filtered = [0.0 for _ in self.arm_command_joint_names]
            self.arm_velocity_accel = [0.0 for _ in self.arm_command_joint_names]
            self.arm_velocity_last_update = time.monotonic()
            self.arm_velocity_keepout_last_check = 0.0
            self.arm_velocity_keepout_last_violation = None
        for _ in range(3):
            self._publish_arm_velocity_zero()

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
        q_start_array = np.array(q_start, dtype=float)
        target_transform = np.array(self.kinematics.fk(q_start_array), dtype=float)
        target_transform[:3, 3] += direction * step
        tolerance = min(
            float(self.get_parameter("arm_jog_position_tolerance").value),
            max(step * 0.25, 1e-4),
        )
        ok, q_target, position_error, orientation_error, iterations = self.kinematics.solve_pose(
            q_start_array,
            target_transform,
            position_tolerance=tolerance,
            orientation_tolerance=float(self.get_parameter("arm_jog_orientation_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_target - q_start_array)))
        if not ok:
            self._status(
                "arm jog pose IK failed: "
                f"pos={position_error:.4f}m rot={math.degrees(orientation_error):.2f}deg "
                f"iterations={iterations}",
                warn=True,
            )
            return
        if max_delta > float(self.get_parameter("arm_jog_max_joint_delta").value):
            self._status(f"arm jog blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        duration = float(self.get_parameter("arm_jog_duration_sec").value)
        self._send_arm_positions(
            [float(value) for value in q_target],
            duration,
            f"jog {axis}",
            wait=False,
            velocities=self._target_joint_velocities(q_start, q_target, duration),
            manual_stream=True,
        )

    def jog_arm_rotation(self, axis: str, sign: float) -> None:
        self.cancel_event.clear()
        if axis not in ("rx", "ry", "rz"):
            self._status(f"arm rotation velocity skipped: unknown axis {axis}", warn=True)
            self.stop_arm_velocity_hold()
            return
        self._set_manual_arm_stream_command(
            {"kind": "rotation", "axis": axis, "sign": float(sign)},
            f"{axis}{'+' if sign > 0 else '-'}",
        )

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
        duration = float(self.get_parameter("arm_rotate_duration_sec").value)
        self._send_arm_positions(
            q_target,
            duration,
            f"rotate {axis}",
            wait=False,
            velocities=self._target_joint_velocities(q_start, q_target, duration),
            manual_stream=True,
        )

    def jog_arm_joint(self, index: int, sign: float) -> None:
        self.cancel_event.clear()
        if index < 0 or index >= len(self.arm_command_joint_names):
            self._status(f"joint velocity skipped: invalid joint {index + 1}", warn=True)
            self.stop_arm_velocity_hold()
            return
        self._set_manual_arm_stream_command(
            {"kind": "joint", "index": int(index), "sign": float(sign)},
            f"J{index + 1}{'+' if sign > 0 else '-'}",
        )

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
        duration = float(self.get_parameter("arm_rotate_duration_sec").value)
        self._send_arm_positions(
            q_target,
            duration,
            f"jog J{index + 1}",
            wait=False,
            velocities=self._target_joint_velocities(q_start, q_target, duration),
            manual_stream=True,
        )

    def move_arm_to_joints_degrees(self, degrees: list[float]) -> None:
        self.cancel_event.clear()
        self._start_worker(lambda: self._move_arm_to_joints_worker(degrees))

    def home_joints_degrees(self) -> list[float]:
        return _parse_joint_degrees(
            str(self.get_parameter("arm_home_joints_deg").value),
            label="arm_home_joints_deg",
        )

    def install_joints_degrees(self) -> list[float]:
        return _parse_joint_degrees(
            str(self.get_parameter("arm_install_joints_deg").value),
            label="arm_install_joints_deg",
        )

    def preset_joints_degrees(self, preset: str) -> list[float]:
        if preset == "home":
            return self.home_joints_degrees()
        if preset == "install":
            return self.install_joints_degrees()
        raise ValueError(f"unknown preset: {preset}")

    def move_arm_home(self) -> None:
        self.move_arm_preset("home")

    def move_arm_install(self) -> None:
        self.move_arm_preset("install")

    def move_arm_preset(self, preset: str) -> None:
        try:
            target = self.preset_joints_degrees(preset)
        except Exception as exc:
            self._status(f"{preset} skipped: {exc}", warn=True)
            return
        self.cancel_event.clear()
        self._start_worker(lambda: self._move_arm_to_joints_worker(target, label=preset))

    def _move_arm_to_joints_worker(self, degrees: list[float], label: str = "joint target") -> None:
        if len(degrees) != 6:
            self._status("joint target skipped: expected 6 joint angles", warn=True)
            return
        target = [math.radians(float(value)) for value in degrees]
        self._move_arm_to_positions_velocity(target, label)

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
        q_start_array = np.array(q_start, dtype=float)
        target_transform = np.array(self.kinematics.fk(q_start_array), dtype=float)
        target_transform[:3, 3] = np.array([float(value) for value in position], dtype=float)
        ok, q_target, position_error, orientation_error, iterations = self.kinematics.solve_pose(
            q_start_array,
            target_transform,
            position_tolerance=float(self.get_parameter("arm_position_tolerance").value),
            orientation_tolerance=float(self.get_parameter("arm_orientation_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_target - q_start_array)))
        if not ok:
            self._status(
                "TCP target pose IK failed: "
                f"pos={position_error:.4f}m rot={math.degrees(orientation_error):.2f}deg "
                f"iterations={iterations}",
                warn=True,
            )
            return
        if max_delta > float(self.get_parameter("arm_target_max_joint_delta").value):
            self._status(f"TCP target blocked: joint delta {max_delta:.3f} rad", warn=True)
            return
        self._move_arm_to_positions_velocity([float(value) for value in q_target], "TCP target")

    def _move_arm_to_positions_velocity(self, target: list[float], label: str) -> bool:
        if len(target) != 6:
            self._status(f"{label} skipped: expected 6 joint positions", warn=True)
            return False
        violation = self._arm_keepout_violation(target, label)
        if violation is not None:
            self._status(f"arm target blocked by safety zone: {violation}", warn=True)
            self.stop_arm_velocity_hold()
            return False
        target_array = np.array([float(value) for value in target], dtype=float)
        tolerance = max(float(self.get_parameter("arm_goal_tolerance").value), 0.005)
        gain = 0.85
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        rate = max(float(self.get_parameter("arm_velocity_publish_rate").value), 1.0)
        period = 1.0 / rate
        duration = float(self.get_parameter("arm_waypoint_duration_sec").value)
        deadline = time.monotonic() + max(duration + 8.0, 4.0)
        self._status(f"arm velocity target started: {label}")
        try:
            while not self.cancel_event.is_set() and time.monotonic() < deadline:
                current = self._current_arm_vector()
                if current is None:
                    self._status(f"{label} skipped: no joint state", warn=True)
                    return False
                current_array = np.array(current, dtype=float)
                error = np.array(
                    [
                        _angle_diff(float(target_value), float(current_value))
                        for target_value, current_value in zip(target_array, current_array)
                    ],
                    dtype=float,
                )
                max_error = float(np.max(np.abs(error)))
                if max_error <= tolerance:
                    self._status(f"arm velocity target reached: {label}")
                    return True
                velocity = np.clip(error * gain, -max_speed, max_speed)
                self._set_manual_arm_velocity([float(value) for value in velocity], label)
                time.sleep(period)
            self._status(f"arm velocity target timeout/cancel: {label}", warn=True)
            return False
        finally:
            self.stop_arm_velocity_hold()

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
            if self._replay_has_arm_targets(waypoints) and not self._ensure_aubo_motion_ready():
                raise RuntimeError("Aubo teach mode is still active")
            replay_gripper = self.gripper_state
            previous_arm_joints: list[float] | None = None
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
                if self._valid_arm_target(waypoint.arm_joints):
                    if previous_arm_joints is None:
                        previous_arm_joints = self._current_arm_vector()
                    if previous_arm_joints is None:
                        raise RuntimeError(f"missing arm joint state before {waypoint.label}")
                    target_arm_joints = [float(value) for value in waypoint.arm_joints]
                    arm_delta = self._max_arm_delta(previous_arm_joints, target_arm_joints)
                    skip_tolerance = max(
                        float(self.get_parameter("arm_goal_tolerance").value) * 0.5,
                        1e-4,
                    )
                    if arm_delta > skip_tolerance:
                        if not self._replay_arm_waypoint(
                            previous_arm_joints, target_arm_joints, waypoint.label
                        ):
                            raise RuntimeError(f"arm waypoint failed: {waypoint.label}")
                    else:
                        self._status(
                            f"arm replay skipped: unchanged target for {waypoint.label}"
                        )
                    previous_arm_joints = target_arm_joints
                elif waypoint.arm_joints:
                    self._status(
                        f"arm replay skipped: invalid joint target for {waypoint.label}",
                        warn=True,
                    )
                if waypoint.gripper in ("open", "close") and waypoint.gripper != replay_gripper:
                    self.publish_gripper(waypoint.gripper)
                    replay_gripper = waypoint.gripper
                    gripper_wait = max(float(self.get_parameter("gripper_settle_sec").value), 0.0)
                    if gripper_wait > 0.0:
                        self._status(f"gripper settle {gripper_wait:.1f}s: {waypoint.gripper}")
                        self._sleep(gripper_wait)
                self._sleep(float(self.get_parameter("replay_settle_sec").value))
            self.set_base_velocity(0.0, 0.0)
            self._status("replay complete" if not self.cancel_event.is_set() else "replay stopped")
        except Exception as exc:
            self.set_base_velocity(0.0, 0.0)
            self._status(f"replay failed: {exc}", warn=True)

    def _replay_has_arm_targets(self, waypoints: list[TeachWaypoint]) -> bool:
        return any(
            waypoint.kind != "wait" and self._valid_arm_target(waypoint.arm_joints)
            for waypoint in waypoints
        )

    def _valid_arm_target(self, target: list[float]) -> bool:
        if not isinstance(target, list):
            return False
        if len(target) != len(self.arm_command_joint_names):
            return False
        try:
            values = [float(value) for value in target]
        except (TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values)

    def _max_arm_delta(self, start: list[float], target: list[float]) -> float:
        if len(start) != len(target):
            return math.inf
        return max(
            (
                abs(_angle_diff(float(target_value), float(start_value)))
                for start_value, target_value in zip(start, target)
            ),
            default=0.0,
        )

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

    def _target_joint_velocities(
        self, start: list[float], target: list[float] | np.ndarray, duration: float
    ) -> list[float]:
        safe_duration = max(float(duration), 1e-3)
        return [
            (float(target_value) - float(start_value)) / safe_duration
            for start_value, target_value in zip(start, target)
        ]

    def _joint_velocity_from_twist(self, q: np.ndarray, twist: np.ndarray) -> list[float] | None:
        try:
            jacobian = self._numeric_twist_jacobian(q)
            damping = max(float(self.get_parameter("arm_velocity_damping").value), 1e-4)
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(6)
            qdot = jacobian.T @ np.linalg.solve(lhs, twist)
        except Exception as exc:
            self._status(f"arm velocity IK failed: {exc}", warn=True)
            return None
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

    def _arm_velocity_keepout_violation(
        self, q_start: np.ndarray, velocity: list[float], label: str
    ) -> str | None:
        if not bool(self.get_parameter("arm_keepout_enabled").value):
            return None
        qdot = np.array(velocity, dtype=float)
        if q_start.shape[0] != 6 or qdot.shape[0] != 6:
            return "invalid arm velocity length"
        predict_sec = max(float(self.get_parameter("arm_velocity_keepout_predict_sec").value), 0.05)
        try:
            boxes = self._arm_keepout_boxes()
            samples_count = max(2, int(math.ceil(predict_sec / 0.05)))
            for path_index in range(1, samples_count + 1):
                q = q_start + qdot * predict_sec * (path_index / samples_count)
                for sample_name, point in self._arm_keepout_samples(q):
                    for box in boxes:
                        if bool(np.all(point >= box.minimum) and np.all(point <= box.maximum)):
                            xyz = ",".join(f"{value:.3f}" for value in point)
                            return (
                                f"{label} predicts {box.name}: {sample_name} "
                                f"at base_link xyz=({xyz}), sample {path_index}/{samples_count}"
                            )
        except Exception as exc:
            return f"keepout config invalid: {exc}"
        return None

    def _send_arm_positions(
        self,
        positions: list[float],
        duration: float,
        label: str,
        *,
        wait: bool,
        velocities: list[float] | None = None,
        accelerations: list[float] | None = None,
        manual_stream: bool = False,
    ) -> bool:
        if label != "hold current":
            violation = self._arm_keepout_violation(positions, label)
            if violation is not None:
                self._status(f"arm motion blocked by safety zone: {violation}", warn=True)
                return False

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_command_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = (
            [float(value) for value in velocities]
            if velocities is not None and len(velocities) == len(point.positions)
            else [0.0 for _ in point.positions]
        )
        point.accelerations = (
            [float(value) for value in accelerations]
            if accelerations is not None and len(accelerations) == len(point.positions)
            else [0.0 for _ in point.positions]
        )
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        if (
            manual_stream
            and not wait
            and bool(self.get_parameter("arm_manual_prefer_topic").value)
            and self.arm_publishers
        ):
            for publisher in self.arm_publishers:
                publisher.publish(trajectory)
            now = time.monotonic()
            if label == "hold current" or now - self._last_manual_arm_stream_status >= 0.8:
                self._status(f"arm stream sent: {label}")
                self._last_manual_arm_stream_status = now
            return True

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

    def _replay_arm_waypoint(self, start: list[float], target: list[float], label: str) -> bool:
        backend = str(self.get_parameter("arm_replay_backend").value).strip().lower()
        if backend in ("sdk", "sdk_move_joint", "movejoint", "move_joint"):
            return self._send_arm_sdk_move_joint(target, label)
        if backend in ("velocity", "velocity_stream", "stream"):
            return self._send_arm_replay_segment(start, target, label)
        self._status(f"unknown arm replay backend {backend!r}; using sdk_move_joint", warn=True)
        return self._send_arm_sdk_move_joint(target, label)

    def _send_arm_sdk_move_joint(self, target: list[float], label: str) -> bool:
        if len(target) != len(self.arm_command_joint_names):
            self._status(f"Aubo SDK moveJoint skipped: invalid joint target for {label}", warn=True)
            return False
        violation = self._arm_keepout_violation(target, label)
        if violation is not None:
            self._status(f"Aubo SDK moveJoint blocked by safety zone: {violation}", warn=True)
            return False

        ip = str(self.get_parameter("aubo_sdk_ip").value)
        port = int(self.get_parameter("aubo_sdk_rpc_port").value)
        timeout = max(float(self.get_parameter("aubo_sdk_rpc_timeout_sec").value), 0.1)
        speed = max(float(self.get_parameter("aubo_sdk_move_speed_rad_sec").value), 0.01)
        accel = max(float(self.get_parameter("aubo_sdk_move_accel_rad_sec2").value), 0.05)
        blend_radius = max(float(self.get_parameter("aubo_sdk_blend_radius").value), 0.0)
        duration = max(float(self.get_parameter("aubo_sdk_move_duration_sec").value), 0.0)
        owner_owned = False
        gate_owned = False

        self._status(
            f"Aubo SDK moveJoint: {label} speed={speed:.3f} accel={accel:.3f}"
        )
        try:
            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                self._sdk_require_running(rpc)
                owner_owned = self._sdk_enter_control_owner()
                gate_owned = self._sdk_enter_gate()
                self._sdk_exit_servo_mode(rpc)
                self._sdk_stop_joint(rpc, "pre-move cleanup", warn_only=True)
                result = rpc.robot_call(
                    "MotionControl.moveJoint",
                    [[float(value) for value in target], accel, speed, blend_radius, duration],
                )
                if result not in (0, None):
                    self._status(f"Aubo SDK moveJoint failed at {label}: result={result}", warn=True)
                    return False
                self._sdk_wait_exec_complete(rpc, label)
                return self._sdk_wait_arrival(rpc, np.asarray(target, dtype=float), label)
        except Exception as exc:
            self._status(f"Aubo SDK moveJoint failed at {label}: {exc}", warn=True)
            return False
        finally:
            if gate_owned:
                try:
                    with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                        self._sdk_stop_joint(rpc, "post-move cleanup", warn_only=True)
                except Exception:
                    pass
            self._sdk_exit_gate(gate_owned)
            self._sdk_exit_control_owner(owner_owned)

    def _sdk_require_running(self, rpc: AuboDirectJsonRpc) -> None:
        mode = str(rpc.robot_call("RobotState.getRobotModeType")).strip().lower()
        safety = str(rpc.robot_call("RobotState.getSafetyModeType")).strip().lower()
        if mode != "running" or safety not in ("normal", "reducedmode"):
            raise RuntimeError(f"Aubo not ready: mode={mode} safety={safety}")

    def _sdk_enter_control_owner(self) -> bool:
        path = Path(str(self.get_parameter("aubo_sdk_control_owner_path").value))
        owner = str(self.get_parameter("aubo_sdk_control_owner_name").value).strip() or "teach_panel"
        ok, message = _claim_control_owner(path, owner)
        if not ok:
            raise RuntimeError(f"Aubo control owner unavailable: {message}")
        return True

    def _sdk_exit_control_owner(self, owner_owned: bool) -> None:
        if not owner_owned:
            return
        path = Path(str(self.get_parameter("aubo_sdk_control_owner_path").value))
        owner = str(self.get_parameter("aubo_sdk_control_owner_name").value).strip() or "teach_panel"
        _release_control_owner(path, owner)

    def _sdk_enter_gate(self) -> bool:
        path = Path(str(self.get_parameter("aubo_sdk_teach_flag_path").value))
        try:
            path.write_text("1\n", encoding="utf-8")
        except OSError as exc:
            raise RuntimeError(f"failed to set Aubo teach gate {path}: {exc}") from exc
        time.sleep(0.15)
        return True

    def _sdk_exit_gate(self, gate_owned: bool) -> None:
        if not gate_owned:
            return
        path = Path(str(self.get_parameter("aubo_sdk_teach_flag_path").value))
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            self._status(f"failed to release Aubo teach gate {path}: {exc}", warn=True)

    def _sdk_exit_servo_mode(self, rpc: AuboDirectJsonRpc) -> None:
        try:
            result = rpc.robot_call("MotionControl.setServoModeSelect", [0])
            if result not in (0, None):
                self._status(f"Aubo SDK setServoModeSelect(0) result={result}", warn=True)
            return
        except Exception as exc:
            self._status(
                f"Aubo SDK setServoModeSelect unavailable, trying setServoMode(false): {exc}",
                warn=True,
            )
        result = rpc.robot_call("MotionControl.setServoMode", [False])
        if result not in (0, None):
            self._status(f"Aubo SDK setServoMode(false) result={result}", warn=True)

    def _sdk_stop_joint(
        self, rpc: AuboDirectJsonRpc, reason: str, *, warn_only: bool = False
    ) -> None:
        accel = max(float(self.get_parameter("aubo_sdk_move_accel_rad_sec2").value), 0.05)
        try:
            result = rpc.robot_call("MotionControl.stopJoint", [accel])
        except Exception as exc:
            if warn_only:
                self._status(f"Aubo SDK stopJoint failed during {reason}: {exc}", warn=True)
                return
            raise
        if result not in (0, None):
            message = f"Aubo SDK stopJoint result={result} during {reason}"
            if warn_only:
                self._status(message, warn=True)
            else:
                raise RuntimeError(message)

    def _sdk_wait_exec_complete(self, rpc: AuboDirectJsonRpc, label: str) -> None:
        timeout = max(float(self.get_parameter("arm_waypoint_duration_sec").value), 0.5) + 8.0
        deadline = time.monotonic() + timeout
        exec_id = rpc.robot_call("MotionControl.getExecId")
        start_deadline = time.monotonic() + 0.5
        while exec_id == -1 and time.monotonic() < start_deadline and not self.cancel_event.is_set():
            time.sleep(0.05)
            exec_id = rpc.robot_call("MotionControl.getExecId")
        while exec_id != -1 and time.monotonic() < deadline and not self.cancel_event.is_set():
            time.sleep(0.05)
            exec_id = rpc.robot_call("MotionControl.getExecId")
        if self.cancel_event.is_set():
            raise RuntimeError(f"Aubo SDK moveJoint cancelled at {label}")
        if exec_id != -1:
            raise TimeoutError(f"Aubo SDK moveJoint exec timeout at {label}: exec_id={exec_id}")

    def _sdk_wait_arrival(self, rpc: AuboDirectJsonRpc, target: np.ndarray, label: str) -> bool:
        tolerance = max(float(self.get_parameter("aubo_sdk_goal_tolerance_rad").value), 0.001)
        speed = max(float(self.get_parameter("aubo_sdk_move_speed_rad_sec").value), 0.01)
        current = np.asarray(
            [float(value) for value in rpc.robot_call("RobotState.getJointPositions")],
            dtype=float,
        )
        max_delta = float(np.max(np.abs(self._joint_delta(target, current))))
        timeout = max(max_delta / speed, 0.5) + max(
            float(self.get_parameter("aubo_sdk_arrival_timeout_padding_sec").value), 0.0
        )
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        stable_required = 0.25
        last_error = max_delta
        while not self.cancel_event.is_set() and time.monotonic() < deadline:
            current = np.asarray(
                [float(value) for value in rpc.robot_call("RobotState.getJointPositions")],
                dtype=float,
            )
            last_error = float(np.max(np.abs(self._joint_delta(target, current))))
            if last_error <= tolerance:
                now = time.monotonic()
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= stable_required:
                    self._status(f"Aubo SDK moveJoint reached: {label}")
                    return True
            else:
                stable_since = None
            time.sleep(0.05)
        self._status(
            f"Aubo SDK moveJoint arrival timeout at {label}: "
            f"max_error={last_error:.3f}rad tolerance={tolerance:.3f}rad",
            warn=True,
        )
        return False

    def _joint_delta(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        return np.asarray(
            [
                _angle_diff(float(target_value), float(current_value))
                for target_value, current_value in zip(target, current)
            ],
            dtype=float,
        )

    def _send_arm_replay_segment(
        self, start: list[float], target: list[float], label: str
    ) -> bool:
        if len(target) != len(self.arm_command_joint_names):
            self._status(f"arm replay skipped: invalid joint target length for {label}", warn=True)
            return False
        if len(start) != len(target):
            self._status(f"arm replay skipped: invalid segment start length for {label}", warn=True)
            return False

        violation = self._arm_keepout_violation(target, label)
        if violation is not None:
            self._status(f"arm replay blocked by safety zone: {violation}", warn=True)
            return False

        duration = self._arm_replay_segment_duration(start, target)
        nominal_velocities, nominal_accelerations = self._arm_replay_segment_derivatives(
            start, target, duration
        )
        peak_speed = max((abs(value) for value in nominal_velocities), default=0.0)
        peak_accel = max((abs(value) for value in nominal_accelerations), default=0.0)
        self._status(
            f"arm replay segment: {label} t={duration:.2f}s "
            f"speed={peak_speed:.3f} accel={peak_accel:.3f}"
        )
        start_array = np.array([float(value) for value in start], dtype=float)
        target_array = np.array([float(value) for value in target], dtype=float)
        delta = np.array(
            [
                _angle_diff(float(target_value), float(start_value))
                for start_value, target_value in zip(start_array, target_array)
            ],
            dtype=float,
        )
        tolerance = max(float(self.get_parameter("arm_goal_tolerance").value), 0.005)
        rate = max(float(self.get_parameter("arm_velocity_publish_rate").value), 1.0)
        period = 1.0 / rate
        gain = 1.1
        deadline = time.monotonic() + max(duration + 8.0, 4.0)
        started = time.monotonic()
        stable_since: float | None = None
        stable_required = 0.25

        try:
            while not self.cancel_event.is_set() and time.monotonic() < deadline:
                now = time.monotonic()
                current = self._current_arm_vector()
                if current is None or len(current) != len(target):
                    self._status(f"arm replay skipped: no joint state for {label}", warn=True)
                    return False
                current_array = np.array([float(value) for value in current], dtype=float)
                elapsed = max(0.0, now - started)
                u = min(elapsed / max(duration, 1e-3), 1.0)
                smooth_u = (3.0 * u * u) - (2.0 * u * u * u)
                smooth_du = (6.0 * u * (1.0 - u)) / max(duration, 1e-3)
                desired = start_array + delta * smooth_u
                feedforward = delta * smooth_du
                position_error = np.array(
                    [
                        _angle_diff(float(desired_value), float(current_value))
                        for desired_value, current_value in zip(desired, current_array)
                    ],
                    dtype=float,
                )
                velocity = feedforward + position_error * gain
                final_error = np.array(
                    [
                        _angle_diff(float(target_value), float(current_value))
                        for target_value, current_value in zip(target_array, current_array)
                    ],
                    dtype=float,
                )
                if float(np.max(np.abs(final_error))) <= tolerance:
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= stable_required:
                        self._status(f"arm replay reached: {label}")
                        return True
                else:
                    stable_since = None
                self._set_manual_arm_velocity([float(value) for value in velocity], label)
                time.sleep(period)
            self._status(f"arm replay timeout/cancel: {label}", warn=True)
            return False
        finally:
            self.stop_arm_velocity_hold()

    def _arm_replay_segment_duration(self, start: list[float], target: list[float]) -> float:
        duration = max(float(self.get_parameter("arm_waypoint_duration_sec").value), 0.2)
        deltas = [abs(_angle_diff(float(t), float(s))) for s, t in zip(start, target)]
        max_delta = max(deltas, default=0.0)
        max_speed = max(float(self.get_parameter("arm_velocity_max_joint_speed_rad_sec").value), 0.01)
        max_accel = max(float(self.get_parameter("arm_velocity_max_joint_accel_rad_sec2").value), 0.01)
        duration = max(duration, max_delta / max_speed)
        duration = max(duration, math.sqrt(max_delta / max_accel) if max_delta > 0.0 else duration)
        return duration

    def _arm_replay_segment_derivatives(
        self, start: list[float], target: list[float], duration: float
    ) -> tuple[list[float], list[float]]:
        safe_duration = max(float(duration), 1e-3)
        velocities = [
            _angle_diff(float(target_value), float(start_value)) / safe_duration
            for start_value, target_value in zip(start, target)
        ]
        accelerations = [value / safe_duration for value in velocities]
        return velocities, accelerations

    def _wait_for_arm_target(
        self, target: list[float], tolerance: float, timeout_sec: float
    ) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        best_error = float("inf")
        stable_since: float | None = None
        stable_required = 0.25
        while time.monotonic() < deadline and not self.cancel_event.is_set():
            current = self._current_arm_vector()
            if current is not None and len(current) == len(target):
                error = max(abs(a - b) for a, b in zip(current, target))
                best_error = min(best_error, error)
                if error <= tolerance:
                    now = time.monotonic()
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= stable_required:
                        return True
                else:
                    stable_since = None
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

    def _managed_process_status_locked(self, name: str) -> str:
        process = self.managed_processes.get(name)
        if process is None:
            return "stopped"
        result = process.poll()
        if result is None:
            return f"running pid={process.pid}"
        return f"exited {result}"

    def start_managed_process(self, name: str) -> None:
        if name not in MANAGED_SERVICE_COMMAND_PARAMS:
            self._status(f"service start ignored unknown service: {name}", warn=True)
            return
        self._start_worker(lambda: self._start_managed_process_worker(name))

    def start_camera_stack(self) -> None:
        self.start_managed_process("camera")

        def delayed_viewer() -> None:
            time.sleep(0.8)
            self.start_managed_process("viewer")

        self._start_worker(delayed_viewer)

    def stop_managed_process(self, name: str) -> None:
        if name not in MANAGED_SERVICE_COMMAND_PARAMS:
            self._status(f"service stop ignored unknown service: {name}", warn=True)
            return
        self._start_worker(lambda: self._stop_managed_process_worker(name))

    def toggle_managed_process(self, name: str) -> None:
        with self.lock:
            process = self.managed_processes.get(name)
            running = process is not None and process.poll() is None
        if running:
            self.stop_managed_process(name)
        else:
            self.start_managed_process(name)

    def stop_all_managed_processes(self) -> None:
        for name in list(MANAGED_SERVICE_COMMAND_PARAMS):
            self._stop_managed_process_worker(name, quiet=True)

    def _start_managed_process_worker(self, name: str) -> None:
        command_param = MANAGED_SERVICE_COMMAND_PARAMS[name]
        command = str(self.get_parameter(command_param).value).strip()
        if not command:
            self._status(f"service {name} has empty command", warn=True)
            return
        already_message = ""
        with self.lock:
            existing = self.managed_processes.get(name)
            if existing is not None and existing.poll() is None:
                already_message = f"service {name} already running pid={existing.pid}"
        if already_message:
            self._status(already_message)
            return

        log_path = self.runtime_log_dir / f"{name}.log"
        shell_command = "\n".join(
            [
                "set -euo pipefail",
                f"cd {shlex.quote(str(self.workspace_root))}",
                "set +u",
                "source scripts/env/arachne_env.sh",
                "[[ -f install/setup.bash ]] && source install/setup.bash",
                "[[ -f scripts/env/arachne_real_defaults.sh ]] && source scripts/env/arachne_real_defaults.sh",
                "set -u",
                f"exec {command}",
            ]
        )
        try:
            handle = log_path.open("a", encoding="utf-8")
            handle.write(
                f"\n[{datetime.now().isoformat(timespec='seconds')}] start {name}: {command}\n"
            )
            handle.flush()
            process = subprocess.Popen(
                ["bash", "-lc", shell_command],
                cwd=str(self.workspace_root),
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except Exception as exc:
            try:
                handle.close()  # type: ignore[possibly-undefined]
            except Exception:
                pass
            self._status(f"service {name} start failed: {exc}", warn=True)
            return

        with self.lock:
            old_handle = self.managed_process_log_handles.pop(name, None)
            if old_handle is not None:
                try:
                    old_handle.close()
                except OSError:
                    pass
            self.managed_processes[name] = process
            self.managed_process_logs[name] = log_path
            self.managed_process_log_handles[name] = handle
            self.hardware_status[f"Svc:{name}"] = f"running pid={process.pid}"
        self._write_event(
            "service_start",
            f"{name} running pid={process.pid}",
            service=name,
            command=command,
            log=str(log_path),
            pid=process.pid,
        )
        self._status(f"service {name} started pid={process.pid}; log={log_path}")

    def _stop_managed_process_worker(self, name: str, *, quiet: bool = False) -> None:
        timeout_sec = max(float(self.get_parameter("service_stop_timeout_sec").value), 0.5)
        with self.lock:
            process = self.managed_processes.get(name)
        if process is None:
            if not quiet:
                self._status(f"service {name} already stopped")
            return

        result = process.poll()
        if result is None:
            if not quiet:
                self._status(f"service {name} stopping pid={process.pid}")
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except Exception as exc:
                    if not quiet:
                        self._status(f"service {name} terminate failed: {exc}", warn=True)
            except ProcessLookupError:
                pass
            except Exception as exc:
                if not quiet:
                    self._status(f"service {name} stop failed: {exc}", warn=True)
        result = process.poll()
        with self.lock:
            self.managed_processes.pop(name, None)
            self.hardware_status[f"Svc:{name}"] = f"stopped ({result})"
            handle = self.managed_process_log_handles.pop(name, None)
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass
        self._write_event("service_stop", f"{name} stopped result={result}", service=name)
        if not quiet:
            self._status(f"service {name} stopped result={result}")

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

    def _arm_keepout_violation(self, positions: list[float], label: str) -> str | None:
        if not bool(self.get_parameter("arm_keepout_enabled").value):
            return None
        if len(positions) != 6:
            return "invalid arm target length"
        try:
            boxes = self._arm_keepout_boxes()
            path = self._arm_keepout_path(np.array(positions, dtype=float))
            for path_index, q in enumerate(path, start=1):
                for sample_name, point in self._arm_keepout_samples(q):
                    for box in boxes:
                        if bool(np.all(point >= box.minimum) and np.all(point <= box.maximum)):
                            xyz = ",".join(f"{value:.3f}" for value in point)
                            return (
                                f"{label} enters {box.name}: {sample_name} "
                                f"at base_link xyz=({xyz}), path sample {path_index}/{len(path)}"
                            )
        except Exception as exc:
            return f"keepout config invalid: {exc}"
        return None

    def _arm_keepout_boxes(self) -> list[KeepoutBox]:
        minimum = _parse_vector3(str(self.get_parameter("rear_rack_keepout_min_xyz").value))
        maximum = _parse_vector3(str(self.get_parameter("rear_rack_keepout_max_xyz").value))
        if bool(np.any(maximum <= minimum)):
            raise ValueError("rear rack keepout max must be greater than min")
        return [KeepoutBox("rear rack keepout", minimum, maximum)]

    def _arm_keepout_path(self, target: np.ndarray) -> list[np.ndarray]:
        current = self._current_arm_vector()
        if current is None or len(current) != len(target):
            return [target]
        start = np.array(current, dtype=float)
        max_delta = float(np.max(np.abs(target - start)))
        joint_step = max(float(self.get_parameter("arm_keepout_joint_step_rad").value), 0.01)
        steps = max(1, min(60, int(math.ceil(max_delta / joint_step))))
        return [start + (target - start) * (index / steps) for index in range(1, steps + 1)]

    def _arm_keepout_samples(self, q: np.ndarray) -> list[tuple[str, np.ndarray]]:
        arm_base = _transform_from_xyz_rpy(
            _parse_vector3(str(self.get_parameter("arm_base_xyz").value)),
            _parse_vector3(str(self.get_parameter("arm_base_rpy").value)),
        )
        link_transforms = self.kinematics.link_transforms(q)
        link_points = [
            (name, (arm_base @ transform)[:3, 3])
            for name, transform in link_transforms
        ]
        samples: list[tuple[str, np.ndarray]] = []
        sample_step = max(float(self.get_parameter("arm_keepout_sample_step_m").value), 0.01)

        for name, point in link_points:
            samples.append((name, point))
        for (start_name, start), (end_name, end) in zip(link_points, link_points[1:]):
            distance = float(np.linalg.norm(end - start))
            steps = max(1, int(math.ceil(distance / sample_step)))
            for index in range(1, steps):
                alpha = index / steps
                samples.append((f"{start_name}->{end_name}", start + (end - start) * alpha))

        tool_transform = arm_base @ link_transforms[-1][1]
        tool_points = {
            "tool0": (0.0, 0.0, 0.0),
            "grasp_frame": (0.0, 0.0, 0.138691938),
            "tool_envelope_x+": (0.070, 0.0, 0.090),
            "tool_envelope_x-": (-0.070, 0.0, 0.090),
            "tool_envelope_y+": (0.0, 0.070, 0.090),
            "tool_envelope_y-": (0.0, -0.070, 0.090),
            "camera_envelope": (0.025, -0.090, 0.050),
        }
        tool_origin = tool_transform[:3, 3]
        for name, local in tool_points.items():
            point = tool_transform[:3, :3] @ np.array(local, dtype=float) + tool_origin
            samples.append((name, point))
        return samples

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
        gripper_settle_sec: float,
        arm_home_joints_deg: str,
        arm_install_joints_deg: str,
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
        if gripper_settle_sec < 0.0:
            raise ValueError("gripper settle must be non-negative")
        home_values = _parse_joint_degrees(arm_home_joints_deg, label="home joints")
        install_values = _parse_joint_degrees(arm_install_joints_deg, label="install joints")
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
                    "gripper_settle_sec",
                    Parameter.Type.DOUBLE,
                    float(gripper_settle_sec),
                ),
                Parameter(
                    "arm_home_joints_deg",
                    Parameter.Type.STRING,
                    _format_joint_degrees(home_values),
                ),
                Parameter(
                    "arm_install_joints_deg",
                    Parameter.Type.STRING,
                    _format_joint_degrees(install_values),
                ),
            ]
        )
        self._status(
            "motion settings applied: "
            f"base={base_linear_speed:.3f}m/s {base_angular_speed:.3f}rad/s, "
            f"tcp={arm_jog_step_m:.3f}m, rot={arm_rotate_step_deg:.1f}deg, "
            f"joint={arm_joint_step_deg:.1f}deg, hold={arm_hold_period_sec:.2f}s, "
            f"gripper_settle={gripper_settle_sec:.1f}s"
        )

    def _status(self, text: str, *, warn: bool = False) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.last_status = text
            self.log_lines.append(f"{stamp} {'WARN' if warn else 'INFO'} {text}")
            self.log_lines = self.log_lines[-300:]
        self._write_event("status", text, level="warn" if warn else "info")
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
        self.root.geometry("1100x720")
        self.root.minsize(820, 520)
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
        self.gripper_settle_var = tk.StringVar(
            value=f"{float(node.get_parameter('gripper_settle_sec').value):.1f}"
        )
        self.home_joints_var = tk.StringVar(
            value=str(node.get_parameter("arm_home_joints_deg").value)
        )
        self.install_joints_var = tk.StringVar(
            value=str(node.get_parameter("arm_install_joints_deg").value)
        )
        self.config_path_var = tk.StringVar(
            value=str(node.get_parameter("teach_config_path").value)
        )
        self.joint_target_vars = [tk.StringVar(value="") for _ in range(6)]
        self.tool_target_vars = {axis: tk.StringVar(value="") for axis in ("x", "y", "z")}
        self.move_status_vars: dict[str, tk.StringVar] = {}
        self.move_joint_vars: list[tk.StringVar] = []
        self._arm_hold_after: str | None = None
        self._arm_hold_callback = None
        self._arm_hold_button: ttk.Button | None = None
        self._arm_hold_started_at = 0.0
        self._arm_hold_stream_active = False
        self._preset_hold_after: str | None = None
        self._preset_hold_active = False
        self.program_record_buttons: list[ttk.Button] = []
        self.listbox: tk.Listbox | None = None
        self.log_text: tk.Text | None = None
        self.joint_tree: ttk.Treeview | None = None
        self._build()
        self.root.bind_all("<ButtonRelease-1>", lambda _event: self._arm_hold_release(), add=True)
        self.root.bind("<FocusOut>", lambda _event: self._arm_hold_release(), add=True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
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
        style.configure("Primary.TButton", foreground="#0f4c81")
        style.configure("Service.TButton", foreground="#245b2a")

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

    def _close(self) -> None:
        self._arm_hold_release()
        self._preset_hold_release()
        self.node.hold_arm_current()
        self.root.destroy()

    def _add_scrollable_tab(self, title: str) -> ttk.Frame:
        outer = ttk.Frame(self.notebook)
        self.notebook.add(outer, text=title)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        canvas = tk.Canvas(outer, highlightthickness=0, borderwidth=0)
        scroll = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")

        content = ttk.Frame(canvas, padding=10)
        window_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def refresh_scrollregion(_event=None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def resize_content(event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        def mousewheel(event) -> None:
            if event.num == 4:
                canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                canvas.yview_scroll(1, "units")
            else:
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        content.bind("<Configure>", refresh_scrollregion)
        canvas.bind("<Configure>", resize_content)
        for widget in (canvas, content):
            widget.bind("<MouseWheel>", mousewheel, add=True)
            widget.bind("<Button-4>", mousewheel, add=True)
            widget.bind("<Button-5>", mousewheel, add=True)
        return content

    def _build_button_group(
        self,
        parent: ttk.Frame,
        title: str,
        buttons: tuple[tuple[str, Any, str | None], ...],
        *,
        columns: int = 2,
    ) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text=title)
        for column in range(columns):
            frame.columnconfigure(column, weight=1)
        for index, (text, command, style_name) in enumerate(buttons):
            button = ttk.Button(frame, text=text, command=command)
            if style_name:
                button.configure(style=style_name)
            if text == "Program Rec":
                self.program_record_buttons.append(button)
            button.grid(
                row=index // columns,
                column=index % columns,
                sticky="ew",
                padx=5,
                pady=4,
            )
        return frame

    def _build_top_bar(self) -> None:
        top = ttk.Frame(self.root, style="Top.TFrame", padding=(12, 8))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(5, weight=1)
        ttk.Label(top, text="Arachne Scope", style="TopTitle.TLabel").grid(
            row=0, column=0, rowspan=2, sticky="w", padx=(0, 18)
        )
        for column, key in enumerate(("Aubo", "Base", "Gripper", "Grasp"), start=1):
            var = tk.StringVar(value="waiting")
            self.status_vars[key] = var
            ttk.Label(top, text=key, style="Top.TLabel").grid(row=0, column=column, sticky="w", padx=8)
            ttk.Label(top, textvariable=var, style="Top.TLabel", width=16).grid(
                row=1, column=column, sticky="w", padx=8
            )
        status_var = tk.StringVar(value="ready")
        self.status_vars["status"] = status_var
        ttk.Label(top, textvariable=status_var, style="Top.TLabel").grid(
            row=0, column=5, rowspan=2, sticky="ew", padx=(16, 8)
        )
        self._make_preset_hold_button(top, "Home", "home").grid(row=0, column=6, rowspan=2, padx=4)
        ttk.Button(top, text="Stop", command=self.node.stop_all, style="Danger.TButton").grid(
            row=0, column=7, rowspan=2, padx=4
        )
        ttk.Button(
            top,
            text="Visual Grasp",
            command=self.node.visual_grasp_start,
            style="Primary.TButton",
        ).grid(
            row=0, column=8, rowspan=2, padx=4
        )
        ttk.Button(top, text="Road", command=lambda: self.node.call_cleanup_task("start")).grid(
            row=0, column=9, rowspan=2, padx=4
        )
        ttk.Button(
            top,
            text="Task Stop",
            command=lambda: self.node.call_grasp_task("stop"),
            style="Danger.TButton",
        ).grid(row=0, column=10, rowspan=2, padx=4)
        ttk.Button(
            top,
            text="Road Stop",
            command=lambda: self.node.call_cleanup_task("stop"),
            style="Danger.TButton",
        ).grid(row=0, column=11, rowspan=2, padx=4)
        ttk.Button(
            top,
            text="Road Pause",
            command=lambda: self.node.call_cleanup_task("pause"),
            style="Danger.TButton",
        ).grid(row=0, column=12, rowspan=2, padx=4)
        ttk.Button(
            top,
            text="Return",
            command=lambda: self.node.call_cleanup_task("return_home"),
        ).grid(row=0, column=13, rowspan=2, padx=4)

    def _confirm_aubo_power_off(self) -> None:
        if not messagebox.askyesno(
            "Aubo Power Off",
            "Power off the real Aubo arm now?\n\nConfirm the arm is supported and the workspace is safe.",
        ):
            return
        self.node.command_aubo_lifecycle("power_off")

    def _build_home_tab(self) -> None:
        tab = self._add_scrollable_tab("Home")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)

        overview = ttk.LabelFrame(tab, text="Robot Status")
        overview.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        overview.columnconfigure(1, weight=1)
        for row, key in enumerate(
            (
                "base",
                "tool",
                "arm",
                "gripper",
                "teach",
                "grasp_task",
                "road_cleanup",
                "camera",
                "viewer",
                "slam",
                "grasp_server",
                "cleanup_server",
                "program",
                "log_dir",
            )
        ):
            ttk.Label(overview, text=key, style="State.TLabel", width=10).grid(
                row=row, column=0, sticky="w", padx=6, pady=4
            )
            var = tk.StringVar(value="waiting")
            self.status_vars[key] = var
            ttk.Label(overview, textvariable=var).grid(row=row, column=1, sticky="ew", padx=6, pady=4)

        quick = ttk.Frame(tab)
        quick.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        for column in range(2):
            quick.columnconfigure(column, weight=1)

        task_group = self._build_button_group(
            quick,
            "Task Flow",
            (
                ("Visual Grasp", self.node.visual_grasp_start, "Primary.TButton"),
                ("Road Start", lambda: self.node.call_cleanup_task("start"), "Primary.TButton"),
                ("Road Pause", lambda: self.node.call_cleanup_task("pause"), "Danger.TButton"),
                ("Road Return", lambda: self.node.call_cleanup_task("return_home"), None),
                ("Grasp Start", lambda: self.node.call_grasp_task("start"), None),
                ("Road Preflight", lambda: self.node.call_cleanup_task("preflight"), None),
                ("Restore", lambda: self.node.call_grasp_task("restore"), None),
                ("Road Stop", lambda: self.node.call_cleanup_task("stop"), "Danger.TButton"),
                ("Grasp Stop", lambda: self.node.call_grasp_task("stop"), "Danger.TButton"),
                ("Stop All", self.node.stop_all, "Danger.TButton"),
            ),
            columns=2,
        )
        task_group.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        service_group = self._build_button_group(
            quick,
            "Runtime Services",
            (
                ("Camera + View", self.node.start_camera_stack, "Service.TButton"),
                ("2D View", lambda: self.node.toggle_managed_process("viewer"), None),
                ("Localize / Nav", lambda: self.node.toggle_managed_process("slam"), None),
                ("Grasp Server", lambda: self.node.toggle_managed_process("grasp_server"), None),
                ("Road Server", lambda: self.node.toggle_managed_process("cleanup_server"), None),
            ),
            columns=2,
        )
        service_group.grid(row=1, column=0, sticky="new", padx=(0, 5), pady=(0, 8))

        arm_group = self._build_button_group(
            quick,
            "Aubo Power / Teach",
            (
                ("Aubo On", lambda: self.node.command_aubo_lifecycle("power_on"), None),
                ("Aubo Start", lambda: self.node.command_aubo_lifecycle("startup"), None),
                ("Teach On", lambda: self.node.set_aubo_teach(True), None),
                ("Teach Off", lambda: self.node.set_aubo_teach(False), None),
                ("Aubo Off", self._confirm_aubo_power_off, "Danger.TButton"),
            ),
            columns=2,
        )
        arm_group.grid(row=1, column=1, sticky="new", padx=(5, 0), pady=(0, 8))

        preset_group = ttk.LabelFrame(quick, text="Pose Presets")
        preset_group.grid(row=2, column=0, sticky="new", padx=(0, 5), pady=(0, 8))
        for column in range(2):
            preset_group.columnconfigure(column, weight=1)
        self._make_preset_hold_button(preset_group, "Hold Home", "home").grid(
            row=0, column=0, sticky="ew", padx=5, pady=4
        )
        self._make_preset_hold_button(preset_group, "Hold Install", "install").grid(
            row=0, column=1, sticky="ew", padx=5, pady=4
        )
        ttk.Button(preset_group, text="Set Home", command=self._set_home_from_current).grid(
            row=1, column=0, sticky="ew", padx=5, pady=4
        )
        ttk.Button(preset_group, text="Set Install", command=self._set_install_from_current).grid(
            row=1, column=1, sticky="ew", padx=5, pady=4
        )

        program_group = self._build_button_group(
            quick,
            "Program",
            (
                ("Program Rec", self._toggle_program_recording, None),
                ("Record", self._record, None),
                ("Replay", self._play, None),
                ("Save Config", self._save_config, None),
            ),
            columns=2,
        )
        program_group.grid(row=2, column=1, sticky="new", padx=(5, 0), pady=(0, 8))

        gripper_group = self._build_button_group(
            quick,
            "Gripper",
            (
                ("Open", lambda: self.node.publish_gripper("open"), None),
                ("Close", lambda: self.node.publish_gripper("close"), None),
            ),
            columns=2,
        )
        gripper_group.grid(row=3, column=0, columnspan=2, sticky="ew")

        services = ttk.LabelFrame(tab, text="Runtime Services")
        services.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        services.columnconfigure(1, weight=1)
        for row, (key, label) in enumerate(
            (
                ("camera", "Gemini Camera"),
                ("viewer", "2D Raw View"),
                ("slam", "Localize / Nav"),
                ("grasp_server", "Grasp Server"),
                ("cleanup_server", "Road Cleanup Server"),
            )
        ):
            ttk.Label(services, text=label, style="State.TLabel", width=16).grid(
                row=row, column=0, sticky="w", padx=6, pady=4
            )
            var = self.status_vars.get(key)
            if var is None:
                var = tk.StringVar(value="stopped")
                self.status_vars[key] = var
            ttk.Label(services, textvariable=var).grid(
                row=row, column=1, sticky="ew", padx=6, pady=4
            )
            ttk.Button(
                services,
                text="Start",
                command=lambda name=key: self.node.start_managed_process(name),
            ).grid(row=row, column=2, sticky="ew", padx=4, pady=4)
            ttk.Button(
                services,
                text="Stop",
                command=lambda name=key: self.node.stop_managed_process(name),
                style="Danger.TButton",
            ).grid(row=row, column=3, sticky="ew", padx=4, pady=4)

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
        tab = self._add_scrollable_tab("Move")
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
        tab = self._add_scrollable_tab("Program")
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
        tab = self._add_scrollable_tab("Configure")
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
            ("Gripper settle s", self.gripper_settle_var),
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
        install_row = home_row + 1
        ttk.Label(motion, text="Install joints deg", width=20).grid(
            row=install_row, column=0, sticky="w", padx=6, pady=5
        )
        ttk.Entry(motion, textvariable=self.install_joints_var, width=52).grid(
            row=install_row, column=1, sticky="ew", padx=6, pady=5
        )
        ttk.Button(motion, text="Apply", command=self._apply_motion_settings).grid(
            row=install_row + 1, column=0, columnspan=2, sticky="ew", padx=6, pady=(8, 6)
        )
        motion.columnconfigure(1, weight=1)

        presets = ttk.LabelFrame(tab, text="Preset Poses")
        presets.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(4):
            presets.columnconfigure(column, weight=1)
        self._make_preset_hold_button(presets, "Hold Home", "home").grid(
            row=0, column=0, sticky="ew", padx=6, pady=5
        )
        self._make_preset_hold_button(presets, "Hold Install", "install").grid(
            row=0, column=1, sticky="ew", padx=6, pady=5
        )
        ttk.Button(presets, text="Set Home From Current", command=self._set_home_from_current).grid(
            row=0, column=2, sticky="ew", padx=6, pady=5
        )
        ttk.Button(
            presets, text="Set Install From Current", command=self._set_install_from_current
        ).grid(row=0, column=3, sticky="ew", padx=6, pady=5)

        config = ttk.LabelFrame(tab, text="Local Config")
        config.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        config.columnconfigure(1, weight=1)
        ttk.Label(config, text="Config path", width=20).grid(
            row=0, column=0, sticky="w", padx=6, pady=5
        )
        ttk.Entry(config, textvariable=self.config_path_var).grid(
            row=0, column=1, columnspan=3, sticky="ew", padx=6, pady=5
        )
        ttk.Button(config, text="Browse", command=self._browse_config).grid(
            row=1, column=0, sticky="ew", padx=6, pady=(0, 6)
        )
        ttk.Button(config, text="Save Config", command=self._save_config).grid(
            row=1, column=1, sticky="ew", padx=6, pady=(0, 6)
        )
        ttk.Button(config, text="Load Config", command=self._load_config).grid(
            row=1, column=2, sticky="ew", padx=6, pady=(0, 6)
        )
        ttk.Button(config, text="Default Path", command=self._use_default_config_path).grid(
            row=1, column=3, sticky="ew", padx=6, pady=(0, 6)
        )

        payload = ttk.LabelFrame(tab, text="Aubo Payload")
        payload.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(payload, text="Startup payload is configured by scripts/hardware/real_full_teach.sh.").grid(
            row=0, column=0, sticky="w", padx=6, pady=4
        )
        payload_mass = float(self.node.get_parameter("aubo_payload_mass_kg").value)
        payload_cog_mm = [
            value * 1000.0
            for value in _parse_vector(
                str(self.node.get_parameter("aubo_payload_cog").value),
                expected=3,
                label="payload cog",
            )
        ]
        ttk.Label(
            payload,
            text=(
                f"Reusable default payload: {payload_mass:.3f}kg, "
                f"CoG {payload_cog_mm[0]:.3f}/{payload_cog_mm[1]:.3f}/{payload_cog_mm[2]:.3f} mm."
            ),
        ).grid(
            row=1, column=0, sticky="w", padx=6, pady=4
        )

    def _build_log_tab(self) -> None:
        tab = self._add_scrollable_tab("Log")
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
        base_rec_button = ttk.Button(
            frame, text="Program Rec Off", command=self._toggle_program_recording
        )
        base_rec_button.grid(row=3, column=0, columnspan=3, padx=4, pady=(8, 4), sticky="ew")
        self.program_record_buttons.append(base_rec_button)

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
        ttk.Button(frame, text="Power On", command=lambda: self.node.command_aubo_lifecycle("power_on")).grid(
            row=7, column=0, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(frame, text="Startup", command=lambda: self.node.command_aubo_lifecycle("startup")).grid(
            row=7, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(frame, text="Power Off", command=self._confirm_aubo_power_off, style="Danger.TButton").grid(
            row=7, column=2, padx=4, pady=4, sticky="ew"
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
        ttk.Button(frame, text="Use Install", command=self._fill_install_joints).grid(
            row=7, column=3, sticky="ew", padx=4, pady=(6, 4)
        )
        ttk.Button(frame, text="Move Joints", command=self._move_to_joint_targets).grid(
            row=7, column=4, sticky="ew", padx=4, pady=(6, 4)
        )
        self._make_preset_hold_button(frame, "Hold Home Pose", "home").grid(
            row=8, column=0, columnspan=3, sticky="ew", padx=4, pady=(0, 4)
        )
        self._make_preset_hold_button(frame, "Hold Install Pose", "install").grid(
            row=8, column=3, columnspan=2, sticky="ew", padx=4, pady=(0, 4)
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
        button.bind("<ButtonPress-1>", lambda _event, b=button: self._arm_hold_start(callback, b))
        button.bind("<ButtonRelease-1>", lambda _event: self._arm_hold_release())
        return button

    def _make_preset_hold_button(self, parent: ttk.Frame, text: str, preset: str):
        button = ttk.Button(parent, text=text)
        button.bind("<ButtonPress-1>", lambda _event: self._preset_hold_start(preset))
        button.bind("<ButtonRelease-1>", lambda _event: self._preset_hold_release())
        button.bind("<Leave>", lambda _event: self._preset_hold_release())
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
        rec_enabled, pending_base = self.node.base_motion_recording_state()
        rec_label = "rec on" if rec_enabled else "rec off"
        snapshot["program"] = f"{len(self.waypoints)} nodes | {rec_label} | base pending={pending_base}"
        for key, var in self.status_vars.items():
            var.set(snapshot.get(key, "waiting"))
        for key, var in self.move_status_vars.items():
            var.set(snapshot.get(key, "waiting"))
        self._update_program_record_button()
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

    def _apply_motion_settings(self) -> bool:
        try:
            self.node.update_motion_settings(
                base_linear_speed=float(self.base_linear_var.get()),
                base_angular_speed=float(self.base_angular_var.get()),
                arm_jog_step_m=float(self.arm_step_var.get()),
                arm_rotate_step_deg=float(self.arm_rotate_var.get()),
                arm_joint_step_deg=float(self.arm_joint_step_var.get()),
                arm_hold_period_sec=float(self.arm_hold_period_var.get()),
                waypoint_duration_sec=float(self.waypoint_duration_var.get()),
                gripper_settle_sec=float(self.gripper_settle_var.get()),
                arm_home_joints_deg=self.home_joints_var.get(),
                arm_install_joints_deg=self.install_joints_var.get(),
            )
            self._sync_config_vars_from_node()
            return True
        except Exception as exc:
            messagebox.showerror("Apply failed", str(exc))
            return False

    def _arm_hold_start(self, callback, button: ttk.Button) -> None:
        self._arm_hold_release(cancel_arm=False)
        self._arm_hold_callback = callback
        self._arm_hold_button = button
        self._arm_hold_started_at = time.monotonic()
        self._arm_hold_callback()
        self._arm_hold_stream_active = True
        self._arm_hold_after = self.root.after(50, self._arm_hold_heartbeat)

    def _arm_hold_heartbeat(self) -> None:
        if self._arm_hold_callback is None:
            self._arm_hold_after = None
            return
        self.node.refresh_arm_velocity_stream_deadman()
        self._arm_hold_after = self.root.after(50, self._arm_hold_heartbeat)

    def _arm_hold_release(self, *, cancel_arm: bool = True) -> None:
        was_active = self._arm_hold_after is not None or self._arm_hold_callback is not None
        if self._arm_hold_after is not None:
            self.root.after_cancel(self._arm_hold_after)
            self._arm_hold_after = None
        self._arm_hold_callback = None
        self._arm_hold_button = None
        self._arm_hold_started_at = 0.0
        self._arm_hold_stream_active = False
        if cancel_arm and was_active:
            self.node.hold_arm_current()

    def _preset_hold_start(self, preset: str) -> None:
        self._preset_hold_release(cancel_arm=False)
        self._preset_hold_active = True

        def start_motion() -> None:
            if not self._preset_hold_active:
                return
            self._preset_hold_after = None
            self.node.move_arm_preset(preset)

        self._preset_hold_after = self.root.after(250, start_motion)

    def _preset_hold_release(self, *, cancel_arm: bool = True) -> None:
        was_active = self._preset_hold_active
        self._preset_hold_active = False
        if self._preset_hold_after is not None:
            self.root.after_cancel(self._preset_hold_after)
            self._preset_hold_after = None
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

    def _fill_install_joints(self) -> None:
        try:
            joints = self.node.install_joints_degrees()
        except Exception as exc:
            messagebox.showerror("Install target", str(exc))
            return
        for var, value in zip(self.joint_target_vars, joints):
            var.set(f"{value:.2f}")

    def _set_home_from_current(self) -> None:
        current = self.node.current_joints_degrees()
        if current is None:
            messagebox.showerror("Set Home", "No Aubo joint state is available.")
            return
        self.home_joints_var.set(_format_joint_degrees(current))
        self._apply_motion_settings()

    def _set_install_from_current(self) -> None:
        current = self.node.current_joints_degrees()
        if current is None:
            messagebox.showerror("Set Install", "No Aubo joint state is available.")
            return
        self.install_joints_var.set(_format_joint_degrees(current))
        self._apply_motion_settings()

    def _sync_config_vars_from_node(self) -> None:
        self.base_linear_var.set(f"{float(self.node.get_parameter('base_linear_speed').value):.3f}")
        self.base_angular_var.set(f"{float(self.node.get_parameter('base_angular_speed').value):.3f}")
        self.arm_step_var.set(f"{float(self.node.get_parameter('arm_jog_step_m').value):.3f}")
        self.arm_rotate_var.set(
            f"{math.degrees(float(self.node.get_parameter('arm_rotate_step_rad').value)):.1f}"
        )
        self.arm_joint_step_var.set(
            f"{math.degrees(float(self.node.get_parameter('arm_joint_step_rad').value)):.1f}"
        )
        self.arm_hold_period_var.set(
            f"{float(self.node.get_parameter('arm_hold_period_sec').value):.2f}"
        )
        self.waypoint_duration_var.set(
            f"{float(self.node.get_parameter('arm_waypoint_duration_sec').value):.2f}"
        )
        self.gripper_settle_var.set(
            f"{float(self.node.get_parameter('gripper_settle_sec').value):.1f}"
        )
        self.home_joints_var.set(str(self.node.get_parameter("arm_home_joints_deg").value))
        self.install_joints_var.set(str(self.node.get_parameter("arm_install_joints_deg").value))
        self.config_path_var.set(str(self.node.get_parameter("teach_config_path").value))

    def _use_default_config_path(self) -> None:
        self.config_path_var.set(DEFAULT_TEACH_CONFIG_PATH)

    def _browse_config(self) -> None:
        default = self.node.teach_config_path(self.config_path_var.get())
        path = filedialog.askopenfilename(
            initialdir=str(default.parent),
            initialfile=default.name,
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
        )
        if path:
            self.config_path_var.set(path)

    def _save_config(self) -> None:
        try:
            if not self._apply_motion_settings():
                return
            path = self.node.save_teach_config(self.config_path_var.get())
            self.config_path_var.set(str(path))
        except Exception as exc:
            messagebox.showerror("Save Config", str(exc))

    def _load_config(self) -> None:
        try:
            loaded = self.node.load_teach_config(self.config_path_var.get())
            if loaded:
                self._sync_config_vars_from_node()
        except Exception as exc:
            messagebox.showerror("Load Config", str(exc))

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

    def _toggle_program_recording(self) -> None:
        enabled, _pending = self.node.base_motion_recording_state()
        self.node.set_base_motion_recording(not enabled)
        self._update_program_record_button()

    def _update_program_record_button(self) -> None:
        if not hasattr(self, "program_record_buttons"):
            return
        enabled, pending = self.node.base_motion_recording_state()
        text = f"Program Rec {'On' if enabled else 'Off'}"
        if pending:
            text += f" ({pending})"
        for button in self.program_record_buttons:
            button.configure(text=text)

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
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        TeachPanelApp(node).run()
    finally:
        node.stop_all()
        node.stop_all_managed_processes()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
