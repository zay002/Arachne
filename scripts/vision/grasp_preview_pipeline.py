#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs.msg import JointState
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Empty, Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "arachne_operator"))
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics

try:
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "arachne_hardware"))
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc


END_EFFECTOR_FRAME = "grasp_frame"
PATH_PLAYBACK_PERIOD = 8.0
ARM_JOINT_NAMES = tuple(joint.name for joint in AuboI5Kinematics.JOINTS)
REAL_ARM_JOINT_NAMES = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
BASE_JOINT_DEFAULTS = (
    ("front_left_wheel", 0.0),
    ("rear_left_wheel", 0.0),
    ("front_right_wheel", 0.0),
    ("rear_right_wheel", 0.0),
)
GRIPPER_JOINT_DEFAULTS = {
    "ms42dc": (
        ("ms42dc_left_finger_joint", 0.0),
        ("ms42dc_right_finger_joint", 0.0),
    ),
    "ag95": (
        ("left_outer_knuckle_joint", 0.0),
        ("right_outer_knuckle_joint", 0.0),
        ("left_finger_joint", 0.0),
        ("right_finger_joint", 0.0),
        ("left_inner_knuckle_joint", 0.0),
        ("right_inner_knuckle_joint", 0.0),
    ),
}
GRIPPER_PREVIEW_PROFILES = {
    "ms42dc": {
        "names": ("ms42dc_left_finger_joint", "ms42dc_right_finger_joint"),
        "open": (0.0, 0.0),
        "closed": (0.6, -0.6),
    },
    "ag95": {
        "names": (
            "left_outer_knuckle_joint",
            "right_outer_knuckle_joint",
            "left_finger_joint",
            "right_finger_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
        ),
        "open": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "closed": (0.93, 0.93, 0.93, 0.93, 0.93, 0.93),
    },
}
DEFAULT_ARM_JOINTS = (
    -1.5707963267949,
    0.201570428261868,
    1.65970467002488,
    0.485178041391533,
    1.67675136677345,
    0.76432946885334,
)
TEACH_ARM_STREAM_RATE_HZ = 80.0
LOCKED_PLAYBACK_RATE = TEACH_ARM_STREAM_RATE_HZ
PREVIEW_IK_TOLERANCE_M = 0.006
PREVIEW_IK_ORIENTATION_TOLERANCE_RAD = 0.01
PREVIEW_IK_DAMPING = 0.08
PREVIEW_IK_MAX_ITERATIONS = 180
PREVIEW_IK_MAX_STEP = 0.05
PREVIEW_IK_ORIENTATION_WEIGHT = 0.5
PREVIEW_MAX_JOINT_SPEED_RAD_SEC = 0.85
PREVIEW_MAX_JOINT_ACCEL_RAD_SEC2 = 4.50
PREVIEW_MAX_JOINT_JERK_RAD_SEC3 = 100.0
PREVIEW_SMOOTHING_TAU_SEC = 0.04
COLLISION_PENALTY = 1_000_000.0
DEFAULT_AUBO_TEACH_FLAG_PATH = "/tmp/arachne_aubo_teach_mode"
DEFAULT_AUBO_CONTROL_OWNER_PATH = "/tmp/arachne_aubo_control_owner"


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
    owner = owner.strip() or "grasp_task_server"
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
    except FileNotFoundError:
        return
    except OSError:
        return
    active_owner, pid = _parse_control_owner(text)
    if active_owner != (owner.strip() or "grasp_task_server"):
        return
    if pid is not None and pid != os.getpid():
        return
    try:
        path.unlink(missing_ok=True)
    except OSError:
        return


@dataclass
class Detection:
    label: str
    class_id: int
    confidence: float
    xyxy: tuple[float, float, float, float]
    mask_xy: tuple[tuple[float, float], ...] | None = None
    mask_area_px: float = 0.0


@dataclass(frozen=True)
class CartesianSegment:
    kind: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    control: tuple[float, float, float] | None = None
    duration: float = 0.0


@dataclass(frozen=True)
class JointTrajectoryFrame:
    time_from_start: float
    positions: tuple[float, float, float, float, float, float]
    velocities: tuple[float, float, float, float, float, float]
    accelerations: tuple[float, float, float, float, float, float]


@dataclass
class MoveItPlanAttempt:
    response: object | None
    message: str


@dataclass(frozen=True)
class CollisionBox:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class PointCloudGraspShape:
    center_base: tuple[float, float, float]
    major_axis_base: tuple[float, float, float]
    minor_axis_base: tuple[float, float, float]
    visual_axis_base: tuple[float, float, float] | None
    extent_major_m: float
    extent_minor_m: float
    extent_z_m: float
    axis_confidence: float
    visual_axis_confidence: float
    point_count: int
    z_bias_m: float


@dataclass
class GraspPreview:
    detection: Detection
    depth_m: float
    grasp_xyz: tuple[float, float, float]
    pregrasp_xyz: tuple[float, float, float]
    lift_xyz: tuple[float, float, float]
    retreat_xyz: tuple[float, float, float]
    roi_points: list[tuple[float, float, float]]
    roi_source: str
    bbox_points: list[tuple[float, float, float]]
    base_path_xyz: list[tuple[float, float, float]]
    base_trajectory_segments: list[CartesianSegment]
    base_waypoints: list[tuple[str, tuple[float, float, float]]]
    arm_trajectory_frames: list[JointTrajectoryFrame]
    basket_safe: bool
    base_grasp_xyz: tuple[float, float, float] | None
    pointcloud_shape: PointCloudGraspShape | None
    snapshot_reason: str
    ik_message: str


def _add_venv_site_packages(venv: Path) -> None:
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        venv / "lib" / pyver / "site-packages",
        venv / "lib64" / pyver / "site-packages",
    ]
    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))


def _load_yolo(venv: Path, model_path: Path, task: str, initial_device: str = ""):
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    _add_venv_site_packages(venv)
    from ultralytics import YOLO

    kwargs = {"task": task} if task else {}
    model = YOLO(str(model_path), **kwargs)
    if initial_device:
        model.overrides["device"] = initial_device
    return model


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview detect-depth-grasp path from Gemini335 RGB-D in RViz."
    )
    hidden = argparse.SUPPRESS
    parser.add_argument(
        "--model", default="yolo_workspace/weights/trash_yolo26n_seg_best.pt", help=hidden
    )
    parser.add_argument("--venv", default="yolo_workspace/.venv", help=hidden)
    parser.add_argument("--yolo-task", default="segment", help=hidden)
    parser.add_argument("--classes", default="trash")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640, help=hidden)
    parser.add_argument("--device-id", default="0", help=hidden)
    parser.add_argument(
        "--onnx-device",
        default=os.environ.get("ARACHNE_GRASP_ONNX_DEVICE", "cpu"),
        help=hidden,
    )
    parser.add_argument("--gripper-type", choices=("ms42dc", "ag95"), default="ms42dc", help=hidden)
    parser.add_argument("--inference-period", type=float, default=0.45, help=hidden)
    parser.add_argument("--snapshot-iou-threshold", type=float, default=0.35, help=hidden)
    parser.add_argument("--snapshot-center-shift", type=float, default=0.12, help=hidden)
    parser.add_argument("--lost-frame-threshold", type=int, default=5, help=hidden)
    parser.add_argument("--locked-visual-rate", type=float, default=10.0, help=hidden)
    parser.add_argument("--restart-search-topic", default="/arachne/grasp_preview/restart_search")
    parser.add_argument("--joint-states-topic", default="/arachne/display/joint_states", help=hidden)
    parser.add_argument("--color-topic", default="/camera/color/image_raw", help=hidden)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw", help=hidden)
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info", help=hidden)
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info", help=hidden)
    parser.add_argument("--depth-scale", type=float, default=0.001, help=hidden)
    parser.add_argument("--min-depth", type=float, default=0.12, help=hidden)
    parser.add_argument("--max-depth", type=float, default=2.5, help=hidden)
    parser.add_argument("--depth-percentile", type=float, default=35.0, help=hidden)
    parser.add_argument("--depth-band", type=float, default=0.08, help=hidden)
    parser.add_argument("--roi-shrink", type=float, default=0.65, help=hidden)
    parser.add_argument("--roi-decimation", type=int, default=3, help=hidden)
    parser.add_argument(
        "--depth-projection-flip-x",
        dest="depth_projection_flip_x",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-depth-projection-flip-x",
        dest="depth_projection_flip_x",
        action="store_false",
        help=hidden,
    )
    parser.add_argument(
        "--depth-projection-flip-y",
        dest="depth_projection_flip_y",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-depth-projection-flip-y",
        dest="depth_projection_flip_y",
        action="store_false",
        help=hidden,
    )
    parser.add_argument("--approach-distance", type=float, default=0.18, help=hidden)
    parser.add_argument("--grasp-standoff", type=float, default=0.035, help=hidden)
    parser.add_argument("--grasp-tcp-offset-m", type=float, default=0.0, help=hidden)
    parser.add_argument("--grasp-base-offset", default="0,0,0", help=hidden)
    parser.add_argument("--disable-pointcloud-grasp-shape", action="store_true", help=hidden)
    parser.add_argument("--pointcloud-grasp-min-points", type=int, default=24, help=hidden)
    parser.add_argument("--pointcloud-axis-confidence-threshold", type=float, default=0.10, help=hidden)
    parser.add_argument("--visual-axis-confidence-threshold", type=float, default=0.12, help=hidden)
    parser.add_argument("--pointcloud-visible-upper-half-z-bias-ratio", type=float, default=0.25, help=hidden)
    parser.add_argument("--pointcloud-visible-upper-half-z-bias-max", type=float, default=0.025, help=hidden)
    parser.add_argument("--lift-distance", type=float, default=0.10, help=hidden)
    parser.add_argument("--base-frame", default="base_link", help=hidden)
    parser.add_argument("--aubo-base-frame", default="grasp_preview_aubo_base_link", help=hidden)
    parser.add_argument("--basket-release-base", default="0.545,0.0,0.18", help=hidden)
    parser.add_argument("--basket-approach-base", default="0.545,0.0,0.34", help=hidden)
    parser.add_argument("--basket-keepout-min-base", default="0.4215,-0.11,-0.1235", help=hidden)
    parser.add_argument("--basket-keepout-max-base", default="0.6655,0.11,0.0635", help=hidden)
    parser.add_argument("--basket-clearance", type=float, default=0.04, help=hidden)
    parser.add_argument("--gripper-radius", type=float, default=0.055, help=hidden)
    parser.add_argument("--arm-collision-radius", type=float, default=0.075, help=hidden)
    parser.add_argument("--arm-collision-samples-per-link", type=int, default=8, help=hidden)
    parser.add_argument("--collision-margin", type=float, default=0.035, help=hidden)
    parser.add_argument("--ground-min-z-base", type=float, default=-0.22, help=hidden)
    parser.add_argument("--ground-clearance", type=float, default=0.02, help=hidden)
    parser.add_argument("--tool-ground-clearance", type=float, default=0.015, help=hidden)
    parser.add_argument("--allow-colliding-best-effort", action="store_true", help=hidden)
    parser.add_argument("--release-tool-tilt-deg", type=float, default=12.0, help=hidden)
    parser.add_argument("--transit-height", type=float, default=0.36, help=hidden)
    parser.add_argument("--transit-arc-height", type=float, default=0.16, help=hidden)
    parser.add_argument("--arc-samples", type=int, default=24, help=hidden)
    parser.add_argument("--line-samples", type=int, default=6, help=hidden)
    parser.add_argument("--basket-descent-samples", type=int, default=8, help=hidden)
    parser.add_argument("--playback-period", type=float, default=0.0, help=hidden)
    parser.add_argument("--playback-rate", type=float, default=LOCKED_PLAYBACK_RATE, help=hidden)
    parser.add_argument("--trajectory-cartesian-step", type=float, default=0.025, help=hidden)
    parser.add_argument("--trajectory-joint-tolerance", type=float, default=0.004, help=hidden)
    parser.add_argument("--trajectory-max-duration", type=float, default=90.0, help=hidden)
    parser.add_argument("--planning-key-waypoints", default="approach,grasp,safe_mid,basket_over", help=hidden)
    parser.add_argument("--planner-backend", choices=("moveit", "local", "none"), default="moveit", help=hidden)
    parser.add_argument("--moveit-plan-service", default="/plan_kinematic_path", help=hidden)
    parser.add_argument(
        "--moveit-planners",
        default="RRTConnectkConfigDefault",
        help=hidden,
    )
    parser.add_argument("--moveit-planning-time", type=float, default=0.5, help=hidden)
    parser.add_argument("--moveit-planning-attempts", type=int, default=1, help=hidden)
    parser.add_argument("--moveit-max-goal-waypoints", type=int, default=3, help=hidden)
    parser.add_argument("--moveit-position-tolerance", type=float, default=0.015, help=hidden)
    parser.add_argument("--moveit-orientation-tolerance", type=float, default=1.20, help=hidden)
    parser.add_argument("--moveit-release-orientation-tolerance", type=float, default=0.55, help=hidden)
    parser.add_argument("--moveit-joint-goal-tolerance", type=float, default=0.025, help=hidden)
    parser.add_argument("--moveit-ik-orientation-tolerance", type=float, default=0.35, help=hidden)
    parser.add_argument("--moveit-ik-max-iterations", type=int, default=90, help=hidden)
    parser.add_argument("--moveit-service-timeout-padding", type=float, default=0.25, help=hidden)
    parser.add_argument("--moveit-soft-waypoint-position-tolerance", type=float, default=0.08, help=hidden)
    parser.add_argument("--moveit-soft-waypoint-orientation-tolerance", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-local-fallback-position-tolerance", type=float, default=0.04, help=hidden)
    parser.add_argument("--moveit-local-fallback-orientation-tolerance", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-grasp-fallback-position-tolerance", type=float, default=0.12, help=hidden)
    parser.add_argument("--moveit-grasp-fallback-orientation-tolerance", type=float, default=0.65, help=hidden)
    parser.add_argument("--moveit-pose-goal-on-ik-failure", action=argparse.BooleanOptionalAction, default=True, help=hidden)
    parser.add_argument(
        "--moveit-local-first",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument("--gripper-close-progress-start", type=float, default=0.30, help=hidden)
    parser.add_argument("--gripper-close-progress-end", type=float, default=0.42, help=hidden)
    parser.add_argument("--gripper-open-progress-start", type=float, default=0.84, help=hidden)
    parser.add_argument("--gripper-open-progress-end", type=float, default=0.94, help=hidden)
    parser.add_argument("--gripper-close-waypoint", default="grasp", help=hidden)
    parser.add_argument("--gripper-open-waypoint", default="basket_over", help=hidden)
    parser.add_argument("--gripper-transition-progress", type=float, default=0.025, help=hidden)
    parser.add_argument("--execute-real", action="store_true", help=hidden)
    parser.add_argument(
        "--execute-real-confirm",
        default=os.environ.get("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL", ""),
        help=hidden,
    )
    parser.add_argument(
        "--real-execute-backend",
        choices=("sdk_move_joint", "follow_joint_trajectory"),
        default="sdk_move_joint",
        help=hidden,
    )
    parser.add_argument(
        "--real-follow-action",
        default="/joint_trajectory_controller/follow_joint_trajectory",
        help=hidden,
    )
    parser.add_argument("--real-joint-states-topic", default="/joint_states", help=hidden)
    parser.add_argument(
        "--real-arm-joint-names",
        default=",".join(REAL_ARM_JOINT_NAMES),
        help=hidden,
    )
    parser.add_argument(
        "--real-start-joints",
        default=os.environ.get("ARACHNE_GRASP_ARM_JOINTS", ""),
        help=hidden,
    )
    parser.add_argument("--real-start-tolerance-rad", type=float, default=0.08, help=hidden)
    parser.add_argument("--real-start-state-timeout", type=float, default=3.0, help=hidden)
    parser.add_argument("--real-goal-time-margin", type=float, default=2.0, help=hidden)
    parser.add_argument("--real-action-timeout-padding", type=float, default=8.0, help=hidden)
    parser.add_argument("--real-trajectory-point-stride", type=int, default=4, help=hidden)
    parser.add_argument("--real-allow-partial", action="store_true", help=hidden)
    parser.add_argument("--real-sdk-ip", default=os.environ.get("AUBO_ROBOT_IP", "192.168.127.128"), help=hidden)
    parser.add_argument("--real-sdk-rpc-port", type=int, default=30004, help=hidden)
    parser.add_argument("--real-sdk-rpc-timeout", type=float, default=2.0, help=hidden)
    parser.add_argument("--real-sdk-teach-flag-path", default=DEFAULT_AUBO_TEACH_FLAG_PATH, help=hidden)
    parser.add_argument(
        "--real-sdk-control-owner-path",
        default=DEFAULT_AUBO_CONTROL_OWNER_PATH,
        help=hidden,
    )
    parser.add_argument(
        "--real-sdk-control-owner-name",
        default="grasp_task_server",
        help=hidden,
    )
    parser.add_argument("--real-sdk-gate-settle-sec", type=float, default=0.12, help=hidden)
    parser.add_argument("--real-sdk-move-speed", type=float, default=0.25, help=hidden)
    parser.add_argument("--real-sdk-move-accel", type=float, default=0.45, help=hidden)
    parser.add_argument("--real-sdk-blend-radius", type=float, default=0.0, help=hidden)
    parser.add_argument("--real-sdk-segment-duration-scale", type=float, default=0.0, help=hidden)
    parser.add_argument("--real-sdk-min-segment-duration", type=float, default=0.35, help=hidden)
    parser.add_argument("--real-sdk-goal-tolerance-rad", type=float, default=0.045, help=hidden)
    parser.add_argument("--real-sdk-arrival-timeout-padding", type=float, default=5.0, help=hidden)
    parser.add_argument("--real-sdk-max-segment-joint-delta", type=float, default=0.55, help=hidden)
    parser.add_argument("--real-sdk-max-targets", type=int, default=18, help=hidden)
    parser.add_argument(
        "--real-return-home",
        dest="real_return_home",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-real-return-home",
        dest="real_return_home",
        action="store_false",
        help=hidden,
    )
    parser.add_argument(
        "--real-home-joints",
        default=os.environ.get(
            "ARACHNE_AUBO_HOME_JOINTS_RAD",
            ",".join(str(value) for value in DEFAULT_ARM_JOINTS),
        ),
        help=hidden,
    )
    parser.add_argument("--real-home-duration", type=float, default=2.5, help=hidden)
    parser.add_argument(
        "--real-execute-gripper",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-real-execute-gripper",
        dest="real_execute_gripper",
        action="store_false",
        help=hidden,
    )
    parser.add_argument("--real-gripper-command-topic", default="/arachne/gripper/command", help=hidden)
    parser.add_argument("--real-gripper-settle-sec", type=float, default=0.35, help=hidden)
    parser.add_argument("--tool-orientation-limit-deg", type=float, default=90.0, help=hidden)
    parser.add_argument("--grasp-topdown-max-tilt-deg", type=float, default=65.0, help=hidden)
    parser.add_argument("--max-grasp-orientation-candidates", type=int, default=24, help=hidden)
    parser.add_argument("--grasp-orientation-yaw-offsets-deg", default="0,30,-30,60,-60,90,-90,180", help=hidden)
    parser.add_argument("--transit-orientation-yaw-offsets-deg", default="0,45,-45,90,-90", help=hidden)
    parser.add_argument("--grasp-orientation-tilt-offsets-deg", default="0,15,-15", help=hidden)
    parser.add_argument("--moveit-use-orientation-path-constraint", action="store_true", help=hidden)
    parser.add_argument("--moveit-max-tool0-reach", type=float, default=1.03, help=hidden)
    parser.add_argument("--moveit-velocity-scale", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-accel-scale", type=float, default=0.80, help=hidden)
    parser.add_argument("--loop-playback", action="store_true", help=hidden)
    parser.add_argument(
        "--preview-max-joint-speed",
        type=float,
        default=PREVIEW_MAX_JOINT_SPEED_RAD_SEC,
        help=hidden,
    )
    parser.add_argument(
        "--preview-max-joint-accel",
        type=float,
        default=PREVIEW_MAX_JOINT_ACCEL_RAD_SEC2,
        help=hidden,
    )
    parser.add_argument(
        "--preview-max-joint-jerk",
        type=float,
        default=PREVIEW_MAX_JOINT_JERK_RAD_SEC3,
        help=hidden,
    )
    parser.add_argument(
        "--preview-smoothing-tau",
        type=float,
        default=PREVIEW_SMOOTHING_TAU_SEC,
        help=hidden,
    )
    parser.add_argument("--save-dir", default="yolo_workspace/runs/grasp_preview", help=hidden)
    return parser.parse_args()


def _class_ids(model, spec: str) -> list[int] | None:
    tokens = [token.strip() for token in spec.split(",") if token.strip()]
    if not tokens:
        return None
    names = getattr(model, "names", {}) or {}
    name_to_id = {str(name).lower(): int(idx) for idx, name in names.items()}
    ids: list[int] = []
    for token in tokens:
        if token.isdigit():
            ids.append(int(token))
            continue
        key = token.lower()
        if key not in name_to_id:
            raise ValueError(f"unknown YOLO class {token!r}; known classes: {list(names.values())}")
        ids.append(name_to_id[key])
    return ids


def _xyz(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.replace(" ", ",").split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError(f"expected xyz triplet, got {value!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _float_values(value: str, expected: int, label: str) -> list[float]:
    parts = [
        part.strip()
        for part in str(value).replace(";", ",").replace(" ", ",").split(",")
        if part.strip()
    ]
    if len(parts) != expected:
        raise ValueError(f"{label} must contain {expected} values, got {len(parts)}")
    return [float(part) for part in parts]


def _float_list(value: str, label: str) -> list[float]:
    parts = [
        part.strip()
        for part in str(value).replace(";", ",").replace(" ", ",").split(",")
        if part.strip()
    ]
    if not parts:
        raise ValueError(f"{label} must contain at least one value")
    return [float(part) for part in parts]


def _header(stamp, frame_id: str) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def _point(xyz: Iterable[float]) -> Point:
    x, y, z = xyz
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def _image_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8", "mono8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    channels = 1 if msg.encoding == "mono8" else 3
    row_pixels = msg.step // channels
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        image = data.reshape(msg.height, msg.step)[:, : msg.width].copy()
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = data.reshape(msg.height, row_pixels, channels)[:, : msg.width, :].copy()
    if msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def _image_to_depth(msg: Image) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    row_pixels = msg.step // 2
    data = np.frombuffer(msg.data, dtype=np.uint16)
    return data.reshape(msg.height, row_pixels)[:, : msg.width].copy()


def _image_from_bgr(image: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    contiguous = np.ascontiguousarray(image)
    msg.step = int(contiguous.shape[1] * 3)
    msg.data = contiguous.tobytes()
    return msg


class GraspPreviewNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("grasp_preview_pipeline")
        self.args = args
        self.model_path = Path(args.model)
        self.venv = Path(args.venv)
        self.save_dir = Path(args.save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        initial_device = (
            str(args.onnx_device).strip()
            if self.model_path.suffix.lower() == ".onnx"
            else str(args.device_id)
        )
        self.model = _load_yolo(
            self.venv, self.model_path, str(args.yolo_task), initial_device
        )
        self.class_ids = _class_ids(self.model, args.classes)
        self.base_frame = str(args.base_frame)
        self.aubo_base_frame = str(args.aubo_base_frame)
        self.grasp_base_offset = _xyz(args.grasp_base_offset)
        self.basket_release_base = _xyz(args.basket_release_base)
        self.basket_approach_base = _xyz(args.basket_approach_base)
        self.basket_keepout_min = _xyz(args.basket_keepout_min_base)
        self.basket_keepout_max = _xyz(args.basket_keepout_max_base)
        self._clamp_basket_points_above_keepout()
        self.collision_boxes = self._make_collision_boxes()
        self.kinematics = AuboI5Kinematics()

        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.color_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self.current_arm_joints = np.asarray(DEFAULT_ARM_JOINTS, dtype=float)
        self.current_joint_positions: dict[str, float] = {}
        self.real_joint_positions: dict[str, float] = {}
        self.real_joint_state_time = 0.0
        self.real_execution_started = False
        self.real_execution_lock = threading.Lock()
        self.preview_ik_joints = np.asarray(DEFAULT_ARM_JOINTS, dtype=float)
        self.preview_ik_velocity = np.zeros(6, dtype=float)
        self.preview_ik_accel = np.zeros(6, dtype=float)
        self.preview_ik_last_update = time.monotonic()
        self.preview_ik_last_progress = 0.0
        self.preview_target_rotation = self.kinematics.fk(self.preview_ik_joints)[:3, :3]
        self.preview_ik_message = "waiting for plan"
        self.last_preview: GraspPreview | None = None
        self.depth_wait_detection: Detection | None = None
        self.depth_wait_reason = ""
        self.depth_wait_header: Header | None = None
        self.depth_wait_last_publish = 0.0
        self.snapshot_detection: Detection | None = None
        self.snapshot_header: Header | None = None
        self.snapshot_time = 0.0
        self.snapshot_count = 0
        self.inference_count = 0
        self.inference_paused = False
        self.planning_thread: threading.Thread | None = None
        self.planning_generation = 0
        self.planning_lock = threading.Lock()
        self.stopping = False
        self.plan_lock_time = 0.0
        self.tool_to_grasp_matrix: np.ndarray | None = None
        self.tool_to_grasp_frames: tuple[str, str] | None = None
        self.locked_visual_last_publish = 0.0
        self.missing_frames = 0
        self.last_log_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, args.color_topic, self._color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.color_info_topic, self._color_info_cb, 10)
        self.create_subscription(CameraInfo, args.depth_info_topic, self._depth_info_cb, 10)
        self.create_subscription(Empty, args.restart_search_topic, self._restart_search_cb, 10)
        self.create_subscription(JointState, args.joint_states_topic, self._joint_state_cb, 10)
        if bool(args.execute_real):
            self.create_subscription(
                JointState,
                args.real_joint_states_topic,
                self._real_joint_state_cb,
                qos_profile_sensor_data,
            )
        self.moveit_plan_client = (
            self.create_client(GetMotionPlan, str(args.moveit_plan_service))
            if str(args.planner_backend) == "moveit"
            else None
        )
        self.real_arm_action_client = (
            ActionClient(self, FollowJointTrajectory, str(args.real_follow_action))
            if bool(args.execute_real)
            and str(args.real_execute_backend) == "follow_joint_trajectory"
            else None
        )
        self.real_gripper_pub = (
            self.create_publisher(String, str(args.real_gripper_command_topic), 10)
            if bool(args.execute_real)
            else None
        )

        self.markers_pub = self.create_publisher(
            MarkerArray, "/arachne/grasp_preview/markers", 10
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "/arachne/grasp_preview/roi_cloud", 10)
        self.path_pub = self.create_publisher(PathMsg, "/arachne/grasp_preview/path", 10)
        self.image_pub = self.create_publisher(
            Image, "/arachne/grasp_preview/annotated_image", 10
        )
        self.arm_preview_pub = self.create_publisher(
            JointState, "/arachne/grasp_preview/joint_states", 10
        )

        period = max(float(args.inference_period), 0.05)
        playback_rate = max(float(args.playback_rate), 1.0)
        self.create_timer(period, self._tick)
        self.create_timer(1.0 / playback_rate, self._playback_tick)
        self.get_logger().info(
            "grasp preview ready: "
            f"model={self.model_path} classes={args.classes or 'all'} "
            f"inference_device={self._yolo_device()} "
            "markers=/arachne/grasp_preview/markers "
            "roi_cloud=/arachne/grasp_preview/roi_cloud "
            f"restart_topic={args.restart_search_topic}"
        )
        if bool(args.execute_real):
            self.get_logger().warn(
                "real execution armed; motion will be sent only after confirmation and start-state checks "
                f"backend={args.real_execute_backend}"
            )

    def _clamp_basket_points_above_keepout(self) -> None:
        top = (
            self.basket_keepout_max[2]
            + max(float(self.args.gripper_radius), 0.0)
            + float(self.args.basket_clearance)
        )
        approach_top = top + 0.12
        rx, ry, rz = self.basket_release_base
        ax, ay, az = self.basket_approach_base
        self.basket_release_base = (rx, ry, max(rz, top))
        self.basket_approach_base = (ax, ay, max(az, approach_top))

    def _make_collision_boxes(self) -> list[CollisionBox]:
        margin = max(float(self.args.collision_margin), 0.0)

        def padded(size: tuple[float, float, float]) -> tuple[float, float, float]:
            return tuple(float(value) + 2.0 * margin for value in size)  # type: ignore[return-value]

        basket_center = (
            0.5 * (self.basket_keepout_min[0] + self.basket_keepout_max[0]),
            0.5 * (self.basket_keepout_min[1] + self.basket_keepout_max[1]),
            0.5 * (self.basket_keepout_min[2] + self.basket_keepout_max[2]),
        )
        basket_size = (
            self.basket_keepout_max[0] - self.basket_keepout_min[0],
            self.basket_keepout_max[1] - self.basket_keepout_min[1],
            self.basket_keepout_max[2] - self.basket_keepout_min[2],
        )
        return [
            CollisionBox("scout_base_main", (0.0, 0.0, 0.008), padded((0.925, 0.380, 0.210))),
            CollisionBox(
                "scout_base_center_ridge",
                (0.0, 0.0, 0.210 / 6.0),
                padded((0.925 / 6.0, 0.380 * 1.65, 0.210 / 3.0)),
            ),
            CollisionBox("front_basket_keepout", basket_center, padded(basket_size)),
            CollisionBox("arm_mount", (0.22, 0.0, 0.155 - 0.025), padded((0.26, 0.22, 0.05))),
        ]

    def _color_cb(self, msg: Image) -> None:
        self.latest_color = msg

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg

    def _color_info_cb(self, msg: CameraInfo) -> None:
        self.color_info = msg

    def _depth_info_cb(self, msg: CameraInfo) -> None:
        self.depth_info = msg

    def _joint_state_cb(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.current_joint_positions[name] = float(msg.position[index])

        values: list[float] = []
        for name in ARM_JOINT_NAMES:
            if name in msg.name:
                index = msg.name.index(name)
                if index < len(msg.position):
                    values.append(float(msg.position[index]))
        if len(values) == 6:
            self.current_arm_joints = np.asarray(values, dtype=float)

    def _real_joint_state_cb(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.real_joint_positions[name] = float(msg.position[index])
        self.real_joint_state_time = time.monotonic()

    def _moveit_start_state_joint_values(self, q_start: np.ndarray) -> tuple[list[str], list[float]]:
        joint_values: dict[str, float] = {
            name: float(value) for name, value in zip(ARM_JOINT_NAMES, q_start)
        }
        gripper_type = str(getattr(self.args, "gripper_type", "ms42dc"))
        for name, default in (
            tuple(BASE_JOINT_DEFAULTS) + tuple(GRIPPER_JOINT_DEFAULTS.get(gripper_type, ()))
        ):
            joint_values[name] = float(self.current_joint_positions.get(name, default))
        for name, value in self.current_joint_positions.items():
            if name not in joint_values and name not in ARM_JOINT_NAMES:
                joint_values[name] = float(value)
        return list(joint_values.keys()), list(joint_values.values())

    def _moveit_unreachable_note(self, target_base: tuple[float, float, float]) -> str:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return ""
        target_aubo = np.asarray(self._transform_point(transform, target_base), dtype=float)
        radius = float(np.linalg.norm(target_aubo))
        max_reach = max(float(self.args.moveit_max_tool0_reach), 0.1)
        tolerance = max(float(self.args.moveit_position_tolerance), 0.0)
        if radius <= max_reach + tolerance:
            return ""
        ax, ay, az = target_aubo
        return (
            f"aubo_target=({ax:.3f},{ay:.3f},{az:.3f}) "
            f"radius={radius:.3f}m exceeds tool0 reach {max_reach:.3f}m"
        )

    def _restart_search_cb(self, _msg: Empty) -> None:
        self._restart_search("restart-topic")

    def _reset_preview_stream(self, message: str) -> None:
        self.preview_ik_joints = np.asarray(self.current_arm_joints, dtype=float)
        self.preview_ik_velocity = np.zeros(6, dtype=float)
        self.preview_ik_accel = np.zeros(6, dtype=float)
        self.preview_ik_last_update = time.monotonic()
        self.preview_ik_last_progress = 0.0
        self.preview_target_rotation = self.kinematics.fk(self.preview_ik_joints)[:3, :3]
        self.preview_ik_message = message

    def _restart_search(self, reason: str) -> None:
        self.inference_paused = False
        self.plan_lock_time = 0.0
        self.last_preview = None
        self.depth_wait_detection = None
        self.depth_wait_reason = ""
        self.depth_wait_header = None
        self.depth_wait_last_publish = 0.0
        self.planning_generation += 1
        with self.real_execution_lock:
            self.real_execution_started = False
        self._reset_preview_stream("waiting for plan")
        self.snapshot_detection = None
        self.snapshot_header = None
        self.snapshot_time = 0.0
        self.missing_frames = 0
        header = self.latest_color.header if self.latest_color is not None else Header()
        self._clear_markers(header)
        self.get_logger().info(f"restart 2D search: {reason}")

    def _tick(self) -> None:
        if self.latest_color is None:
            self._throttled_log("waiting for color topic")
            return
        if self.depth_wait_detection is not None:
            self._tick_waiting_for_depth()
            return
        if self.inference_paused and self.last_preview is not None:
            now = time.monotonic()
            locked_period = 1.0 / max(float(self.args.locked_visual_rate), 1.0)
            if now - self.locked_visual_last_publish < locked_period:
                return
            try:
                color = _image_to_bgr(self.latest_color)
            except ValueError as exc:
                self._throttled_log(str(exc))
                return
            self._publish_locked_annotation(color, self.latest_color.header)
            return
        try:
            color = _image_to_bgr(self.latest_color)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return

        result = self._predict(color)
        detection = self._best_detection(result)
        annotated = result.plot()
        header = self.latest_color.header
        if detection is None:
            self.missing_frames += 1
            cv2.putText(
                annotated,
                "no target detection",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self.image_pub.publish(_image_from_bgr(annotated, header))
            if self.missing_frames >= max(int(self.args.lost_frame_threshold), 1):
                self.last_preview = None
                self.snapshot_detection = None
                self.snapshot_header = None
                self._clear_markers(header)
            self._throttled_log("no target detection")
            return

        self.missing_frames = 0
        needs_snapshot, reason = self._needs_depth_snapshot(detection, color.shape)
        if needs_snapshot:
            if self.latest_depth is None or self.depth_info is None:
                self.depth_wait_detection = detection
                self.depth_wait_reason = reason
                self.depth_wait_header = header
                self.inference_paused = True
                self._publish_wait_depth_annotation(color, header)
                self._throttled_log(
                    f"2D target locked; YOLO paused; waiting for {self._missing_depth_text()}"
                )
                return
            try:
                depth = _image_to_depth(self.latest_depth)
            except ValueError as exc:
                self._throttled_log(str(exc))
                return
            preview = self._make_preview(detection, depth, reason)
        elif self.last_preview is not None:
            preview = replace(self.last_preview, detection=detection, snapshot_reason="cached")
        else:
            preview = None

        if preview is None:
            cv2.putText(
                annotated,
                "detection has no valid depth",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self.image_pub.publish(_image_from_bgr(annotated, header))
            self._clear_markers(header)
            self._throttled_log("detection found, but ROI depth is invalid")
            return

        if needs_snapshot:
            self.snapshot_header = self.latest_depth.header if self.latest_depth is not None else header
            self.snapshot_detection = detection
            self.snapshot_time = time.monotonic()
            self.snapshot_count += 1
            self.inference_paused = True
            self.plan_lock_time = self.snapshot_time
            self._reset_preview_stream("plan locked")
        preview_header = self.snapshot_header or (
            self.latest_depth.header if self.latest_depth is not None else header
        )
        self.last_preview = preview
        self._publish_preview(preview, preview_header)
        self._publish_annotated(annotated, preview, header)
        self._save_latest(annotated, color, preview)
        if needs_snapshot:
            self._start_arm_planning(preview, preview_header)

    def _missing_depth_text(self) -> str:
        missing = []
        if self.latest_depth is None:
            missing.append(str(self.args.depth_topic))
        if self.depth_info is None:
            missing.append(str(self.args.depth_info_topic))
        return ", ".join(missing) if missing else "valid ROI depth"

    def _tick_waiting_for_depth(self) -> None:
        detection = self.depth_wait_detection
        if detection is None:
            return
        try:
            color = _image_to_bgr(self.latest_color)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return
        header = self.latest_color.header
        if self.latest_depth is None or self.depth_info is None:
            now = time.monotonic()
            if now - self.depth_wait_last_publish >= 0.5:
                self._publish_wait_depth_annotation(color, header)
                self._throttled_log(
                    f"YOLO paused on locked target; waiting for {self._missing_depth_text()}"
                )
            return
        try:
            depth = _image_to_depth(self.latest_depth)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return

        preview = self._make_preview(detection, depth, self.depth_wait_reason or "new-target")
        if preview is None:
            now = time.monotonic()
            if now - self.depth_wait_last_publish >= 0.5:
                self._publish_wait_depth_annotation(
                    color, header, "locked target; waiting for valid ROI depth"
                )
                self._throttled_log("YOLO paused on locked target; waiting for valid ROI depth")
            return

        self.depth_wait_detection = None
        self.depth_wait_reason = ""
        self.depth_wait_header = None
        self.snapshot_header = self.latest_depth.header
        self.snapshot_detection = detection
        self.snapshot_time = time.monotonic()
        self.snapshot_count += 1
        self.plan_lock_time = self.snapshot_time
        self._reset_preview_stream("planning pending")
        preview_header = self.snapshot_header
        self.last_preview = preview
        annotated = color.copy()
        self._draw_detection_overlay(annotated, detection, "LOCKED")
        self._publish_preview(preview, preview_header)
        self._publish_annotated(annotated, preview, header)
        self._save_latest(annotated, color, preview)
        self._start_arm_planning(preview, preview_header)

    def _draw_detection_overlay(
        self, image: np.ndarray, detection: Detection, prefix: str, footer: str | None = None
    ) -> None:
        x1, y1, x2, y2 = (int(round(v)) for v in detection.xyxy)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 190, 255), 2)
        if detection.mask_xy:
            polygon = np.asarray(detection.mask_xy, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [polygon], isClosed=True, color=(0, 255, 80), thickness=2)
        cv2.putText(
            image,
            f"{prefix} {detection.label} {detection.confidence:.2f}",
            (max(x1, 8), max(y1 - 8, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 190, 255),
            2,
            cv2.LINE_AA,
        )
        if footer:
            cv2.putText(
                image,
                footer,
                (12, max(image.shape[0] - 18, 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 190, 255),
                2,
                cv2.LINE_AA,
            )

    def _publish_wait_depth_annotation(
        self, color: np.ndarray, header: Header, message: str | None = None
    ) -> None:
        detection = self.depth_wait_detection
        if detection is None:
            return
        self.depth_wait_last_publish = time.monotonic()
        annotated = color.copy()
        text = message or f"YOLO paused; waiting for {self._missing_depth_text()}"
        self._draw_detection_overlay(annotated, detection, "LOCKED_2D", text)
        cv2.putText(
            annotated,
            text,
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 180, 255),
            2,
            cv2.LINE_AA,
        )
        self.image_pub.publish(_image_from_bgr(annotated, header))

    def _start_arm_planning(self, preview: GraspPreview, preview_header: Header) -> None:
        if str(self.args.planner_backend) == "none":
            perception = replace(
                preview,
                base_path_xyz=self._sample_trajectory_for_display(preview.base_trajectory_segments),
                ik_message="perception_only: shape target ready; planning skipped",
            )
            self.last_preview = perception
            self.preview_ik_message = perception.ik_message
            self.plan_lock_time = time.monotonic()
            self._publish_preview(perception, preview_header)
            self.get_logger().info("3D snapshot published; arm planning skipped")
            return
        with self.planning_lock:
            if self.planning_thread is not None and self.planning_thread.is_alive():
                return
            self.planning_generation += 1
            generation = self.planning_generation
            self.preview_ik_message = "planning trajectory"
            self.planning_thread = threading.Thread(
                target=self._arm_planning_worker,
                args=(generation, preview, preview_header),
                daemon=True,
            )
            self.planning_thread.start()
        self.get_logger().info("3D snapshot published; arm trajectory planning started")

    def _arm_planning_worker(
        self, generation: int, preview: GraspPreview, preview_header: Header
    ) -> None:
        start = time.monotonic()
        arm_frames, ik_message = self._make_constrained_arm_trajectory(preview)
        if self.stopping or not rclpy.ok():
            return
        planned_path = self._arm_trajectory_grasp_path_base(arm_frames) if arm_frames else []
        planned = replace(
            preview,
            base_path_xyz=planned_path,
            arm_trajectory_frames=arm_frames,
            ik_message=ik_message,
        )
        with self.planning_lock:
            if generation != self.planning_generation:
                return
            self.last_preview = planned
            self.preview_ik_message = ik_message
            if arm_frames:
                self.plan_lock_time = time.monotonic()
        if self.stopping or not rclpy.ok():
            return
        try:
            self._publish_preview(planned, preview_header)
        except RuntimeError as exc:
            if "publisher's context is invalid" in str(exc):
                return
            raise
        elapsed = time.monotonic() - start
        if arm_frames and "partial" not in ik_message:
            self.get_logger().info(
                f"arm trajectory ready: frames={len(arm_frames)} "
                f"duration={arm_frames[-1].time_from_start:.2f}s "
                f"planning_wall={elapsed:.2f}s {ik_message}"
            )
            self._maybe_execute_real_trajectory(planned)
        elif arm_frames:
            self.get_logger().warn(
                f"arm trajectory partial after {elapsed:.2f}s: frames={len(arm_frames)} "
                f"duration={arm_frames[-1].time_from_start:.2f}s {ik_message}"
            )
            self._maybe_execute_real_trajectory(planned)
        else:
            self.get_logger().warn(f"arm trajectory unavailable after {elapsed:.2f}s: {ik_message}")

    def stop(self) -> None:
        self.stopping = True
        with self.planning_lock:
            self.planning_generation += 1
            thread = self.planning_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.3)

    def _maybe_execute_real_trajectory(self, preview: GraspPreview) -> None:
        if not bool(self.args.execute_real):
            return
        with self.real_execution_lock:
            if self.real_execution_started:
                return
            self.real_execution_started = True

        try:
            self._execute_real_trajectory(preview)
        except Exception as exc:
            self.get_logger().error(f"real execution blocked: {exc}")

    def _execute_real_trajectory(self, preview: GraspPreview) -> None:
        if not preview.arm_trajectory_frames:
            raise RuntimeError("planned trajectory is empty")
        if "partial" in preview.ik_message and not bool(self.args.real_allow_partial):
            raise RuntimeError(f"refusing partial trajectory: {preview.ik_message}")
        if str(self.args.execute_real_confirm).strip().upper() != "YES":
            raise RuntimeError(
                "set ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES or pass --execute-real-confirm YES"
            )
        if not self._real_start_state_matches(preview.arm_trajectory_frames[0]):
            return
        backend = str(self.args.real_execute_backend)
        if backend == "sdk_move_joint":
            self._execute_real_sdk_move_joint(preview)
            return
        if backend == "follow_joint_trajectory":
            self._execute_real_follow_joint_trajectory(preview)
            return
        raise RuntimeError(f"unsupported real execution backend: {backend}")

    def _execute_real_sdk_move_joint(self, preview: GraspPreview) -> None:
        frames = preview.arm_trajectory_frames
        targets = self._real_sdk_move_targets(preview, frames)
        if not targets:
            raise RuntimeError("real SDK moveJoint target list is empty")

        ip = str(self.args.real_sdk_ip)
        port = int(self.args.real_sdk_rpc_port)
        timeout = max(float(self.args.real_sdk_rpc_timeout), 0.1)
        speed = max(float(self.args.real_sdk_move_speed), 0.01)
        accel = max(float(self.args.real_sdk_move_accel), 0.05)
        blend_radius = max(float(self.args.real_sdk_blend_radius), 0.0)
        duration_scale = max(float(self.args.real_sdk_segment_duration_scale), 0.0)
        owner_owned = False
        gate_owned = False
        self.get_logger().warn(
            f"sending REAL arm motion through Aubo SDK moveJoint: targets={len(targets)} "
            f"rpc={ip}:{port} speed={speed:.3f}rad/s accel={accel:.3f}rad/s2"
        )

        with AuboDirectJsonRpc(ip, port, timeout) as rpc:
            try:
                self._real_sdk_require_running(rpc)
                owner_owned = self._real_sdk_enter_control_owner()
                gate_owned = self._real_sdk_enter_gate()
                self._real_sdk_exit_servo_mode(rpc)
                self._real_sdk_stop_joint(rpc, "pre-move cleanup", warn_only=True)
                if bool(self.args.real_execute_gripper):
                    self._publish_real_gripper("open")
                    settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                    if settle > 0.0:
                        time.sleep(settle)

                previous_time = 0.0
                closed = False
                opened = False
                (
                    _close_start,
                    _close_end,
                    _open_start,
                    _open_end,
                    close_label,
                    open_label,
                ) = self._gripper_event_progresses(preview)
                for index, (label, frame) in enumerate(targets, start=1):
                    if self.stopping or not rclpy.ok():
                        return
                    target = [float(value) for value in frame.positions]
                    segment_dt = max(float(frame.time_from_start) - previous_time, 0.0)
                    duration = 0.0
                    if duration_scale > 0.0:
                        duration = max(
                            segment_dt * duration_scale,
                            max(float(self.args.real_sdk_min_segment_duration), 0.0),
                        )
                    result = rpc.robot_call(
                        "MotionControl.moveJoint",
                        [target, accel, speed, blend_radius, duration],
                    )
                    if result not in (0, None):
                        raise RuntimeError(f"Aubo SDK moveJoint failed at {label}: result={result}")
                    self.get_logger().warn(
                        f"REAL moveJoint {index}/{len(targets)} {label}: "
                        f"t={frame.time_from_start:.2f}s duration={duration:.2f}s"
                    )
                    self._real_sdk_wait_arrival(rpc, np.asarray(target, dtype=float), label, segment_dt)
                    previous_time = float(frame.time_from_start)

                    if bool(self.args.real_execute_gripper) and not closed:
                        if label == close_label:
                            self._publish_real_gripper("close")
                            closed = True
                            settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                            if settle > 0.0:
                                time.sleep(settle)
                    if bool(self.args.real_execute_gripper) and not opened:
                        if label == open_label:
                            self._publish_real_gripper("open")
                            opened = True
                            settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                            if settle > 0.0:
                                time.sleep(settle)
                if not self.stopping and rclpy.ok():
                    self.get_logger().warn("REAL arm SDK moveJoint sequence complete")
            finally:
                try:
                    if gate_owned:
                        self._real_sdk_stop_joint(rpc, "post-move cleanup", warn_only=True)
                finally:
                    self._real_sdk_exit_gate(gate_owned)
                    self._real_sdk_exit_control_owner(owner_owned)

    def _execute_real_follow_joint_trajectory(self, preview: GraspPreview) -> None:
        if self.real_arm_action_client is None:
            raise RuntimeError("real FollowJointTrajectory action client is not configured")
        action_name = str(self.args.real_follow_action)
        if not self.real_arm_action_client.wait_for_server(timeout_sec=2.0):
            raise TimeoutError(f"real arm action server unavailable: {action_name}")

        frames = preview.arm_trajectory_frames
        if not bool(self.args.real_execute_gripper):
            self._send_real_follow_joint_segment(frames, 0, len(frames) - 1, "full")
            self.get_logger().warn("REAL arm trajectory complete")
            return

        close_start, _close_end, open_start, _open_end, _close_label, _open_label = (
            self._gripper_event_progresses(preview)
        )
        semantic_indices = self._real_sdk_semantic_frame_indices(preview, frames)
        close_index = semantic_indices.get(str(self.args.gripper_close_waypoint).strip())
        open_index = semantic_indices.get(str(self.args.gripper_open_waypoint).strip())
        if close_index is None:
            close_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * close_start
            )
        if open_index is None:
            open_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * open_start
            )
        close_index = min(max(int(close_index), 1), len(frames) - 1)
        open_index = min(max(int(open_index), close_index), len(frames) - 1)

        self._publish_real_gripper("open")
        settle = max(float(self.args.real_gripper_settle_sec), 0.0)
        if settle > 0.0:
            time.sleep(settle)
        self._send_real_follow_joint_segment(frames, 0, close_index, "to_grasp")
        self._publish_real_gripper("close")
        if settle > 0.0:
            time.sleep(settle)
        self._send_real_follow_joint_segment(frames, close_index, open_index, "to_release")
        self._publish_real_gripper("open")
        if settle > 0.0:
            time.sleep(settle)
        self._send_real_follow_joint_segment(frames, open_index, len(frames) - 1, "finish")
        self.get_logger().warn("REAL arm trajectory complete")

    def _send_real_follow_joint_segment(
        self,
        frames: list[JointTrajectoryFrame],
        start_index: int,
        end_index: int,
        label: str,
    ) -> None:
        start_index = min(max(int(start_index), 0), len(frames) - 1)
        end_index = min(max(int(end_index), start_index), len(frames) - 1)
        if end_index <= start_index:
            return
        segment = self._rebased_trajectory_frames(frames[start_index : end_index + 1])
        trajectory = self._real_joint_trajectory_msg(segment)
        duration = float(segment[-1].time_from_start)
        action_name = str(self.args.real_follow_action)
        self.get_logger().warn(
            f"sending REAL arm trajectory segment {label}: points={len(trajectory.points)} "
            f"duration={duration:.2f}s action={action_name}"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        margin = max(float(self.args.real_goal_time_margin), 0.0)
        goal.goal_time_tolerance.sec = int(margin)
        goal.goal_time_tolerance.nanosec = int((margin % 1.0) * 1e9)

        goal_future = self.real_arm_action_client.send_goal_async(goal)
        if not self._wait_future(goal_future, 3.0):
            raise TimeoutError(f"real arm action segment {label} goal response timed out")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"real arm action segment {label} goal rejected")

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        timeout = duration + max(float(self.args.real_action_timeout_padding), 0.0)
        while rclpy.ok() and not self.stopping and not result_future.done():
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                cancel_future = goal_handle.cancel_goal_async()
                self._wait_future(cancel_future, 1.0)
                raise TimeoutError(
                    f"real arm action segment {label} result timed out after {elapsed:.2f}s"
                )
            time.sleep(0.02)

        if self.stopping or not rclpy.ok():
            return
        result_response = result_future.result()
        result = result_response.result if result_response is not None else None
        if result is None:
            raise RuntimeError(f"real arm action segment {label} returned empty result")
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"real arm action segment {label} failed: "
                f"code={result.error_code} {result.error_string}"
            )

    def _real_sdk_move_targets(
        self, preview: GraspPreview, frames: list[JointTrajectoryFrame]
    ) -> list[tuple[str, JointTrajectoryFrame]]:
        if len(frames) <= 1:
            return []
        close_start, _close_end, open_start, _open_end, close_label, open_label = (
            self._gripper_event_progresses(preview)
        )
        semantic_indices = self._real_sdk_semantic_frame_indices(preview, frames)
        close_index = semantic_indices.get(str(self.args.gripper_close_waypoint).strip())
        open_index = semantic_indices.get(str(self.args.gripper_open_waypoint).strip())
        if close_index is None:
            close_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * close_start
            )
        if open_index is None:
            open_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * open_start
            )
        close_index = min(max(int(close_index), 1), len(frames) - 1)
        open_index = min(max(int(open_index), close_index), len(frames) - 1)
        if open_index <= close_index and close_index < len(frames) - 1:
            open_index = close_index + 1
        selected: dict[int, str] = {}
        planned_names = {
            name for name, _point, _progress in self._planning_target_samples(preview)
        }
        semantic_labels = {
            "approach": "approach",
            "grasp": close_label,
            "safe_mid": "safe_mid",
            "basket_over": open_label,
            "drop": "drop",
        }
        for name, label in semantic_labels.items():
            if name not in planned_names:
                continue
            index = semantic_indices.get(name)
            if index is not None and index > 0:
                if name == "grasp":
                    index = close_index
                elif name == str(self.args.gripper_open_waypoint).strip():
                    index = open_index
                selected[min(max(int(index), 1), len(frames) - 1)] = label
        selected[close_index] = close_label
        selected[open_index] = open_label

        max_delta = max(float(self.args.real_sdk_max_segment_joint_delta), 0.05)
        max_targets = max(int(self.args.real_sdk_max_targets), 2)
        last_q = np.asarray(frames[0].positions, dtype=float)
        safe_mid_index = semantic_indices.get("safe_mid", open_index)
        for index, frame in enumerate(frames[1:], start=1):
            if index in selected:
                last_q = np.asarray(frame.positions, dtype=float)
                continue
            if close_index < index < safe_mid_index:
                continue
            delta = float(np.max(np.abs(self._joint_delta(np.asarray(frame.positions, dtype=float), last_q))))
            if delta >= max_delta:
                selected[index] = f"segment_{len(selected) + 1}"
                last_q = np.asarray(frame.positions, dtype=float)
            if len(selected) >= max_targets:
                break

        if (len(frames) - 1) not in selected:
            selected[len(frames) - 1] = "final"
        ordered = sorted(selected.items(), key=lambda item: item[0])
        if len(ordered) > max_targets:
            keep = {
                index
                for index, label in ordered
                if label in {close_label, open_label, "safe_mid", "final"}
            }
            remaining = [index for index, _label in ordered if index not in keep]
            budget = max(max_targets - len(keep), 0)
            if budget > 0 and remaining:
                step = max(int(math.ceil(len(remaining) / float(budget))), 1)
                keep.update(remaining[::step][:budget])
            ordered = [(index, selected[index]) for index in sorted(keep)]
        targets = [(label, frames[index]) for index, label in ordered if index > 0]
        if bool(self.args.real_return_home):
            targets.append(("home", self._real_home_frame(frames[-1])))
        return targets

    def _real_sdk_semantic_frame_indices(
        self, preview: GraspPreview, frames: list[JointTrajectoryFrame]
    ) -> dict[str, int]:
        waypoint_map = {name: xyz for name, xyz in preview.base_waypoints}
        planned_names = {
            name for name, _point, _progress in self._planning_target_samples(preview)
        }
        names = [
            name
            for name in ["approach", "grasp", "safe_mid", "basket_over", "drop"]
            if name in planned_names
        ]
        targets = {name: waypoint_map[name] for name in names if name in waypoint_map}
        if not targets:
            return {}
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return {}
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return {}
        base_from_aubo = self._matrix_from_transform(transform)
        grasp_points = []
        for frame in frames:
            tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
            grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
            grasp_points.append(np.asarray(grasp_in_base[:3, 3], dtype=float))
        result: dict[str, int] = {}
        previous_index = 1
        for name in names:
            target = targets.get(name)
            if target is None:
                continue
            target_vec = np.asarray(target, dtype=float)
            start = min(max(previous_index, 1), len(frames) - 1)
            distances = [
                float(np.linalg.norm(point - target_vec)) if index >= start else float("inf")
                for index, point in enumerate(grasp_points)
            ]
            index = int(np.argmin(distances))
            if not np.isfinite(distances[index]):
                continue
            result[name] = index
            previous_index = min(max(index + 1, previous_index), len(frames) - 1)
        return result

    def _real_home_frame(self, previous_frame: JointTrajectoryFrame) -> JointTrajectoryFrame:
        positions = tuple(_float_values(str(self.args.real_home_joints), 6, "--real-home-joints"))
        duration = max(float(self.args.real_home_duration), 0.0)
        zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return JointTrajectoryFrame(
            float(previous_frame.time_from_start) + duration,
            positions,  # type: ignore[arg-type]
            zeros,
            zeros,
        )

    def _frame_index_at_or_after(self, frames: list[JointTrajectoryFrame], target_time: float) -> int:
        for index, frame in enumerate(frames):
            if float(frame.time_from_start) >= float(target_time):
                return index
        return len(frames) - 1

    def _real_frame_progress(
        self, frames: list[JointTrajectoryFrame], frame: JointTrajectoryFrame
    ) -> float:
        duration = max(float(frames[-1].time_from_start), 1e-6)
        return min(max(float(frame.time_from_start) / duration, 0.0), 1.0)

    def _normalise_sdk_joints(self, value: object) -> list[float]:
        if isinstance(value, dict):
            for key in ("joint_positions", "jointPositions", "positions", "q", "data", "value"):
                if key in value:
                    return self._normalise_sdk_joints(value[key])
        if isinstance(value, (list, tuple)) and len(value) >= 6:
            joints = [float(item) for item in value[:6]]
            if max(abs(item) for item in joints) > 10.0:
                raise RuntimeError(f"Aubo SDK joint values do not look like radians: {joints}")
            return joints
        raise RuntimeError(f"cannot parse Aubo SDK joint positions from {value!r}")

    def _real_sdk_joint_positions(self, rpc: AuboDirectJsonRpc) -> np.ndarray:
        return np.asarray(
            self._normalise_sdk_joints(rpc.robot_call("RobotState.getJointPositions")),
            dtype=float,
        )

    def _real_sdk_require_running(self, rpc: AuboDirectJsonRpc) -> None:
        mode = str(rpc.robot_call("RobotState.getRobotModeType")).strip().lower()
        safety = str(rpc.robot_call("RobotState.getSafetyModeType")).strip().lower()
        if mode != "running" or safety not in ("normal", "reducedmode"):
            raise RuntimeError(
                "Aubo SDK execution requires Running/Normal or ReducedMode: "
                f"mode={mode} safety={safety}"
            )

    def _real_sdk_enter_gate(self) -> bool:
        path = Path(str(self.args.real_sdk_teach_flag_path))
        if path.exists():
            try:
                text = path.read_text(encoding="utf-8", errors="replace").strip()
            except OSError as exc:
                raise RuntimeError(f"real SDK teach gate is unreadable: {path}: {exc}") from exc
            raise RuntimeError(
                "real SDK teach gate is already active; turn teach/jog off before grasp "
                f"path={path} value={text!r}"
            )
        path.write_text("1\n", encoding="utf-8")
        settle = max(float(self.args.real_sdk_gate_settle_sec), 0.0)
        if settle > 0.0:
            time.sleep(settle)
        return True

    def _real_sdk_enter_control_owner(self) -> bool:
        path = Path(str(self.args.real_sdk_control_owner_path))
        owner = str(self.args.real_sdk_control_owner_name).strip() or "grasp_task_server"
        ok, message = _claim_control_owner(path, owner)
        if not ok:
            raise RuntimeError(
                "Aubo control is busy; stop teach/manual jog before real grasp: "
                f"{message}"
            )
        self.get_logger().warn(f"REAL Aubo control owner acquired: {message}")
        return True

    def _real_sdk_exit_control_owner(self, owner_owned: bool) -> None:
        if not owner_owned:
            return
        path = Path(str(self.args.real_sdk_control_owner_path))
        owner = str(self.args.real_sdk_control_owner_name).strip() or "grasp_task_server"
        _release_control_owner(path, owner)

    def _real_sdk_exit_gate(self, gate_owned: bool) -> None:
        if not gate_owned:
            return
        path = Path(str(self.args.real_sdk_teach_flag_path))
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().error(f"failed to release real SDK teach gate {path}: {exc}")

    def _real_sdk_exit_servo_mode(self, rpc: AuboDirectJsonRpc) -> None:
        try:
            result = rpc.robot_call("MotionControl.setServoModeSelect", [0])
            if result not in (0, None):
                self.get_logger().warn(f"Aubo SDK setServoModeSelect(0) result={result}")
            return
        except Exception as exc:
            self.get_logger().warn(
                f"Aubo SDK setServoModeSelect unavailable, trying setServoMode(false): {exc}"
            )
        result = rpc.robot_call("MotionControl.setServoMode", [False])
        if result not in (0, None):
            self.get_logger().warn(f"Aubo SDK setServoMode(false) result={result}")

    def _real_sdk_stop_joint(
        self, rpc: AuboDirectJsonRpc, reason: str, *, warn_only: bool = False
    ) -> None:
        accel = max(float(self.args.real_sdk_move_accel), 0.05)
        try:
            result = rpc.robot_call("MotionControl.stopJoint", [accel])
        except Exception as exc:
            if warn_only:
                self.get_logger().warn(f"Aubo SDK stopJoint failed during {reason}: {exc}")
                return
            raise
        if result not in (0, None):
            message = f"Aubo SDK stopJoint result={result} during {reason}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def _real_sdk_wait_arrival(
        self,
        rpc: AuboDirectJsonRpc,
        target: np.ndarray,
        label: str,
        planned_segment_dt: float,
    ) -> None:
        tolerance = max(float(self.args.real_sdk_goal_tolerance_rad), 0.001)
        speed = max(float(self.args.real_sdk_move_speed), 0.01)
        current = self._real_sdk_joint_positions(rpc)
        max_delta = float(np.max(np.abs(self._joint_delta(target, current))))
        timeout = max(
            planned_segment_dt,
            max_delta / speed,
            0.5,
        ) + max(float(self.args.real_sdk_arrival_timeout_padding), 0.0)
        deadline = time.monotonic() + timeout
        last_error = max_delta
        while rclpy.ok() and not self.stopping and time.monotonic() < deadline:
            current = self._real_sdk_joint_positions(rpc)
            last_error = float(np.max(np.abs(self._joint_delta(target, current))))
            if last_error <= tolerance:
                return
            time.sleep(0.05)
        raise TimeoutError(
            f"Aubo SDK moveJoint arrival timeout at {label}: "
            f"max_error={last_error:.3f}rad tolerance={tolerance:.3f}rad"
        )

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while rclpy.ok() and not self.stopping and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return bool(future.done())

    def _real_arm_joint_names(self) -> list[str]:
        names = [
            token.strip()
            for token in str(self.args.real_arm_joint_names).replace(";", ",").split(",")
            if token.strip()
        ]
        if len(names) != 6:
            raise ValueError("--real-arm-joint-names must contain 6 joint names")
        return names

    def _real_joint_value(self, name: str) -> float | None:
        if name in self.real_joint_positions:
            return float(self.real_joint_positions[name])
        alias = f"aubo_{name}" if not name.startswith("aubo_") else name.removeprefix("aubo_")
        if alias in self.real_joint_positions:
            return float(self.real_joint_positions[alias])
        return None

    def _real_synced_start_values(self) -> list[float] | None:
        raw = str(self.args.real_start_joints).strip()
        if not raw:
            return None
        try:
            return _float_values(raw, 6, "--real-start-joints")
        except ValueError as exc:
            self.get_logger().error(f"invalid synchronized real start joints: {exc}")
            return None

    def _real_start_values(self) -> tuple[list[float] | None, str]:
        names = self._real_arm_joint_names()
        timeout = max(float(self.args.real_start_state_timeout), 0.0)
        deadline = time.monotonic() + timeout
        values: list[float | None] = []
        while rclpy.ok() and not self.stopping and time.monotonic() <= deadline:
            values = [self._real_joint_value(name) for name in names]
            if all(value is not None for value in values):
                break
            time.sleep(0.05)
        if values and all(value is not None for value in values):
            return [float(value) for value in values], str(self.args.real_joint_states_topic)

        synced = self._real_synced_start_values()
        if synced is not None:
            missing = [name for name, value in zip(names, values) if value is None] if values else names
            self.get_logger().warn(
                "real joint states incomplete; using synchronized start pose from "
                f"ARACHNE_GRASP_ARM_JOINTS. missing_on_{self.args.real_joint_states_topic}={missing}"
            )
            return synced, "ARACHNE_GRASP_ARM_JOINTS"
        missing = [name for name, value in zip(names, values) if value is None] if values else names
        self.get_logger().error(
            "real execution blocked: missing real joint states "
            f"on {self.args.real_joint_states_topic}: {missing}"
        )
        return None, str(self.args.real_joint_states_topic)

    def _real_start_state_matches(self, frame: JointTrajectoryFrame) -> bool:
        actual_values, source = self._real_start_values()
        if actual_values is None:
            return False
        planned = np.asarray(frame.positions, dtype=float)
        actual = np.asarray(actual_values, dtype=float)
        deltas = np.abs([self._wrap_angle(float(p - a)) for p, a in zip(planned, actual)])
        max_delta = float(np.max(deltas))
        tolerance = max(float(self.args.real_start_tolerance_rad), 0.0)
        if max_delta > tolerance:
            names = self._real_arm_joint_names()
            detail = ", ".join(
                f"{name}={delta:.3f}" for name, delta in zip(names, deltas)
            )
            self.get_logger().error(
                "real execution blocked: real arm is not at planned start "
                f"source={source} max_delta={max_delta:.3f}rad "
                f"tolerance={tolerance:.3f}rad deltas=[{detail}]"
            )
            return False
        self.get_logger().warn(
            f"real execution start-state check passed via {source}: max_delta={max_delta:.3f}rad"
        )
        return True

    def _real_joint_trajectory_msg(
        self, frames: list[JointTrajectoryFrame]
    ) -> JointTrajectory:
        stride = max(int(self.args.real_trajectory_point_stride), 1)
        selected = list(frames[::stride])
        if selected[-1] is not frames[-1]:
            selected.append(frames[-1])

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self._real_arm_joint_names()
        for frame in selected:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in frame.positions]
            point.velocities = [float(value) for value in frame.velocities]
            point.accelerations = [float(value) for value in frame.accelerations]
            point.time_from_start.sec = int(frame.time_from_start)
            point.time_from_start.nanosec = int((frame.time_from_start % 1.0) * 1e9)
            trajectory.points.append(point)
        return trajectory

    def _rebased_trajectory_frames(
        self, frames: list[JointTrajectoryFrame]
    ) -> list[JointTrajectoryFrame]:
        if not frames:
            return []
        start_time = float(frames[0].time_from_start)
        rebased: list[JointTrajectoryFrame] = []
        for index, frame in enumerate(frames):
            time_from_start = max(float(frame.time_from_start) - start_time, 0.0)
            velocities = tuple(float(value) for value in frame.velocities)
            accelerations = tuple(float(value) for value in frame.accelerations)
            if index == 0 or index == len(frames) - 1:
                velocities = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                accelerations = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            rebased.append(
                JointTrajectoryFrame(
                    time_from_start,
                    tuple(float(value) for value in frame.positions),  # type: ignore[arg-type]
                    velocities,  # type: ignore[arg-type]
                    accelerations,  # type: ignore[arg-type]
                )
            )
        return rebased

    def _publish_real_gripper(self, command: str) -> None:
        if self.real_gripper_pub is None:
            return
        msg = String()
        msg.data = command
        self.real_gripper_pub.publish(msg)
        self.get_logger().warn(f"REAL gripper command: {command}")

    def _draw_locked_detection(self, image: np.ndarray, preview: GraspPreview) -> None:
        self._draw_detection_overlay(image, preview.detection, "LOCKED")
        cv2.putText(
            image,
            f"YOLO paused; yolo_calls={self.inference_count}; publish restart_search to search again",
            (12, max(image.shape[0] - 18, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (0, 190, 255),
            2,
            cv2.LINE_AA,
        )

    def _publish_locked_annotation(self, color: np.ndarray, header: Header) -> None:
        now = time.monotonic()
        self.locked_visual_last_publish = now
        annotated = color.copy()
        self._draw_locked_detection(annotated, self.last_preview)
        self._publish_annotated(annotated, self.last_preview, header)

    def _predict(self, frame: np.ndarray):
        self.inference_count += 1
        kwargs = {
            "imgsz": int(self.args.imgsz),
            "conf": float(self.args.conf),
            "device": self._yolo_device(),
            "verbose": False,
        }
        if self.class_ids is not None:
            kwargs["classes"] = self.class_ids
        return self.model.predict(frame, **kwargs)[0]

    def _yolo_device(self) -> str:
        if self.model_path.suffix.lower() == ".onnx":
            device = str(self.args.onnx_device).strip()
            if device:
                return device
        return str(self.args.device_id)

    def _best_detection(self, result) -> Detection | None:
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None
        best_idx = int(np.argmax(boxes.conf.cpu().numpy()))
        xyxy = tuple(float(v) for v in boxes.xyxy[best_idx].cpu().numpy())
        class_id = int(boxes.cls[best_idx].item())
        confidence = float(boxes.conf[best_idx].item())
        label = str(result.names.get(class_id, class_id))
        mask_xy: tuple[tuple[float, float], ...] | None = None
        mask_area_px = 0.0
        masks = getattr(result, "masks", None)
        polygons = getattr(masks, "xy", None) if masks is not None else None
        if polygons is not None and len(polygons) > best_idx:
            points = np.asarray(polygons[best_idx], dtype=np.float32)
            if points.ndim == 2 and points.shape[0] >= 3 and points.shape[1] >= 2:
                points = points[:, :2]
                mask_xy = tuple((float(x), float(y)) for x, y in points)
                mask_area_px = float(abs(cv2.contourArea(points)))
        return Detection(
            label=label,
            class_id=class_id,
            confidence=confidence,
            xyxy=xyxy,
            mask_xy=mask_xy,
            mask_area_px=mask_area_px,
        )

    def _needs_depth_snapshot(
        self, detection: Detection, image_shape: tuple[int, ...]
    ) -> tuple[bool, str]:
        if self.last_preview is None:
            return True, "new-target"
        previous = self.snapshot_detection or self.last_preview.detection
        if detection.class_id != previous.class_id:
            return True, "class-change"
        iou = self._bbox_iou(previous.xyxy, detection.xyxy)
        shift = self._normalized_center_shift(previous.xyxy, detection.xyxy, image_shape)
        if iou < float(self.args.snapshot_iou_threshold):
            return True, f"bbox-iou={iou:.2f}"
        if shift > float(self.args.snapshot_center_shift):
            return True, f"bbox-shift={shift:.2f}"
        return False, "cached"

    def _bbox_iou(
        self, a: tuple[float, float, float, float], b: tuple[float, float, float, float]
    ) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        inter = max(ix2 - ix1, 0.0) * max(iy2 - iy1, 0.0)
        area_a = max(ax2 - ax1, 0.0) * max(ay2 - ay1, 0.0)
        area_b = max(bx2 - bx1, 0.0) * max(by2 - by1, 0.0)
        union = area_a + area_b - inter
        return float(inter / union) if union > 1e-6 else 0.0

    def _normalized_center_shift(
        self,
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
        image_shape: tuple[int, ...],
    ) -> float:
        height = max(float(image_shape[0]), 1.0)
        width = max(float(image_shape[1]), 1.0)
        ax = 0.5 * (a[0] + a[2])
        ay = 0.5 * (a[1] + a[3])
        bx = 0.5 * (b[0] + b[2])
        by = 0.5 * (b[1] + b[3])
        diag = max((width * width + height * height) ** 0.5, 1.0)
        return float(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5 / diag)

    def _make_preview(
        self, detection: Detection, depth: np.ndarray, snapshot_reason: str
    ) -> GraspPreview | None:
        info = self.depth_info
        if info is None:
            return None
        dh, dw = depth.shape[:2]
        cw = max(int(self.latest_color.width if self.latest_color is not None else dw), 1)
        ch = max(int(self.latest_color.height if self.latest_color is not None else dh), 1)
        x1, y1, x2, y2 = detection.xyxy
        sx = dw / float(cw)
        sy = dh / float(ch)
        x1d, x2d = x1 * sx, x2 * sx
        y1d, y2d = y1 * sy, y2 * sy
        mask_depth = self._detection_depth_mask(detection, dw, dh, cw, ch)

        cx = 0.5 * (x1d + x2d)
        cy = 0.5 * (y1d + y2d)
        shrink = min(max(float(self.args.roi_shrink), 0.15), 1.0)
        half_w = max(2.0, 0.5 * (x2d - x1d) * shrink)
        half_h = max(2.0, 0.5 * (y2d - y1d) * shrink)
        ix1 = int(np.clip(cx - half_w, 0, dw - 1))
        ix2 = int(np.clip(cx + half_w, ix1 + 1, dw))
        iy1 = int(np.clip(cy - half_h, 0, dh - 1))
        iy2 = int(np.clip(cy + half_h, iy1 + 1, dh))
        roi = depth[iy1:iy2, ix1:ix2].astype(np.float32) * float(self.args.depth_scale)
        valid_mask = (roi >= self.args.min_depth) & (roi <= self.args.max_depth)
        mask_roi = None
        if mask_depth is not None:
            candidate = mask_depth[iy1:iy2, ix1:ix2]
            if int(np.count_nonzero(candidate)) >= 8:
                mask_roi = candidate
                valid_mask &= mask_roi
        roi_source = "mask" if mask_roi is not None else "bbox"
        valid = roi[valid_mask]
        if valid.size < 8:
            return None

        depth_m = float(np.percentile(valid, float(self.args.depth_percentile)))
        roi_points = self._roi_points(
            depth, ix1, iy1, ix2, iy2, depth_m, mask_depth if mask_roi is not None else None
        )
        visual_axis_camera, visual_axis_confidence = self._detection_visual_axis_camera(
            detection, depth_m, dw, dh, cw, ch
        )
        if roi_points:
            arr = np.asarray(roi_points, dtype=np.float32)
            grasp_x, grasp_y, grasp_z = np.median(arr, axis=0).astype(float)
        else:
            grasp_x, grasp_y, grasp_z = self._pixel_to_xyz(cx, cy, depth_m)

        grasp_z = max(grasp_z - float(self.args.grasp_standoff), self.args.min_depth)
        grasp = (grasp_x, grasp_y, grasp_z)
        pregrasp = (
            grasp_x,
            grasp_y,
            max(grasp_z - float(self.args.approach_distance), self.args.min_depth * 0.5),
        )
        lift = (grasp_x, grasp_y - float(self.args.lift_distance), grasp_z)
        retreat = (pregrasp[0], pregrasp[1] - float(self.args.lift_distance), pregrasp[2])
        bbox_points = [
            self._pixel_to_xyz(x1d, y1d, depth_m),
            self._pixel_to_xyz(x2d, y1d, depth_m),
            self._pixel_to_xyz(x2d, y2d, depth_m),
            self._pixel_to_xyz(x1d, y2d, depth_m),
        ]
        (
            _base_path,
            base_segments,
            base_waypoints,
            base_grasp,
            pointcloud_shape,
        ) = self._make_base_path(
            [pregrasp, grasp, lift, retreat],
            roi_points,
            visual_axis_camera,
            visual_axis_confidence,
        )
        return GraspPreview(
            detection=detection,
            depth_m=depth_m,
            grasp_xyz=grasp,
            pregrasp_xyz=pregrasp,
            lift_xyz=lift,
            retreat_xyz=retreat,
            roi_points=roi_points,
            roi_source=roi_source,
            bbox_points=bbox_points,
            base_path_xyz=_base_path if str(self.args.planner_backend) == "none" else [],
            base_trajectory_segments=base_segments,
            base_waypoints=base_waypoints,
            arm_trajectory_frames=[],
            basket_safe=True,
            base_grasp_xyz=base_grasp,
            pointcloud_shape=pointcloud_shape,
            snapshot_reason=snapshot_reason,
            ik_message="planning pending",
        )

    def _detection_depth_mask(
        self, detection: Detection, depth_w: int, depth_h: int, color_w: int, color_h: int
    ) -> np.ndarray | None:
        if not detection.mask_xy:
            return None
        points = np.asarray(detection.mask_xy, dtype=np.float32)
        if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] < 2:
            return None
        points = points[:, :2].copy()
        points[:, 0] *= float(depth_w) / max(float(color_w), 1.0)
        points[:, 1] *= float(depth_h) / max(float(color_h), 1.0)
        points[:, 0] = np.clip(points[:, 0], 0, max(depth_w - 1, 0))
        points[:, 1] = np.clip(points[:, 1], 0, max(depth_h - 1, 0))
        mask = np.zeros((depth_h, depth_w), dtype=np.uint8)
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
        return mask.astype(bool)

    def _roi_points(
        self,
        depth: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        depth_m: float,
        mask: np.ndarray | None = None,
    ) -> list[tuple[float, float, float]]:
        step = max(int(self.args.roi_decimation), 1)
        points: list[tuple[float, float, float]] = []
        band = max(float(self.args.depth_band), 0.01)
        for y in range(y1, y2, step):
            for x in range(x1, x2, step):
                if mask is not None and not bool(mask[y, x]):
                    continue
                z = float(depth[y, x]) * float(self.args.depth_scale)
                if z < self.args.min_depth or z > self.args.max_depth:
                    continue
                if abs(z - depth_m) > band:
                    continue
                points.append(self._pixel_to_xyz(float(x), float(y), z))
        return points

    def _detection_visual_axis_camera(
        self,
        detection: Detection,
        depth_m: float,
        depth_w: int,
        depth_h: int,
        color_w: int,
        color_h: int,
    ) -> tuple[np.ndarray | None, float]:
        if detection.mask_xy and len(detection.mask_xy) >= 6:
            pixels = np.asarray(detection.mask_xy, dtype=np.float64)
            pixels[:, 0] *= float(depth_w) / max(float(color_w), 1.0)
            pixels[:, 1] *= float(depth_h) / max(float(color_h), 1.0)
        else:
            x1, y1, x2, y2 = detection.xyxy
            sx = float(depth_w) / max(float(color_w), 1.0)
            sy = float(depth_h) / max(float(color_h), 1.0)
            pixels = np.asarray(
                [
                    (x1 * sx, y1 * sy),
                    (x2 * sx, y1 * sy),
                    (x2 * sx, y2 * sy),
                    (x1 * sx, y2 * sy),
                ],
                dtype=np.float64,
            )
        if pixels.ndim != 2 or pixels.shape[0] < 3:
            return None, 0.0
        center = np.mean(pixels[:, :2], axis=0)
        centered = pixels[:, :2] - center
        cov = np.cov(centered.T)
        if not np.all(np.isfinite(cov)):
            return None, 0.0
        eig_values, eig_vectors = np.linalg.eigh(cov)
        order = np.argsort(eig_values)[::-1]
        eig_values = eig_values[order]
        eig_vectors = eig_vectors[:, order]
        if float(eig_values[0]) <= 1e-6:
            return None, 0.0
        axis_px = eig_vectors[:, 0]
        axis_px /= max(float(np.linalg.norm(axis_px)), 1e-9)
        if axis_px[0] < -1e-9 or (abs(axis_px[0]) < 1e-9 and axis_px[1] < 0.0):
            axis_px *= -1.0
        spread = np.sqrt(max(float(eig_values[0]), 1.0))
        delta_px = float(np.clip(spread * 0.35, 6.0, 28.0))
        p0 = center
        p1 = center + axis_px * delta_px
        xyz0 = np.asarray(self._pixel_to_xyz(float(p0[0]), float(p0[1]), depth_m), dtype=float)
        xyz1 = np.asarray(self._pixel_to_xyz(float(p1[0]), float(p1[1]), depth_m), dtype=float)
        axis_camera = xyz1 - xyz0
        norm = float(np.linalg.norm(axis_camera))
        if norm < 1e-6:
            return None, 0.0
        anisotropy = float((eig_values[0] - eig_values[1]) / max(eig_values[0] + eig_values[1], 1e-9))
        return axis_camera / norm, float(np.clip(anisotropy, 0.0, 1.0))

    def _pixel_to_xyz(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        info = self.depth_info
        if info is None:
            return (0.0, 0.0, 0.0)
        fx = float(info.k[0]) if info.k[0] else float(info.width) * 0.9
        fy = float(info.k[4]) if info.k[4] else fx
        cx = float(info.k[2]) if info.k[2] else (float(info.width) - 1.0) * 0.5
        cy = float(info.k[5]) if info.k[5] else (float(info.height) - 1.0) * 0.5
        pixel_x = cx - float(u) if bool(self.args.depth_projection_flip_x) else float(u) - cx
        pixel_y = cy - float(v) if bool(self.args.depth_projection_flip_y) else float(v) - cy
        x = pixel_x * float(z) / fx
        y = pixel_y * float(z) / fy
        return (float(x), float(y), float(z))

    def _publish_preview(self, preview: GraspPreview, source_header: Header) -> None:
        header = _header(source_header.stamp, source_header.frame_id)
        if preview.base_path_xyz:
            preview.basket_safe = self._basket_path_safe(preview.base_path_xyz)
        self.markers_pub.publish(self._markers(preview, header))
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, preview.roi_points))
        self.path_pub.publish(self._path(preview, header))
        self._publish_preview_arm_state(preview)
        now = time.monotonic()
        if now - self.last_log_time > 0.75:
            self.last_log_time = now
            gx, gy, gz = preview.grasp_xyz
            shape_msg = "shape=none"
            if preview.pointcloud_shape is not None:
                shape = preview.pointcloud_shape
                shape_msg = (
                    f"shape=pc points={shape.point_count} conf={shape.axis_confidence:.2f} "
                    f"visual_conf={shape.visual_axis_confidence:.2f} "
                    f"extent=({shape.extent_major_m:.3f},{shape.extent_minor_m:.3f},{shape.extent_z_m:.3f}) "
                    f"z_bias={shape.z_bias_m:.3f}"
                )
            self.get_logger().info(
                f"{preview.detection.label} conf={preview.detection.confidence:.2f} "
                f"depth={preview.depth_m:.3f}m grasp_camera=({gx:.3f},{gy:.3f},{gz:.3f}) "
                f"base_offset=({self.grasp_base_offset[0]:.3f},{self.grasp_base_offset[1]:.3f},{self.grasp_base_offset[2]:.3f}) "
                f"roi={preview.roi_source} roi_points={len(preview.roi_points)} snapshot={preview.snapshot_reason} "
                f"{shape_msg} "
                f"planned_path_points={len(preview.base_path_xyz)} "
                f"stream_rate={max(float(self.args.playback_rate), 1.0):.1f}Hz "
                f"trajectory={preview.ik_message} "
                f"yolo_calls={self.inference_count} basket_safe={preview.basket_safe}"
            )

    def _playback_tick(self) -> None:
        if not self.inference_paused or self.last_preview is None:
            return
        if not self.last_preview.arm_trajectory_frames:
            return
        self._publish_preview_arm_state(self.last_preview)
        self.markers_pub.publish(MarkerArray(markers=self._basket_markers(self.last_preview)))

    def _publish_preview_arm_state(self, preview: GraspPreview) -> None:
        if not preview.arm_trajectory_frames:
            return
        frame = self._current_trajectory_frame(preview)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        gripper_names, gripper_positions = self._gripper_preview_positions(preview, frame)
        msg.name = list(ARM_JOINT_NAMES) + gripper_names
        msg.position = [float(value) for value in frame.positions] + gripper_positions
        msg.velocity = [float(value) for value in frame.velocities] + [0.0] * len(gripper_names)
        self.arm_preview_pub.publish(msg)

    def _gripper_preview_positions(
        self, preview: GraspPreview, frame: JointTrajectoryFrame
    ) -> tuple[list[str], list[float]]:
        profile = GRIPPER_PREVIEW_PROFILES.get(str(self.args.gripper_type))
        if not profile or not preview.arm_trajectory_frames:
            return [], []
        duration = max(float(preview.arm_trajectory_frames[-1].time_from_start), 1e-6)
        progress = min(max(float(frame.time_from_start) / duration, 0.0), 1.0)
        close_start, close_end, open_start, open_end, _close_label, _open_label = (
            self._gripper_event_progresses(preview)
        )

        if progress < close_start:
            amount = 0.0
        elif progress < open_start:
            amount = 1.0
        else:
            amount = 0.0

        open_positions = np.asarray(profile["open"], dtype=float)
        closed_positions = np.asarray(profile["closed"], dtype=float)
        positions = open_positions * (1.0 - amount) + closed_positions * amount
        return list(profile["names"]), [float(value) for value in positions]

    def _gripper_event_progresses(
        self, preview: GraspPreview
    ) -> tuple[float, float, float, float, str, str]:
        progress_by_name = {
            name: float(progress) for name, _point, progress in self._planning_target_samples(preview)
        }
        close_waypoint = str(self.args.gripper_close_waypoint).strip()
        open_waypoint = str(self.args.gripper_open_waypoint).strip()
        close_start = progress_by_name.get(
            close_waypoint,
            min(max(float(self.args.gripper_close_progress_start), 0.0), 1.0),
        )
        open_start = progress_by_name.get(
            open_waypoint,
            min(max(float(self.args.gripper_open_progress_start), 0.0), 1.0),
        )
        close_start = min(max(float(close_start), 0.0), 1.0)
        open_start = min(max(float(open_start), close_start), 1.0)
        transition = min(max(float(self.args.gripper_transition_progress), 1e-6), 0.25)
        close_end = min(max(close_start + transition, close_start + 1e-6), open_start)
        if open_start <= close_end:
            open_start = min(max(close_end, open_start), 1.0)
        open_end = min(max(open_start + transition, open_start + 1e-6), 1.0)
        close_label = f"{close_waypoint or 'grasp'}:close"
        open_label = f"{open_waypoint or 'basket_over'}:open"
        return close_start, close_end, open_start, open_end, close_label, open_label

    def _current_trajectory_frame(self, preview: GraspPreview) -> JointTrajectoryFrame:
        frames = preview.arm_trajectory_frames
        if not frames:
            q = tuple(float(value) for value in self.current_arm_joints)
            zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return JointTrajectoryFrame(0.0, q, zeros, zeros)  # type: ignore[arg-type]
        duration = max(float(frames[-1].time_from_start), 1e-6)
        period_override = float(self.args.playback_period)
        period = max(period_override, duration) if period_override > 0.0 else duration
        start_time = self.plan_lock_time or self.snapshot_time or time.monotonic()
        elapsed = max(time.monotonic() - start_time, 0.0)
        if bool(self.args.loop_playback):
            t = elapsed % max(period, 1e-6)
        else:
            t = min(elapsed, duration)
        frame = self._interpolated_trajectory_frame(frames, t)
        return frame

    def _interpolated_trajectory_frame(
        self, frames: list[JointTrajectoryFrame], t: float
    ) -> JointTrajectoryFrame:
        if len(frames) == 1 or t <= frames[0].time_from_start:
            return frames[0]
        for index, end in enumerate(frames[1:], start=1):
            start = frames[index - 1]
            if end.time_from_start >= t:
                span = max(end.time_from_start - start.time_from_start, 1e-9)
                local = (t - start.time_from_start) / span
                q0 = np.asarray(start.positions, dtype=float)
                q1 = np.asarray(end.positions, dtype=float)
                v0 = np.asarray(start.velocities, dtype=float)
                v1 = np.asarray(end.velocities, dtype=float)
                a0 = np.asarray(start.accelerations, dtype=float)
                a1 = np.asarray(end.accelerations, dtype=float)
                q = q0 * (1.0 - local) + q1 * local
                v = v0 * (1.0 - local) + v1 * local
                a = a0 * (1.0 - local) + a1 * local
                return JointTrajectoryFrame(
                    float(t),
                    tuple(float(value) for value in q),  # type: ignore[arg-type]
                    tuple(float(value) for value in v),  # type: ignore[arg-type]
                    tuple(float(value) for value in a),  # type: ignore[arg-type]
                )
        return frames[-1]

    def _arm_trajectory_grasp_path_base(
        self, frames: list[JointTrajectoryFrame]
    ) -> list[tuple[float, float, float]]:
        if not frames:
            return []
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return []
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._throttled_log(f"waiting for TF {self.base_frame} <- {self.aubo_base_frame}: {exc}")
            return []
        base_from_aubo = self._matrix_from_transform(transform)
        max_points = 500
        stride = max(int(math.ceil(len(frames) / max_points)), 1)
        selected = list(frames[::stride])
        if selected[-1] is not frames[-1]:
            selected.append(frames[-1])

        path: list[tuple[float, float, float]] = []
        for frame in selected:
            tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
            grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
            path.append((float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3])))
        return path

    def _current_grasp_position_base(self, preview: GraspPreview) -> tuple[float, float, float] | None:
        if not preview.arm_trajectory_frames:
            return None
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return None
        frame = self._current_trajectory_frame(preview)
        base_from_aubo = self._matrix_from_transform(transform)
        tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
        grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
        return (float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3]))

    def _current_grasp_origin_base(self) -> tuple[float, float, float] | None:
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        base_from_aubo = self._matrix_from_transform(transform)
        tool_in_aubo = self.kinematics.fk(np.asarray(self.current_arm_joints, dtype=float))
        grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
        return (float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3]))

    def _tool_to_grasp_matrix(self) -> tuple[np.ndarray | None, str]:
        if self.tool_to_grasp_matrix is not None:
            return self.tool_to_grasp_matrix, "cached"
        tcp_offset = float(self.args.grasp_tcp_offset_m)
        if tcp_offset > 0.0:
            self.tool_to_grasp_matrix = np.eye(4, dtype=np.float64)
            self.tool_to_grasp_matrix[2, 3] = tcp_offset
            self.tool_to_grasp_frames = ("tool0", f"virtual_grasp_tcp_z_{tcp_offset:.3f}m")
            return self.tool_to_grasp_matrix, self.tool_to_grasp_frames[1]
        last_error = ""
        for tool_frame in self._frame_candidates("tool0"):
            for grasp_frame in self._frame_candidates(END_EFFECTOR_FRAME):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        tool_frame,
                        grasp_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.02),
                    )
                except TransformException as exc:
                    last_error = str(exc)
                    continue
                self.tool_to_grasp_matrix = self._matrix_from_transform(transform)
                self.tool_to_grasp_frames = (tool_frame, grasp_frame)
                return self.tool_to_grasp_matrix, f"{tool_frame}<-{grasp_frame}"
        return None, f"waiting for TF tool0 <- {END_EFFECTOR_FRAME}: {last_error}"

    def _tool0_target_from_grasp_target(
        self,
        grasp_position_base: tuple[float, float, float],
        grasp_rotation_base: np.ndarray,
    ) -> tuple[tuple[float, float, float], np.ndarray] | None:
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        grasp_in_base = np.eye(4, dtype=np.float64)
        grasp_in_base[:3, :3] = np.asarray(grasp_rotation_base, dtype=np.float64)
        grasp_in_base[:3, 3] = np.asarray(grasp_position_base, dtype=np.float64)
        tool_in_base = grasp_in_base @ self._invert_rigid(tool_to_grasp)
        return (
            (float(tool_in_base[0, 3]), float(tool_in_base[1, 3]), float(tool_in_base[2, 3])),
            self._orthonormalize_rotation(tool_in_base[:3, :3]),
        )

    def _frame_candidates(self, frame_id: str) -> list[str]:
        prefix = self._display_frame_prefix()
        candidates = []
        if prefix and not frame_id.startswith(prefix):
            candidates.append(f"{prefix}{frame_id}")
        candidates.append(frame_id)
        return list(dict.fromkeys(candidates))

    def _display_frame_prefix(self) -> str:
        suffix = "aubo_base_link"
        if self.aubo_base_frame.endswith(suffix):
            return self.aubo_base_frame[: -len(suffix)]
        return ""

    def _joint_delta(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        raw = np.asarray(target, dtype=float) - np.asarray(current, dtype=float)
        return np.arctan2(np.sin(raw), np.cos(raw))

    def _wrap_joints(self, q: np.ndarray) -> np.ndarray:
        return np.arctan2(np.sin(q), np.cos(q))

    def _make_constrained_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        if str(self.args.planner_backend) == "none":
            return [], "perception_only: planning skipped"
        if str(self.args.planner_backend) == "moveit":
            return self._make_moveit_arm_trajectory(preview)
        return self._make_local_arm_trajectory(preview)

    def _make_moveit_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        targets = self._planning_target_samples(preview)
        if not targets:
            return [], "moveit: no planning key waypoints"
        if self.moveit_plan_client is None:
            return [], "moveit: planner client not configured"
        if not self.moveit_plan_client.service_is_ready():
            self.moveit_plan_client.wait_for_service(timeout_sec=0.05)
        if not self.moveit_plan_client.service_is_ready():
            return [], f"moveit: waiting for {self.args.moveit_plan_service}"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return [], f"moveit: waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"
        aubo_from_base = self._matrix_from_transform(transform)

        q_current = np.asarray(self.current_arm_joints, dtype=float)
        q_waypoints: list[np.ndarray] = [np.asarray(q_current, dtype=float)]
        planner_ids = [
            token.strip()
            for token in str(self.args.moveit_planners).split(",")
            if token.strip()
        ] or ["RRTConnectkConfigDefault"]
        planner_used: list[str] = []
        reached_targets: list[str] = []
        total_points = 0
        soft_waypoints = 0
        local_fallbacks = 0
        local_firsts = 0
        fallback_notes: list[str] = []
        current_rotation_base = self._current_end_effector_rotation_base()
        current_tool0_rotation_base = self._current_tool_rotation_base()
        base_from_aubo = self._invert_rigid(aubo_from_base)

        for index, (target_name, target_base, progress) in enumerate(targets):
            is_soft_waypoint = target_name != "grasp"
            target_rotations_base = self._target_orientation_candidates_base(
                target_base,
                progress,
                current_rotation_base,
                preview.pointcloud_shape,
                target_name,
            )
            response = None
            response_planner = ""
            failure_messages: list[str] = []
            fallback_candidate = None
            local_first_accepted = False
            for planner_id in planner_ids:
                for orientation_index, target_rotation_base in enumerate(target_rotations_base):
                    tool_target = self._tool0_target_from_grasp_target(
                        target_base, target_rotation_base
                    )
                    if tool_target is None:
                        _matrix, message = self._tool_to_grasp_matrix()
                        return [], f"moveit: {message}"
                    target_tool0_base, target_tool0_rotation_base = tool_target
                    tool_ground_ok, tool_ground_clearance, tool_ground_hit = (
                        self._tool_target_ground_clearance_base(target_tool0_base, target_base)
                    )
                    if not tool_ground_ok:
                        tx, ty, tz = target_tool0_base
                        failure_messages.append(
                            f"ori{orientation_index}/{target_name}: tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                            f"{tool_ground_hit} clearance={tool_ground_clearance:.3f}m"
                        )
                        continue
                    unreachable_note = self._moveit_unreachable_note(target_tool0_base)
                    if unreachable_note:
                        x, y, z = target_base
                        tx, ty, tz = target_tool0_base
                        failure_messages.append(
                            f"ori{orientation_index}/{target_name}: grasp=({x:.3f},{y:.3f},{z:.3f}) "
                            f"tool0=({tx:.3f},{ty:.3f},{tz:.3f}) {unreachable_note}"
                        )
                        continue
                    ik_ok, q_goal, position_error, orientation_error = self._solve_tool0_goal_joints(
                        q_current,
                        aubo_from_base,
                        target_tool0_base,
                        target_tool0_rotation_base,
                    )
                    if not ik_ok and is_soft_waypoint:
                        ik_ok = (
                            position_error
                            <= max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0)
                            and orientation_error
                            <= max(float(self.args.moveit_soft_waypoint_orientation_tolerance), 0.01)
                        )
                        if ik_ok:
                            soft_waypoints += 1
                    q_unwrapped_goal = np.asarray(q_current, dtype=float) + self._joint_delta(
                        q_goal, q_current
                    )
                    endpoint_safe, endpoint_clearance, endpoint_hit_name = self._arm_collision_clearance_base(
                        q_unwrapped_goal, base_from_aubo
                    )
                    segment_safe, segment_clearance, segment_hit_name = (
                        self._joint_segment_collision_clearance_base(
                            q_current, q_unwrapped_goal, base_from_aubo
                        )
                    )
                    collision_free = bool(endpoint_safe and segment_safe)
                    clearance = min(float(endpoint_clearance), float(segment_clearance))
                    hit_name = endpoint_hit_name if not endpoint_safe else segment_hit_name
                    (
                        fallback_position_tolerance,
                        fallback_orientation_tolerance,
                    ) = self._moveit_fallback_tolerances(target_name, is_soft_waypoint)
                    fallback_ok = (
                        position_error <= fallback_position_tolerance
                        and orientation_error <= fallback_orientation_tolerance
                        and (collision_free or bool(self.args.allow_colliding_best_effort))
                    )
                    if fallback_ok:
                        fallback_score = (
                            100.0 * float(position_error)
                            + 5.0 * float(orientation_error)
                            + 0.25 * float(np.linalg.norm(q_unwrapped_goal - q_current))
                            + (0.0 if collision_free else COLLISION_PENALTY)
                        )
                        candidate = (
                            fallback_score,
                            q_unwrapped_goal,
                            float(position_error),
                            float(orientation_error),
                            collision_free,
                            hit_name,
                            orientation_index,
                            target_rotation_base,
                            target_tool0_rotation_base,
                        )
                        if fallback_candidate is None or fallback_score < fallback_candidate[0]:
                            fallback_candidate = candidate
                    local_first_ok = (
                        bool(self.args.moveit_local_first)
                        and position_error
                        <= max(float(self.args.moveit_position_tolerance) * 2.0, 0.01)
                        and orientation_error
                        <= max(float(self.args.moveit_ik_orientation_tolerance), 0.01)
                        and collision_free
                    )
                    if local_first_ok:
                        if np.max(np.abs(q_unwrapped_goal - q_waypoints[-1])) > 1e-5:
                            q_waypoints.append(np.asarray(q_unwrapped_goal, dtype=float))
                        q_current = np.asarray(q_waypoints[-1], dtype=float)
                        reached_targets.append(target_name)
                        planner_used.append(f"local_first_ori{orientation_index}")
                        local_firsts += 1
                        fallback_notes.append(
                            f"{target_name}/ori{orientation_index}:"
                            f"pos_err={position_error:.3f}m,"
                            f"ori_err={orientation_error:.3f}rad"
                        )
                        current_rotation_base = self._orthonormalize_rotation(
                            np.asarray(target_rotation_base, dtype=float)
                        )
                        current_tool0_rotation_base = self._orthonormalize_rotation(
                            np.asarray(target_tool0_rotation_base, dtype=float)
                        )
                        self._throttled_log(
                            f"MoveIt local-first accepted at {target_name}; "
                            f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad "
                            f"clearance={clearance:.3f}m"
                        )
                        local_first_accepted = True
                        break
                    if not ik_ok:
                        if not bool(self.args.moveit_pose_goal_on_ik_failure):
                            tx, ty, tz = target_tool0_base
                            failure_messages.append(
                                f"ik/ori{orientation_index} tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                                f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad"
                            )
                            continue
                        attempt = self._call_moveit_pose_plan(
                            q_current,
                            target_tool0_base,
                            target_tool0_rotation_base,
                            planner_id,
                            max(float(self.args.moveit_ik_orientation_tolerance), 0.01),
                            current_tool0_rotation_base,
                        )
                    else:
                        attempt = self._call_moveit_plan(
                            q_current,
                            q_goal,
                            current_tool0_rotation_base,
                            planner_id,
                        )
                    if attempt.response is not None:
                        path_safe, path_message = self._moveit_response_ground_safe(
                            attempt.response, q_current, base_from_aubo
                        )
                        if not path_safe:
                            failure_messages.append(
                                f"{planner_id}/ori{orientation_index}: {path_message}"
                            )
                            continue
                        response = attempt.response
                        response_planner = planner_id
                        break
                    tx, ty, tz = target_tool0_base
                    failure_messages.append(
                        f"{planner_id}/ori{orientation_index} "
                        f"tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                        f"q_goal_pos_err={position_error:.3f}m q_goal_ori_err={orientation_error:.3f}rad: "
                        f"{attempt.message}"
                    )
                if local_first_accepted:
                    break
                if response is not None:
                    break
            if local_first_accepted:
                continue
            if response is None:
                x, y, z = target_base
                failure_text = (
                    f"moveit: planning failed at {target_name} {index + 1}/{len(targets)} "
                    f"grasp_xyz=({x:.3f},{y:.3f},{z:.3f}); "
                    + " | ".join(failure_messages[-6:])
                )
                if fallback_candidate is not None:
                    (
                        _score,
                        q_fallback,
                        position_error,
                        orientation_error,
                        collision_free,
                        hit_name,
                        orientation_index,
                        fallback_rotation_base,
                        fallback_tool0_rotation_base,
                    ) = fallback_candidate
                    if not collision_free and not bool(self.args.allow_colliding_best_effort):
                        failure_text += f"; local fallback collision at {hit_name or 'vehicle'}"
                    else:
                        if np.max(np.abs(q_fallback - q_waypoints[-1])) > 1e-5:
                            q_waypoints.append(np.asarray(q_fallback, dtype=float))
                        q_current = np.asarray(q_waypoints[-1], dtype=float)
                        reached_targets.append(target_name)
                        planner_used.append(f"local_fallback_ori{orientation_index}")
                        local_fallbacks += 1
                        fallback_notes.append(
                            f"{target_name}/ori{orientation_index}:"
                            f"pos_err={position_error:.3f}m,"
                            f"ori_err={orientation_error:.3f}rad"
                        )
                        current_rotation_base = self._orthonormalize_rotation(
                            np.asarray(fallback_rotation_base, dtype=float)
                        )
                        current_tool0_rotation_base = self._orthonormalize_rotation(
                            np.asarray(fallback_tool0_rotation_base, dtype=float)
                        )
                        self._throttled_log(
                            f"MoveIt blocked at {target_name}; using local fallback "
                            f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad"
                        )
                        continue
                if len(q_waypoints) > 1:
                    frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
                    return (
                        frames,
                        "moveit_ompl_joint_goal_partial "
                        f"partial_until={reached_targets[-1] if reached_targets else 'start'} "
                        f"blocked_at={target_name} "
                        f"planners={'+'.join(planner_used)} raw_points={total_points} "
                        f"targets={','.join(reached_targets)} soft_waypoints={soft_waypoints} "
                        f"local_firsts={local_firsts} local_fallbacks={local_fallbacks} "
                        f"fallbacks={';'.join(fallback_notes) if fallback_notes else 'none'} "
                        f"{limit_message}; {failure_text}",
                    )
                return (
                    [],
                    failure_text,
                )

            joint_trajectory = response.trajectory.joint_trajectory
            if not joint_trajectory.points:
                return [], f"moveit: empty trajectory from {response_planner}"
            name_to_index = {name: i for i, name in enumerate(joint_trajectory.joint_names)}
            for point in joint_trajectory.points[1:]:
                if not all(name in name_to_index for name in ARM_JOINT_NAMES):
                    return [], f"moveit: trajectory missing Aubo joints from {response_planner}"
                q_raw = np.asarray(
                    [point.positions[name_to_index[name]] for name in ARM_JOINT_NAMES],
                    dtype=float,
                )
                q_unwrapped = q_waypoints[-1] + self._joint_delta(q_raw, q_waypoints[-1])
                if np.max(np.abs(q_unwrapped - q_waypoints[-1])) > 1e-5:
                    q_waypoints.append(q_unwrapped)
            q_current = np.asarray(q_waypoints[-1], dtype=float)
            current_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_rotation_base, dtype=float)
            )
            current_tool0_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_tool0_rotation_base, dtype=float)
            )
            planner_used.append(response_planner)
            reached_targets.append(target_name)
            total_points += len(joint_trajectory.points)
            self._throttled_log(
                f"MoveIt segment ready: {target_name} "
                f"points={len(joint_trajectory.points)} planner={response_planner}"
            )

        frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
        return (
            frames,
            "moveit_ompl_joint_goal "
            f"planners={'+'.join(planner_used)} raw_points={total_points} "
            f"targets={','.join(reached_targets)} "
            f"soft_waypoints={soft_waypoints} local_firsts={local_firsts} "
            f"local_fallbacks={local_fallbacks} "
            f"fallbacks={';'.join(fallback_notes) if fallback_notes else 'none'} "
            f"{limit_message}",
        )

    def _tool_target_ground_clearance_base(
        self,
        tool0_base: tuple[float, float, float],
        grasp_base: tuple[float, float, float],
    ) -> tuple[bool, float, str | None]:
        threshold = float(self.args.ground_min_z_base) + max(
            float(self.args.tool_ground_clearance), 0.0
        )
        min_clearance = float("inf")
        hit_name: str | None = None
        a = np.asarray(tool0_base, dtype=float)
        b = np.asarray(grasp_base, dtype=float)
        samples = max(int(self.args.arm_collision_samples_per_link), 2)
        for t in np.linspace(0.0, 1.0, samples):
            point = a * (1.0 - t) + b * t
            clearance = float(point[2] - threshold)
            if clearance < min_clearance:
                min_clearance = clearance
                hit_name = "ground/gripper_tcp"
            if clearance < 0.0:
                return False, clearance, hit_name
        return True, float(min_clearance if np.isfinite(min_clearance) else 0.0), hit_name

    def _moveit_response_ground_safe(
        self,
        response,
        q_start: np.ndarray,
        base_from_aubo: np.ndarray,
    ) -> tuple[bool, str]:
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            return False, "empty trajectory"
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}
        if not all(name in name_to_index for name in ARM_JOINT_NAMES):
            return False, "trajectory missing Aubo joints"
        q_previous = np.asarray(q_start, dtype=float)
        for point_index, point in enumerate(trajectory.points[1:], start=1):
            q_raw = np.asarray(
                [point.positions[name_to_index[name]] for name in ARM_JOINT_NAMES],
                dtype=float,
            )
            q_unwrapped = q_previous + self._joint_delta(q_raw, q_previous)
            safe, clearance, hit_name = self._arm_collision_clearance_base(
                q_unwrapped, base_from_aubo
            )
            if not safe:
                return (
                    False,
                    f"path collision at point {point_index}: {hit_name or 'vehicle'} "
                    f"clearance={clearance:.3f}m",
                )
            q_previous = q_unwrapped
        return True, "ok"

    def _joint_segment_collision_clearance_base(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        base_from_aubo: np.ndarray,
    ) -> tuple[bool, float, str | None]:
        delta = self._joint_delta(np.asarray(q_goal, dtype=float), np.asarray(q_start, dtype=float))
        samples = max(int(self.args.arm_collision_samples_per_link), 3)
        min_clearance = float("inf")
        min_hit_name: str | None = None
        for alpha in np.linspace(0.0, 1.0, samples):
            q_sample = np.asarray(q_start, dtype=float) + float(alpha) * delta
            safe, clearance, hit_name = self._arm_collision_clearance_base(
                q_sample, base_from_aubo
            )
            if clearance < min_clearance:
                min_clearance = float(clearance)
                min_hit_name = hit_name
            if not safe:
                return False, float(clearance), hit_name
        return (
            True,
            float(min_clearance if np.isfinite(min_clearance) else 0.0),
            min_hit_name,
        )

    def _moveit_fallback_tolerances(
        self, target_name: str, is_soft_waypoint: bool
    ) -> tuple[float, float]:
        if is_soft_waypoint:
            return (
                max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0),
                max(float(self.args.moveit_soft_waypoint_orientation_tolerance), 0.01),
            )
        if target_name == "grasp":
            return (
                max(float(self.args.moveit_grasp_fallback_position_tolerance), 0.0),
                max(float(self.args.moveit_grasp_fallback_orientation_tolerance), 0.01),
            )
        return (
            max(float(self.args.moveit_local_fallback_position_tolerance), 0.0),
            max(float(self.args.moveit_local_fallback_orientation_tolerance), 0.01),
        )

    def _planning_target_samples(
        self, preview: GraspPreview
    ) -> list[tuple[str, tuple[float, float, float], float]]:
        waypoint_map = {name: xyz for name, xyz in preview.base_waypoints}
        names = [
            token.strip()
            for token in str(self.args.planning_key_waypoints).split(",")
            if token.strip()
        ]
        selected: list[tuple[str, tuple[float, float, float]]] = []
        for name in names:
            if name in waypoint_map:
                selected.append((name, waypoint_map[name]))
        if not selected and preview.base_trajectory_segments:
            selected = [
                (f"segment_{index}", segment.end)
                for index, segment in enumerate(preview.base_trajectory_segments[:-1])
            ]
        if not selected:
            return []
        if len(selected) == 1:
            return [(selected[0][0], selected[0][1], 1.0)]
        return [
            (name, point, index / float(len(selected) - 1))
            for index, (name, point) in enumerate(selected)
        ]

    def _solve_tool0_goal_joints(
        self,
        q_start: np.ndarray,
        aubo_from_base: np.ndarray,
        target_tool0_base: tuple[float, float, float],
        target_tool0_rotation_base: np.ndarray,
    ) -> tuple[bool, np.ndarray, float, float]:
        target_transform = np.eye(4, dtype=np.float64)
        target_transform[:3, :3] = (
            np.asarray(aubo_from_base[:3, :3], dtype=np.float64)
            @ np.asarray(target_tool0_rotation_base, dtype=np.float64)
        )
        target_transform[:3, 3] = np.asarray(
            self._transform_matrix_point(aubo_from_base, np.asarray(target_tool0_base, dtype=float)),
            dtype=np.float64,
        )
        position_tolerance = max(float(self.args.moveit_position_tolerance), PREVIEW_IK_TOLERANCE_M)
        orientation_tolerance = max(float(self.args.moveit_ik_orientation_tolerance), 0.01)
        ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
            np.asarray(q_start, dtype=float),
            target_transform,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            damping=PREVIEW_IK_DAMPING,
            max_iterations=max(int(self.args.moveit_ik_max_iterations), 20),
            max_step=PREVIEW_IK_MAX_STEP,
            orientation_weight=0.25,
        )
        q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
        return bool(ok), q_goal, float(position_error), float(orientation_error)

    def _call_moveit_pose_plan(
        self,
        q_start: np.ndarray,
        target_tool0_base: tuple[float, float, float],
        target_tool0_rotation_base: np.ndarray,
        planner_id: str,
        orientation_tolerance: float,
        reference_rotation_base: np.ndarray,
    ) -> MoveItPlanAttempt:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "aubo_arm"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = planner_id
        motion_request.num_planning_attempts = max(int(self.args.moveit_planning_attempts), 1)
        motion_request.allowed_planning_time = max(float(self.args.moveit_planning_time), 0.2)
        motion_request.max_velocity_scaling_factor = min(
            max(float(self.args.moveit_velocity_scale), 0.01), 1.0
        )
        motion_request.max_acceleration_scaling_factor = min(
            max(float(self.args.moveit_accel_scale), 0.01), 1.0
        )
        start_names, start_positions = self._moveit_start_state_joint_values(q_start)
        motion_request.start_state.joint_state.name = start_names
        motion_request.start_state.joint_state.position = start_positions
        motion_request.start_state.is_diff = False
        motion_request.workspace_parameters.header.frame_id = self.base_frame
        motion_request.workspace_parameters.min_corner.x = -1.0
        motion_request.workspace_parameters.min_corner.y = -1.0
        motion_request.workspace_parameters.min_corner.z = -0.3
        motion_request.workspace_parameters.max_corner.x = 1.6
        motion_request.workspace_parameters.max_corner.y = 1.0
        motion_request.workspace_parameters.max_corner.z = 1.2
        motion_request.goal_constraints = [
            self._moveit_pose_goal_constraint(
                target_tool0_base,
                target_tool0_rotation_base,
                orientation_tolerance,
            )
        ]
        if bool(self.args.moveit_use_orientation_path_constraint):
            motion_request.path_constraints = self._moveit_orientation_path_constraint(
                reference_rotation_base
            )

        future = self.moveit_plan_client.call_async(request)
        deadline = time.monotonic() + max(
            float(self.args.moveit_planning_time)
            + max(float(self.args.moveit_service_timeout_padding), 0.0),
            0.3,
        )
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return MoveItPlanAttempt(None, "pose goal service timeout")
        result = future.result()
        if result is None:
            return MoveItPlanAttempt(None, "pose goal empty service result")
        response = result.motion_plan_response
        if int(response.error_code.val) != 1:
            return MoveItPlanAttempt(
                None,
                f"pose goal {self._moveit_error_message(int(response.error_code.val))}",
            )
        return MoveItPlanAttempt(response, "pose goal success")

    def _moveit_joint_goal_constraint(self, q_goal: np.ndarray) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_joint_goal"
        tolerance = max(float(self.args.moveit_joint_goal_tolerance), 1e-4)
        for name, position in zip(ARM_JOINT_NAMES, np.asarray(q_goal, dtype=float)):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _call_moveit_plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        reference_rotation_base: np.ndarray,
        planner_id: str,
    ) -> MoveItPlanAttempt:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "aubo_arm"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = planner_id
        motion_request.num_planning_attempts = max(int(self.args.moveit_planning_attempts), 1)
        motion_request.allowed_planning_time = max(float(self.args.moveit_planning_time), 0.2)
        motion_request.max_velocity_scaling_factor = min(
            max(float(self.args.moveit_velocity_scale), 0.01), 1.0
        )
        motion_request.max_acceleration_scaling_factor = min(
            max(float(self.args.moveit_accel_scale), 0.01), 1.0
        )
        start_names, start_positions = self._moveit_start_state_joint_values(q_start)
        motion_request.start_state.joint_state.name = start_names
        motion_request.start_state.joint_state.position = start_positions
        motion_request.start_state.is_diff = False
        motion_request.workspace_parameters.header.frame_id = self.base_frame
        motion_request.workspace_parameters.min_corner.x = -1.0
        motion_request.workspace_parameters.min_corner.y = -1.0
        motion_request.workspace_parameters.min_corner.z = -0.3
        motion_request.workspace_parameters.max_corner.x = 1.6
        motion_request.workspace_parameters.max_corner.y = 1.0
        motion_request.workspace_parameters.max_corner.z = 1.2
        motion_request.goal_constraints = [self._moveit_joint_goal_constraint(q_goal)]
        if bool(self.args.moveit_use_orientation_path_constraint):
            motion_request.path_constraints = self._moveit_orientation_path_constraint(
                reference_rotation_base
            )

        future = self.moveit_plan_client.call_async(request)
        deadline = time.monotonic() + max(
            float(self.args.moveit_planning_time)
            + max(float(self.args.moveit_service_timeout_padding), 0.0),
            0.3,
        )
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return MoveItPlanAttempt(None, "service timeout")
        result = future.result()
        if result is None:
            return MoveItPlanAttempt(None, "empty service result")
        response = result.motion_plan_response
        if int(response.error_code.val) != 1:
            return MoveItPlanAttempt(
                None,
                self._moveit_error_message(int(response.error_code.val)),
            )
        return MoveItPlanAttempt(response, "success")

    def _moveit_pose_goal_constraint(
        self,
        target_base: tuple[float, float, float],
        target_rotation_base: np.ndarray,
        orientation_tolerance: float,
    ) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_pose_goal"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [max(float(self.args.moveit_position_tolerance), 0.002)]
        pose = Pose()
        pose.position.x = float(target_base[0])
        pose.position.y = float(target_base[1])
        pose.position.z = float(target_base[2])
        pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = "tool0"
        position.constraint_region.primitives.append(primitive)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = "tool0"
        orientation.orientation = self._quat_msg_from_matrix(target_rotation_base)
        tolerance = min(max(float(orientation_tolerance), 0.01), self._tool_orientation_limit_rad())
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 0.8
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _moveit_orientation_path_constraint(self, reference_rotation_base: np.ndarray) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_tool_orientation_limit"
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = "tool0"
        orientation.orientation = self._quat_msg_from_matrix(reference_rotation_base)
        tolerance = self._tool_orientation_limit_rad()
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _moveit_error_message(self, code: int) -> str:
        names = {
            1: "SUCCESS",
            99999: "FAILURE",
            -1: "PLANNING_FAILED",
            -2: "INVALID_MOTION_PLAN",
            -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -4: "CONTROL_FAILED",
            -5: "UNABLE_TO_ACQUIRE_SENSOR_DATA",
            -6: "TIMED_OUT",
            -7: "PREEMPTED",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -26: "START_STATE_INVALID",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS",
            -17: "INVALID_ROBOT_STATE",
            -18: "INVALID_LINK_NAME",
            -21: "FRAME_TRANSFORM_FAILURE",
            -22: "COLLISION_CHECKING_UNAVAILABLE",
            -23: "ROBOT_STATE_STALE",
            -27: "GOAL_STATE_INVALID",
            -28: "UNRECOGNIZED_GOAL_TYPE",
            -31: "NO_IK_SOLUTION",
        }
        return f"{names.get(code, 'UNKNOWN')}({code})"

    def _current_tool_rotation_base(self) -> np.ndarray:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
            base_from_aubo = self._matrix_from_transform(transform)
            return base_from_aubo[:3, :3] @ self.kinematics.fk(
                np.asarray(self.current_arm_joints, dtype=float)
            )[:3, :3]
        except TransformException:
            return self.kinematics.fk(np.asarray(self.current_arm_joints, dtype=float))[:3, :3]

    def _current_end_effector_rotation_base(self) -> np.ndarray:
        if float(self.args.grasp_tcp_offset_m) > 0.0:
            return self._current_tool_rotation_base()
        for frame_id in self._frame_candidates(END_EFFECTOR_FRAME):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.01),
                )
            except TransformException:
                continue
            return self._matrix_from_transform(transform)[:3, :3]
        return self._current_tool_rotation_base()

    def _make_local_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        targets = self._planning_target_samples(preview)
        if not targets:
            return [], "no planning key waypoints"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return [], f"waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"

        aubo_from_base = self._matrix_from_transform(transform)
        base_from_aubo = self._invert_rigid(aubo_from_base)
        current_rotation_base = self._current_end_effector_rotation_base()
        q_seed = np.asarray(self.current_arm_joints, dtype=float)
        q_waypoints: list[np.ndarray] = [np.asarray(q_seed, dtype=float)]
        failures = 0
        collision_failures = 0
        max_position_error = 0.0
        max_orientation_error = 0.0
        orientation_candidates = 0
        for target_name, point, progress in targets:
            best_candidate = None
            for candidate_index, target_rotation_base in enumerate(
                self._target_orientation_candidates_base(
                    point,
                    progress,
                    current_rotation_base,
                    preview.pointcloud_shape,
                    target_name,
                )
            ):
                orientation_candidates += 1
                tool_target = self._tool0_target_from_grasp_target(point, target_rotation_base)
                if tool_target is None:
                    _matrix, message = self._tool_to_grasp_matrix()
                    return [], f"local ik: {message}"
                target_tool0_base, target_tool0_rotation_base = tool_target
                target_position = np.asarray(
                    self._transform_point(transform, target_tool0_base), dtype=float
                )
                target_transform = np.eye(4)
                target_transform[:3, :3] = aubo_from_base[:3, :3] @ target_tool0_rotation_base
                target_transform[:3, 3] = target_position
                ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
                    q_seed,
                    target_transform,
                    position_tolerance=PREVIEW_IK_TOLERANCE_M,
                    orientation_tolerance=PREVIEW_IK_ORIENTATION_TOLERANCE_RAD,
                    damping=PREVIEW_IK_DAMPING,
                    max_iterations=PREVIEW_IK_MAX_ITERATIONS,
                    max_step=PREVIEW_IK_MAX_STEP,
                    orientation_weight=PREVIEW_IK_ORIENTATION_WEIGHT,
                )
                q_unwrapped = q_waypoints[-1] + self._joint_delta(q_solution, q_waypoints[-1])
                endpoint_safe, endpoint_clearance, endpoint_hit_name = self._arm_collision_clearance_base(
                    q_unwrapped, base_from_aubo
                )
                segment_safe, segment_clearance, segment_hit_name = (
                    self._joint_segment_collision_clearance_base(
                        q_waypoints[-1], q_unwrapped, base_from_aubo
                    )
                )
                collision_free = bool(endpoint_safe and segment_safe)
                clearance = min(float(endpoint_clearance), float(segment_clearance))
                hit_name = endpoint_hit_name if not endpoint_safe else segment_hit_name
                joint_step = float(np.linalg.norm(q_unwrapped - q_waypoints[-1]))
                collision_score = 0.0 if collision_free else COLLISION_PENALTY
                score = (
                    collision_score
                    + 150.0 * float(position_error)
                    + 8.0 * float(orientation_error)
                    + 1.2 * joint_step
                    + 0.05 * float(candidate_index)
                    - 0.4 * float(clearance)
                )
                candidate = (
                    score,
                    collision_free,
                    ok,
                    q_solution,
                    q_unwrapped,
                    float(position_error),
                    float(orientation_error),
                    hit_name,
                    target_rotation_base,
                )
                if best_candidate is None or score < best_candidate[0]:
                    best_candidate = candidate

            if best_candidate is None:
                failures += 1
                continue

            (
                _score,
                collision_free,
                ok,
                q_solution,
                q_unwrapped,
                position_error,
                orientation_error,
                hit_name,
                target_rotation_base,
            ) = best_candidate
            if not collision_free:
                collision_failures += 1
                if not bool(self.args.allow_colliding_best_effort):
                    return [], f"collision blocked at {hit_name or 'vehicle'} progress={progress:.2f}"
            if not ok:
                failures += 1
            max_position_error = max(max_position_error, float(position_error))
            max_orientation_error = max(max_orientation_error, float(orientation_error))
            if np.max(np.abs(q_unwrapped - q_waypoints[-1])) > 1e-5:
                q_waypoints.append(q_unwrapped)
            q_seed = np.asarray(q_solution, dtype=float)
            current_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_rotation_base, dtype=float)
            )

        frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
        status = "ok" if failures == 0 else f"best-effort ik_failures={failures}"
        if collision_failures:
            status += f" collision_failures={collision_failures}"
        return (
            frames,
            f"{status}, {limit_message}, "
            f"targets={','.join(name for name, _point, _progress in targets)} "
            f"orientation_candidates={orientation_candidates} "
            f"max_pos_error={max_position_error:.3f}m "
            f"max_ori_error={max_orientation_error:.3f}rad",
        )

    def _trajectory_reference_samples(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[tuple[float, float, float], float]]:
        points = self._trajectory_reference_points(segments)
        if not points:
            return []
        if len(points) == 1:
            return [(points[0], 0.0)]
        return [(point, index / float(len(points) - 1)) for index, point in enumerate(points)]

    def _target_orientation_candidates_base(
        self,
        point_base: tuple[float, float, float],
        progress: float,
        current_rotation_base: np.ndarray,
        pointcloud_shape: PointCloudGraspShape | None = None,
        target_name: str = "",
    ) -> list[np.ndarray]:
        release_tilt = math.radians(float(self.args.release_tool_tilt_deg))
        tilted_down = np.asarray(
            [-math.sin(release_tilt), 0.0, -math.cos(release_tilt)], dtype=float
        )
        downward = self._rotation_from_tool_z_base(tilted_down, np.asarray([1.0, 0.0, 0.0]))

        grasp_hint = np.asarray(point_base, dtype=float) - np.asarray(
            [self.basket_release_base[0], self.basket_release_base[1], point_base[2]],
            dtype=float,
        )
        if np.linalg.norm(grasp_hint) < 1e-6:
            grasp_hint = np.asarray([1.0, 0.0, 0.0], dtype=float)
        grasp_down = self._rotation_from_tool_z_base(
            np.asarray([0.0, 0.0, -1.0], dtype=float), grasp_hint
        )
        shape_hints = self._pointcloud_shape_orientation_hints(pointcloud_shape)

        phase = self._target_orientation_phase(target_name, progress)
        if phase == "current":
            nominal = current_rotation_base
        elif phase == "grasp":
            nominal = grasp_down
        elif phase == "carry":
            nominal = current_rotation_base
        else:
            nominal = downward

        limit_rad = self._tool_orientation_limit_rad()
        if phase in {"current", "carry"}:
            yaw_offsets = (0.0,)
            tilt_offsets = ((0.0, 0.0),)
        elif phase == "grasp":
            yaw_offsets = self._angle_offsets_rad(
                str(self.args.grasp_orientation_yaw_offsets_deg),
                "--grasp-orientation-yaw-offsets-deg",
            )
            tilt_offsets = self._grasp_tilt_offsets_rad()
        else:
            yaw_offsets = self._angle_offsets_rad(
                str(self.args.transit_orientation_yaw_offsets_deg),
                "--transit-orientation-yaw-offsets-deg",
            )
            tilt_offsets = ((0.0, 0.0),)
        candidates: list[np.ndarray] = []
        if phase == "grasp":
            for hint in shape_hints:
                shape_nominal = self._rotation_from_tool_z_base(
                    np.asarray([0.0, 0.0, -1.0], dtype=float), hint
                )
                for yaw in (0.0,):
                    for roll, pitch in tilt_offsets[:3]:
                        candidate = self._orthonormalize_rotation(
                            shape_nominal @ self._rpy_matrix(roll, pitch, yaw)
                        )
                        if not self._is_topdown_grasp_orientation(candidate):
                            continue
                        if not any(
                            self._rotation_distance(candidate, existing) < 1e-4
                            for existing in candidates
                        ):
                            candidates.append(candidate)
                            if len(candidates) >= self._max_grasp_orientation_candidates():
                                return candidates
        for yaw in yaw_offsets:
            for roll, pitch in tilt_offsets:
                candidate = nominal @ self._rpy_matrix(roll, pitch, yaw)
                if phase == "current":
                    candidate = self._limit_rotation_from_reference(
                        current_rotation_base, candidate, limit_rad
                    )
                else:
                    candidate = self._orthonormalize_rotation(candidate)
                if phase in {"grasp", "carry", "release"} and not self._is_topdown_grasp_orientation(
                    candidate
                ):
                    continue
                if not any(
                    self._rotation_distance(candidate, existing) < 1e-4
                    for existing in candidates
                ):
                    candidates.append(candidate)
                    if (
                        phase == "grasp"
                        and len(candidates) >= self._max_grasp_orientation_candidates()
                    ):
                        return candidates
        return candidates

    def _target_orientation_phase(self, target_name: str, progress: float) -> str:
        name = str(target_name).strip()
        if name in {"start_ee", "observe_start"}:
            return "current"
        if name in {"approach", "grasp"}:
            return "grasp"
        if name in {"safe_mid", "lift"}:
            return "carry"
        if name in {"basket_over", "drop"}:
            return "release"
        if progress < 0.18:
            return "current"
        if progress < 0.60:
            return "grasp"
        return "release"

    def _max_grasp_orientation_candidates(self) -> int:
        return max(int(self.args.max_grasp_orientation_candidates), 1)

    def _pointcloud_shape_orientation_hints(
        self, pointcloud_shape: PointCloudGraspShape | None
    ) -> list[np.ndarray]:
        if pointcloud_shape is None:
            return []
        axes: list[np.ndarray] = []
        if (
            pointcloud_shape.visual_axis_base is not None
            and pointcloud_shape.visual_axis_confidence
            >= max(float(self.args.visual_axis_confidence_threshold), 0.0)
        ):
            visual_major = np.asarray(pointcloud_shape.visual_axis_base, dtype=float)
            visual_minor = np.asarray([-visual_major[1], visual_major[0], 0.0], dtype=float)
            axes.extend([visual_minor, -visual_minor, visual_major, -visual_major])
        if pointcloud_shape.axis_confidence >= max(
            float(self.args.pointcloud_axis_confidence_threshold), 0.0
        ):
            axes.extend(
                [
                    np.asarray(pointcloud_shape.minor_axis_base, dtype=float),
                    -np.asarray(pointcloud_shape.minor_axis_base, dtype=float),
                    np.asarray(pointcloud_shape.major_axis_base, dtype=float),
                    -np.asarray(pointcloud_shape.major_axis_base, dtype=float),
                ]
            )
        hints: list[np.ndarray] = []
        for axis in axes:
            axis[2] = 0.0
            norm = float(np.linalg.norm(axis))
            if norm < 1e-6:
                continue
            axis = axis / norm
            if not any(float(np.linalg.norm(axis - existing)) < 1e-3 for existing in hints):
                hints.append(axis)
        return hints

    def _is_topdown_grasp_orientation(self, rotation_base: np.ndarray) -> bool:
        tool_z_base = np.asarray(rotation_base, dtype=float)[:3, 2]
        if float(np.linalg.norm(tool_z_base)) < 1e-9:
            return False
        tool_z_base = tool_z_base / float(np.linalg.norm(tool_z_base))
        max_tilt = min(max(float(self.args.grasp_topdown_max_tilt_deg), 1.0), 89.0)
        return float(np.dot(tool_z_base, np.asarray([0.0, 0.0, -1.0]))) >= math.cos(
            math.radians(max_tilt)
        )

    def _angle_offsets_rad(self, value: str, label: str) -> tuple[float, ...]:
        offsets = [math.radians(item) for item in _float_list(value, label)]
        if not any(abs(item) < 1e-9 for item in offsets):
            offsets.insert(0, 0.0)
        return tuple(offsets)

    def _grasp_tilt_offsets_rad(self) -> tuple[tuple[float, float], ...]:
        values = self._angle_offsets_rad(
            str(self.args.grasp_orientation_tilt_offsets_deg),
            "--grasp-orientation-tilt-offsets-deg",
        )
        offsets: list[tuple[float, float]] = [(0.0, 0.0)]
        for value in values:
            if abs(value) < 1e-9:
                continue
            offsets.append((value, 0.0))
            offsets.append((0.0, value))
        return tuple(offsets)

    def _rotation_from_tool_z_base(self, tool_z: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
        z_axis = np.asarray(tool_z, dtype=float)
        z_axis /= max(float(np.linalg.norm(z_axis)), 1e-9)
        x_axis = np.asarray(x_hint, dtype=float)
        x_axis = x_axis - z_axis * float(np.dot(x_axis, z_axis))
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.asarray([0.0, 1.0, 0.0], dtype=float)
            x_axis = x_axis - z_axis * float(np.dot(x_axis, z_axis))
        x_axis /= max(float(np.linalg.norm(x_axis)), 1e-9)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-9)
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def _blend_rotation(self, start: np.ndarray, end: np.ndarray, amount: float) -> np.ndarray:
        raw = (1.0 - amount) * np.asarray(start, dtype=float) + amount * np.asarray(end, dtype=float)
        u, _s, vh = np.linalg.svd(raw)
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh
        return rotation

    def _tool_orientation_limit_rad(self) -> float:
        return max(math.radians(float(self.args.tool_orientation_limit_deg)), math.radians(1.0))

    def _limit_rotation_from_reference(
        self, reference: np.ndarray, candidate: np.ndarray, limit_rad: float
    ) -> np.ndarray:
        delta = np.asarray(reference, dtype=float).T @ np.asarray(candidate, dtype=float)
        roll, pitch, yaw = self._matrix_to_rpy(delta)
        clipped_delta = self._rpy_matrix(
            float(np.clip(roll, -limit_rad, limit_rad)),
            float(np.clip(pitch, -limit_rad, limit_rad)),
            float(np.clip(yaw, -limit_rad, limit_rad)),
        )
        return self._orthonormalize_rotation(np.asarray(reference, dtype=float) @ clipped_delta)

    def _matrix_to_rpy(self, matrix: np.ndarray) -> tuple[float, float, float]:
        rotation = np.asarray(matrix, dtype=np.float64)
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
        cp = math.cos(pitch)
        if abs(cp) > 1e-8:
            roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
            yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        else:
            roll = 0.0
            yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
        return (self._wrap_angle(roll), self._wrap_angle(pitch), self._wrap_angle(yaw))

    def _wrap_angle(self, angle: float) -> float:
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

    def _smoothstep(self, value: float) -> float:
        x = min(max(float(value), 0.0), 1.0)
        return x * x * (3.0 - 2.0 * x)

    def _rotation_distance(self, left: np.ndarray, right: np.ndarray) -> float:
        delta = np.asarray(left, dtype=float).T @ np.asarray(right, dtype=float)
        trace = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
        return math.acos(trace)

    def _orthonormalize_rotation(self, matrix: np.ndarray) -> np.ndarray:
        u, _s, vh = np.linalg.svd(np.asarray(matrix, dtype=float))
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh
        return rotation

    def _arm_collision_clearance_base(
        self, q: np.ndarray, base_from_aubo: np.ndarray
    ) -> tuple[bool, float, str | None]:
        link_points = [
            self._transform_matrix_point(base_from_aubo, transform[:3, 3])
            for _name, transform in self.kinematics.link_transforms(np.asarray(q, dtype=float))
        ]
        samples_per_link = max(int(self.args.arm_collision_samples_per_link), 2)
        radius = max(float(self.args.arm_collision_radius), 0.0)
        ground_threshold = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.ground_clearance), 0.0)
            + radius
        )
        min_clearance = float("inf")
        hit_name: str | None = None
        for start, end in zip(link_points[1:], link_points[2:]):
            a = np.asarray(start, dtype=float)
            b = np.asarray(end, dtype=float)
            for t in np.linspace(0.0, 1.0, samples_per_link):
                point = a * (1.0 - t) + b * t
                ground_clearance = float(point[2] - ground_threshold)
                if ground_clearance < min_clearance:
                    min_clearance = ground_clearance
                    hit_name = "ground/arm"
                if ground_clearance < 0.0:
                    return False, ground_clearance, "ground/arm"
                for box in self.collision_boxes:
                    clearance = self._point_box_clearance(point, box) - radius
                    if clearance < min_clearance:
                        min_clearance = float(clearance)
                        hit_name = box.name
                    if clearance < 0.0:
                        return False, float(clearance), box.name
        return True, float(min_clearance if np.isfinite(min_clearance) else 0.0), hit_name

    def _point_box_clearance(self, point: np.ndarray, box: CollisionBox) -> float:
        center = np.asarray(box.center, dtype=float)
        half = 0.5 * np.asarray(box.size, dtype=float)
        delta = np.abs(np.asarray(point, dtype=float) - center) - half
        outside = np.maximum(delta, 0.0)
        outside_distance = float(np.linalg.norm(outside))
        if outside_distance > 0.0:
            return outside_distance
        return float(np.max(delta))

    def _trajectory_reference_points(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[float, float, float]]:
        step_m = max(float(self.args.trajectory_cartesian_step), 0.005)
        points: list[tuple[float, float, float]] = []
        for segment in segments:
            count = max(int(np.ceil(self._segment_length(segment) / step_m)) + 1, 2)
            for index, t in enumerate(np.linspace(0.0, 1.0, count)):
                if points and index == 0:
                    continue
                points.append(self._sample_segment(segment, float(t)))
        return points

    def _generate_limited_joint_frames(
        self, waypoints: list[np.ndarray]
    ) -> tuple[list[JointTrajectoryFrame], str]:
        if not waypoints:
            return [], "no joint waypoints"
        rate = max(float(self.args.playback_rate), 1.0)
        dt = 1.0 / rate
        max_speed = max(float(self.args.preview_max_joint_speed), 0.01)
        max_accel = max(float(self.args.preview_max_joint_accel), 0.05)
        max_jerk = max(float(self.args.preview_max_joint_jerk), 0.1)
        tau = max(float(self.args.preview_smoothing_tau), 0.02)
        tolerance = max(float(self.args.trajectory_joint_tolerance), 1e-4)
        max_frames = max(int(max(float(self.args.trajectory_max_duration), dt) * rate), 1)

        q = np.asarray(waypoints[0], dtype=float)
        velocity = np.zeros(6, dtype=float)
        accel = np.zeros(6, dtype=float)
        frames = [self._joint_frame(0.0, q, velocity, accel)]
        target_index = 1
        time_from_start = 0.0
        max_observed_speed = 0.0
        max_observed_accel = 0.0

        while target_index < len(waypoints) and len(frames) < max_frames:
            target = np.asarray(waypoints[target_index], dtype=float)
            delta = target - q
            if float(np.max(np.abs(delta))) <= tolerance:
                target_index += 1
                continue

            target_velocity = np.clip(delta / max(dt, 1e-9), -max_speed, max_speed)
            alpha = min(max(dt / tau, 0.0), 1.0)
            smoothed_target = velocity + alpha * (target_velocity - velocity)
            desired_accel = np.clip((smoothed_target - velocity) / dt, -max_accel, max_accel)
            accel_delta = np.clip(desired_accel - accel, -max_jerk * dt, max_jerk * dt)
            accel = accel + accel_delta
            next_velocity = np.clip(velocity + accel * dt, -max_speed, max_speed)
            step = next_velocity * dt

            for joint_index, (candidate_step, remaining) in enumerate(zip(step, delta)):
                if abs(candidate_step) >= abs(remaining) or candidate_step * remaining < 0.0:
                    step[joint_index] = remaining
                    next_velocity[joint_index] = 0.0
                    accel[joint_index] = 0.0

            q = q + step
            velocity = next_velocity
            time_from_start += dt
            max_observed_speed = max(max_observed_speed, float(np.max(np.abs(velocity))))
            max_observed_accel = max(max_observed_accel, float(np.max(np.abs(accel))))
            frames.append(self._joint_frame(time_from_start, q, velocity, accel))

        if frames:
            final = np.asarray(frames[-1].positions, dtype=float)
            frames.append(self._joint_frame(frames[-1].time_from_start + dt, final, np.zeros(6), np.zeros(6)))
        reached = target_index >= len(waypoints)
        status = "reached" if reached else "truncated"
        return (
            frames,
            f"frames={len(frames)} duration={frames[-1].time_from_start:.2f}s "
            f"{status} max_speed={max_observed_speed:.3f}rad/s "
            f"max_accel={max_observed_accel:.3f}rad/s2",
        )

    def _joint_frame(
        self, time_from_start: float, q: np.ndarray, velocity: np.ndarray, accel: np.ndarray
    ) -> JointTrajectoryFrame:
        return JointTrajectoryFrame(
            float(time_from_start),
            tuple(float(value) for value in q),  # type: ignore[arg-type]
            tuple(float(value) for value in velocity),  # type: ignore[arg-type]
            tuple(float(value) for value in accel),  # type: ignore[arg-type]
        )

    def _make_base_path(
        self,
        camera_path: list[tuple[float, float, float]],
        roi_points: list[tuple[float, float, float]] | None = None,
        visual_axis_camera: np.ndarray | None = None,
        visual_axis_confidence: float = 0.0,
    ) -> tuple[
        list[tuple[float, float, float]],
        list[CartesianSegment],
        list[tuple[str, tuple[float, float, float]]],
        tuple[float, float, float] | None,
        PointCloudGraspShape | None,
    ]:
        if self.latest_depth is None:
            return [], [], [], None, None
        source_frame = self.latest_depth.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._throttled_log(f"waiting for TF {self.base_frame} <- {source_frame}: {exc}")
            return [], [], [], None, None

        base_grasp_path = [
            self._apply_grasp_base_offset(self._transform_point(transform, point))
            for point in camera_path
        ]
        pointcloud_shape = self._estimate_pointcloud_grasp_shape_base(
            roi_points,
            transform,
            visual_axis_camera,
            visual_axis_confidence,
        )
        if pointcloud_shape is not None and pointcloud_shape.z_bias_m > 1e-6:
            base_grasp_path = self._apply_visible_upper_half_bias(
                base_grasp_path, pointcloud_shape.z_bias_m
            )
            pointcloud_shape = replace(
                pointcloud_shape,
                center_base=(
                    pointcloud_shape.center_base[0],
                    pointcloud_shape.center_base[1],
                    pointcloud_shape.center_base[2] - pointcloud_shape.z_bias_m,
                ),
            )
        start_base = self._current_grasp_origin_base()
        if start_base is None:
            start_base = self._frame_origin_in_base(END_EFFECTOR_FRAME)
        if start_base is None:
            start_base = self._transform_point(transform, (0.0, 0.0, 0.0))
        observe_base = start_base
        pregrasp_base = base_grasp_path[0]
        grasp_base = base_grasp_path[1]
        lift_base = base_grasp_path[2]
        transit_z = max(
            float(self.args.transit_height),
            lift_base[2] + 0.04,
            self.basket_approach_base[2],
        )
        safe_mid_base = (
            0.5 * (lift_base[0] + self.basket_approach_base[0]),
            0.5 * (lift_base[1] + self.basket_approach_base[1]),
            transit_z,
        )
        waypoints = [
            ("start_ee", start_base),
            ("approach", pregrasp_base),
            ("grasp", grasp_base),
            ("safe_mid", safe_mid_base),
            ("basket_over", self.basket_approach_base),
            ("drop", self.basket_release_base),
            ("observe_start", observe_base),
        ]
        raw_segments = [
            CartesianSegment(
                "quadratic",
                start_base,
                pregrasp_base,
                self._arc_control_point(start_base, pregrasp_base),
            ),
            CartesianSegment("line", pregrasp_base, grasp_base),
            CartesianSegment("line", grasp_base, lift_base),
            CartesianSegment(
                "quadratic",
                lift_base,
                safe_mid_base,
                self._arc_control_point(lift_base, safe_mid_base),
            ),
            CartesianSegment(
                "quadratic",
                safe_mid_base,
                self.basket_approach_base,
                self._arc_control_point(safe_mid_base, self.basket_approach_base),
            ),
            CartesianSegment("line", self.basket_approach_base, self.basket_release_base),
            CartesianSegment(
                "quadratic",
                self.basket_release_base,
                observe_base,
                self._arc_control_point(self.basket_release_base, observe_base),
            ),
        ]
        segments = self._time_parameterize_segments(raw_segments)
        path = self._sample_trajectory_for_display(segments)
        return path, segments, waypoints, grasp_base, pointcloud_shape

    def _estimate_pointcloud_grasp_shape_base(
        self,
        roi_points: list[tuple[float, float, float]] | None,
        transform,
        visual_axis_camera: np.ndarray | None = None,
        visual_axis_confidence: float = 0.0,
    ) -> PointCloudGraspShape | None:
        if bool(self.args.disable_pointcloud_grasp_shape):
            return None
        visual_axis_base = self._visual_axis_base(transform, visual_axis_camera)
        if not roi_points:
            return self._visual_only_grasp_shape(visual_axis_base, visual_axis_confidence, 0)
        min_points = max(int(self.args.pointcloud_grasp_min_points), 6)
        if len(roi_points) < min_points:
            return self._visual_only_grasp_shape(
                visual_axis_base, visual_axis_confidence, len(roi_points)
            )
        base_points = np.asarray(
            [self._transform_point(transform, point) for point in roi_points],
            dtype=np.float64,
        )
        finite = np.all(np.isfinite(base_points), axis=1)
        base_points = base_points[finite]
        if base_points.shape[0] < min_points:
            return self._visual_only_grasp_shape(
                visual_axis_base, visual_axis_confidence, int(base_points.shape[0])
            )

        low, high = np.percentile(base_points, [5.0, 95.0], axis=0)
        trimmed = base_points[np.all((base_points >= low) & (base_points <= high), axis=1)]
        if trimmed.shape[0] >= min_points:
            base_points = trimmed

        center = np.median(base_points, axis=0)
        xy = base_points[:, :2]
        xy_center = np.median(xy, axis=0)
        centered_xy = xy - xy_center
        if centered_xy.shape[0] < 3:
            return None
        cov = np.cov(centered_xy.T)
        if not np.all(np.isfinite(cov)):
            return None
        eig_values, eig_vectors = np.linalg.eigh(cov)
        order = np.argsort(eig_values)[::-1]
        eig_values = eig_values[order]
        eig_vectors = eig_vectors[:, order]
        if float(eig_values[0]) <= 1e-10:
            return None

        major_xy = eig_vectors[:, 0].astype(np.float64)
        major_xy /= max(float(np.linalg.norm(major_xy)), 1e-9)
        if major_xy[0] < -1e-9 or (abs(major_xy[0]) < 1e-9 and major_xy[1] < 0.0):
            major_xy *= -1.0
        minor_xy = np.asarray([-major_xy[1], major_xy[0]], dtype=np.float64)

        major_projection = centered_xy @ major_xy
        minor_projection = centered_xy @ minor_xy
        extent_major = float(np.percentile(major_projection, 90.0) - np.percentile(major_projection, 10.0))
        extent_minor = float(np.percentile(minor_projection, 90.0) - np.percentile(minor_projection, 10.0))
        extent_z = float(np.percentile(base_points[:, 2], 90.0) - np.percentile(base_points[:, 2], 10.0))
        anisotropy = float((eig_values[0] - eig_values[1]) / max(eig_values[0] + eig_values[1], 1e-9))
        density_score = min(float(base_points.shape[0]) / float(max(min_points * 2, 1)), 1.0)
        axis_confidence = float(np.clip(anisotropy * density_score, 0.0, 1.0))
        z_bias = min(
            max(float(self.args.pointcloud_visible_upper_half_z_bias_ratio), 0.0)
            * max(extent_z, 0.0),
            max(float(self.args.pointcloud_visible_upper_half_z_bias_max), 0.0),
        )
        ground_limit = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.tool_ground_clearance), 0.0)
            + 0.01
        )
        z_bias = min(z_bias, max(float(center[2]) - ground_limit, 0.0))
        shifted_center = self._apply_grasp_base_offset(
            (float(center[0]), float(center[1]), float(center[2]))
        )
        return PointCloudGraspShape(
            center_base=shifted_center,
            major_axis_base=(float(major_xy[0]), float(major_xy[1]), 0.0),
            minor_axis_base=(float(minor_xy[0]), float(minor_xy[1]), 0.0),
            visual_axis_base=visual_axis_base,
            extent_major_m=max(extent_major, 0.0),
            extent_minor_m=max(extent_minor, 0.0),
            extent_z_m=max(extent_z, 0.0),
            axis_confidence=axis_confidence,
            visual_axis_confidence=float(np.clip(visual_axis_confidence, 0.0, 1.0))
            if visual_axis_base is not None
            else 0.0,
            point_count=int(base_points.shape[0]),
            z_bias_m=max(float(z_bias), 0.0),
        )

    def _visual_only_grasp_shape(
        self,
        visual_axis_base: tuple[float, float, float] | None,
        visual_axis_confidence: float,
        point_count: int,
    ) -> PointCloudGraspShape | None:
        if visual_axis_base is None:
            return None
        return PointCloudGraspShape(
            center_base=(0.0, 0.0, 0.0),
            major_axis_base=visual_axis_base,
            minor_axis_base=(-visual_axis_base[1], visual_axis_base[0], 0.0),
            visual_axis_base=visual_axis_base,
            extent_major_m=0.0,
            extent_minor_m=0.0,
            extent_z_m=0.0,
            axis_confidence=0.0,
            visual_axis_confidence=float(np.clip(visual_axis_confidence, 0.0, 1.0)),
            point_count=max(int(point_count), 0),
            z_bias_m=0.0,
        )

    def _visual_axis_base(
        self, transform, visual_axis_camera: np.ndarray | None
    ) -> tuple[float, float, float] | None:
        if visual_axis_camera is None:
            return None
        matrix = self._matrix_from_transform(transform)
        axis = np.asarray(matrix[:3, :3], dtype=np.float64) @ np.asarray(
            visual_axis_camera, dtype=np.float64
        )
        axis[2] = 0.0
        norm = float(np.linalg.norm(axis))
        if norm < 1e-6:
            return None
        axis /= norm
        if axis[0] < -1e-9 or (abs(axis[0]) < 1e-9 and axis[1] < 0.0):
            axis *= -1.0
        return (float(axis[0]), float(axis[1]), 0.0)

    def _apply_visible_upper_half_bias(
        self,
        base_grasp_path: list[tuple[float, float, float]],
        z_bias: float,
    ) -> list[tuple[float, float, float]]:
        if z_bias <= 0.0 or len(base_grasp_path) < 2:
            return base_grasp_path
        ground_limit = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.tool_ground_clearance), 0.0)
            + 0.01
        )
        shifted: list[tuple[float, float, float]] = []
        for index, point in enumerate(base_grasp_path):
            if index in {1, 2}:
                z = max(float(point[2]) - z_bias, ground_limit)
                shifted.append((float(point[0]), float(point[1]), z))
            else:
                shifted.append(point)
        return shifted

    def _apply_grasp_base_offset(
        self, point_base: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        offset = np.asarray(self.grasp_base_offset, dtype=np.float64)
        point = np.asarray(point_base, dtype=np.float64)
        shifted = point + offset
        return (float(shifted[0]), float(shifted[1]), float(shifted[2]))

    def _time_parameterize_segments(
        self, segments: list[CartesianSegment]
    ) -> list[CartesianSegment]:
        if not segments:
            return []
        lengths = [max(self._segment_length(segment), 1e-6) for segment in segments]
        total = max(sum(lengths), 1e-6)
        period = max(float(self.args.playback_period), 0.5)
        return [
            CartesianSegment(
                kind=segment.kind,
                start=segment.start,
                end=segment.end,
                control=segment.control,
                duration=period * length / total,
            )
            for segment, length in zip(segments, lengths)
        ]

    def _sample_trajectory_for_display(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[float, float, float]]:
        path: list[tuple[float, float, float]] = []
        for segment in segments:
            if segment.kind == "line" and segment.start == self.basket_approach_base:
                samples = self.args.basket_descent_samples
            else:
                samples = self.args.line_samples if segment.kind == "line" else self.args.arc_samples
            points = [
                self._sample_segment(segment, t)
                for t in np.linspace(0.0, 1.0, max(int(samples), 2))
            ]
            self._extend_path(path, points)
        return path

    def _segment_length(self, segment: CartesianSegment) -> float:
        points = [self._sample_segment(segment, t) for t in np.linspace(0.0, 1.0, 25)]
        return float(
            sum(
                np.linalg.norm(np.asarray(b, dtype=np.float64) - np.asarray(a, dtype=np.float64))
                for a, b in zip(points, points[1:])
            )
        )

    def _sample_segment(
        self, segment: CartesianSegment, t: float
    ) -> tuple[float, float, float]:
        u = min(max(float(t), 0.0), 1.0)
        if segment.kind == "quadratic" and segment.control is not None:
            return self._quadratic_bezier_point(segment.start, segment.control, segment.end, u)
        a = np.asarray(segment.start, dtype=np.float64)
        b = np.asarray(segment.end, dtype=np.float64)
        point = a * (1.0 - u) + b * u
        return (float(point[0]), float(point[1]), float(point[2]))

    def _point_on_trajectory(
        self, segments: list[CartesianSegment], progress: float
    ) -> tuple[float, float, float]:
        if not segments:
            return (0.0, 0.0, 0.0)
        total = sum(max(float(segment.duration), 0.0) for segment in segments)
        if total <= 1e-9:
            return segments[0].start
        target_time = min(max(float(progress), 0.0), 1.0) * total
        elapsed = 0.0
        for segment in segments:
            duration = max(float(segment.duration), 1e-9)
            if elapsed + duration >= target_time:
                return self._sample_segment(segment, (target_time - elapsed) / duration)
            elapsed += duration
        return segments[-1].end

    def _base_point_to_aubo(
        self, point: tuple[float, float, float]
    ) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException as exc:
            self.preview_ik_message = f"waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"
            return None
        return self._transform_point(transform, point)

    def _frame_origin_in_base(self, frame_id: str) -> tuple[float, float, float] | None:
        last_error = ""
        for candidate in self._frame_candidates(frame_id):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    candidate,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.03),
                )
            except TransformException as exc:
                last_error = str(exc)
                continue
            translation = transform.transform.translation
            return (float(translation.x), float(translation.y), float(translation.z))
        self._throttled_log(f"waiting for TF {self.base_frame} <- {frame_id}: {last_error}")
        return None

    def _extend_path(
        self, path: list[tuple[float, float, float]], segment: list[tuple[float, float, float]]
    ) -> None:
        for index, point in enumerate(segment):
            if path and index == 0:
                continue
            path.append(point)

    def _sample_line(
        self, start: tuple[float, float, float], end: tuple[float, float, float], samples: int
    ) -> list[tuple[float, float, float]]:
        count = max(int(samples), 2)
        a = np.asarray(start, dtype=np.float64)
        b = np.asarray(end, dtype=np.float64)
        return [
            tuple((a * (1.0 - t) + b * t).astype(float))
            for t in np.linspace(0.0, 1.0, count)
        ]

    def _sample_quadratic_bezier(
        self,
        start: tuple[float, float, float],
        control: tuple[float, float, float],
        end: tuple[float, float, float],
        samples: int,
    ) -> list[tuple[float, float, float]]:
        count = max(int(samples), 3)
        points = []
        for t in np.linspace(0.0, 1.0, count):
            points.append(self._quadratic_bezier_point(start, control, end, float(t)))
        return points

    def _quadratic_bezier_point(
        self,
        start: tuple[float, float, float],
        control: tuple[float, float, float],
        end: tuple[float, float, float],
        t: float,
    ) -> tuple[float, float, float]:
        a = np.asarray(start, dtype=np.float64)
        c = np.asarray(control, dtype=np.float64)
        b = np.asarray(end, dtype=np.float64)
        one = 1.0 - t
        point = one * one * a + 2.0 * one * t * c + t * t * b
        return (float(point[0]), float(point[1]), float(point[2]))

    def _arc_control_point(
        self, start: tuple[float, float, float], end: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        sx, sy, sz = start
        ex, ey, ez = end
        return (
            0.5 * (sx + ex),
            0.5 * (sy + ey),
            max(sz, ez, float(self.args.transit_height)) + max(float(self.args.transit_arc_height), 0.0),
        )

    def _transform_point(self, transform, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        vec = np.asarray(xyz, dtype=np.float64)
        out = matrix @ vec + np.asarray([translation.x, translation.y, translation.z])
        return (float(out[0]), float(out[1]), float(out[2]))

    def _matrix_from_transform(self, transform) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        matrix[:3, 3] = np.asarray([translation.x, translation.y, translation.z], dtype=np.float64)
        return matrix

    def _invert_rigid(self, transform: np.ndarray) -> np.ndarray:
        rotation = np.asarray(transform[:3, :3], dtype=np.float64)
        translation = np.asarray(transform[:3, 3], dtype=np.float64)
        inverse = np.eye(4, dtype=np.float64)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def _transform_matrix_point(self, transform: np.ndarray, xyz: np.ndarray) -> tuple[float, float, float]:
        vec = np.asarray([float(xyz[0]), float(xyz[1]), float(xyz[2]), 1.0], dtype=np.float64)
        out = np.asarray(transform, dtype=np.float64) @ vec
        return (float(out[0]), float(out[1]), float(out[2]))

    def _quat_to_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = max((x * x + y * y + z * z + w * w) ** 0.5, 1e-9)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.asarray(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ],
            dtype=np.float64,
        )

    def _quat_msg_from_matrix(self, matrix: np.ndarray) -> Quaternion:
        rotation = np.asarray(matrix, dtype=np.float64)
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
        else:
            diagonal = np.diag(rotation)
            index = int(np.argmax(diagonal))
            if index == 0:
                scale = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12)) * 2.0
                w = (rotation[2, 1] - rotation[1, 2]) / scale
                x = 0.25 * scale
                y = (rotation[0, 1] + rotation[1, 0]) / scale
                z = (rotation[0, 2] + rotation[2, 0]) / scale
            elif index == 1:
                scale = math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12)) * 2.0
                w = (rotation[0, 2] - rotation[2, 0]) / scale
                x = (rotation[0, 1] + rotation[1, 0]) / scale
                y = 0.25 * scale
                z = (rotation[1, 2] + rotation[2, 1]) / scale
            else:
                scale = math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12)) * 2.0
                w = (rotation[1, 0] - rotation[0, 1]) / scale
                x = (rotation[0, 2] + rotation[2, 0]) / scale
                y = (rotation[1, 2] + rotation[2, 1]) / scale
                z = 0.25 * scale
        norm = max(math.sqrt(x * x + y * y + z * z + w * w), 1e-9)
        return Quaternion(x=float(x / norm), y=float(y / norm), z=float(z / norm), w=float(w / norm))

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return rz @ ry @ rx

    def _basket_path_safe(self, points: list[tuple[float, float, float]]) -> bool:
        if not points:
            return False
        radius = max(float(self.args.gripper_radius), 0.0)
        mins = np.asarray(self.basket_keepout_min, dtype=np.float64) - radius
        maxs = np.asarray(self.basket_keepout_max, dtype=np.float64) + radius
        for start, end in zip(points, points[1:]):
            a = np.asarray(start, dtype=np.float64)
            b = np.asarray(end, dtype=np.float64)
            for t in np.linspace(0.0, 1.0, 25):
                p = a * (1.0 - t) + b * t
                if np.all(p >= mins) and np.all(p <= maxs):
                    return False
        return True

    def _markers(self, preview: GraspPreview, header: Header) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        bbox = self._marker(header, "bbox", 1, Marker.LINE_LIST, _color(1.0, 0.55, 0.0))
        bbox.scale.x = 0.008
        corners = preview.bbox_points
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            bbox.points.extend([_point(corners[a]), _point(corners[b])])
        markers.markers.append(bbox)

        path = self._marker(header, "camera_path", 2, Marker.LINE_STRIP, _color(0.0, 0.8, 1.0))
        path.scale.x = 0.012
        for xyz in [
            preview.pregrasp_xyz,
            preview.grasp_xyz,
            preview.lift_xyz,
            preview.retreat_xyz,
        ]:
            path.points.append(_point(xyz))
        markers.markers.append(path)

        arrow = self._marker(header, "approach", 3, Marker.ARROW, _color(0.0, 1.0, 0.25))
        arrow.scale.x = 0.018
        arrow.scale.y = 0.035
        arrow.scale.z = 0.035
        arrow.points.extend([_point(preview.pregrasp_xyz), _point(preview.grasp_xyz)])
        markers.markers.append(arrow)

        markers.markers.append(
            self._sphere(header, "pregrasp", 4, preview.pregrasp_xyz, _color(1.0, 0.85, 0.0))
        )
        markers.markers.append(
            self._sphere(header, "grasp", 5, preview.grasp_xyz, _color(0.0, 1.0, 0.2))
        )
        markers.markers.append(
            self._sphere(header, "lift", 6, preview.lift_xyz, _color(0.35, 0.45, 1.0))
        )

        text = self._marker(header, "label", 7, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0))
        text.pose.position = _point(preview.lift_xyz)
        text.pose.position.y -= 0.04
        text.scale.z = 0.055
        gx, gy, gz = preview.grasp_xyz
        text.text = (
            f"{preview.detection.label} {preview.detection.confidence:.2f}\n"
            f"depth {preview.depth_m:.3f} m\n"
            f"grasp [{gx:.3f}, {gy:.3f}, {gz:.3f}]\n"
            f"3D {preview.snapshot_reason}\n"
            f"IK {preview.ik_message}\n"
            f"basket {'clear' if preview.basket_safe else 'risk'}"
        )
        markers.markers.append(text)
        if preview.base_path_xyz:
            markers.markers.extend(self._basket_markers(preview))
        return markers

    def _basket_markers(self, preview: GraspPreview) -> list[Marker]:
        if not preview.base_path_xyz:
            return []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
        safe_color = _color(0.0, 0.95, 0.25) if preview.basket_safe else _color(1.0, 0.0, 0.0)
        markers: list[Marker] = []

        basket_path = self._marker(header, "planned_grasp_path", 20, Marker.LINE_STRIP, safe_color)
        basket_path.scale.x = 0.018
        for xyz in preview.base_path_xyz:
            basket_path.points.append(_point(xyz))
        markers.append(basket_path)
        markers.extend(self._base_waypoint_markers(header, preview))
        markers.extend(self._path_playback_markers(header, preview))

        for idx, (name, xyz, color) in enumerate(
            [
                ("basket_approach", self.basket_approach_base, _color(0.0, 0.65, 1.0)),
                ("basket_release", self.basket_release_base, _color(1.0, 0.35, 0.0)),
            ],
            start=21,
        ):
            markers.append(self._sphere(header, name, idx, xyz, color))

        keepout = self._marker(header, "basket_keepout", 30, Marker.CUBE, _color(1.0, 0.0, 0.0, 0.20))
        min_x, min_y, min_z = self.basket_keepout_min
        max_x, max_y, max_z = self.basket_keepout_max
        keepout.pose.position = _point(
            ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)
        )
        keepout.scale.x = max_x - min_x
        keepout.scale.y = max_y - min_y
        keepout.scale.z = max_z - min_z
        markers.append(keepout)

        label = self._marker(header, "basket_label", 31, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0))
        label.pose.position = _point((self.basket_release_base[0], self.basket_release_base[1], self.basket_release_base[2] + 0.08))
        label.scale.z = 0.055
        label.text = "release over basket\nkeepout clear" if preview.basket_safe else "basket collision risk"
        markers.append(label)
        return markers

    def _base_waypoint_markers(self, header: Header, preview: GraspPreview) -> list[Marker]:
        colors = {
            "start_ee": _color(1.0, 1.0, 1.0),
            "approach": _color(1.0, 0.85, 0.0),
            "grasp": _color(0.0, 1.0, 0.2),
            "safe_mid": _color(0.0, 0.65, 1.0),
            "basket_over": _color(0.0, 0.85, 1.0),
            "drop": _color(1.0, 0.35, 0.0),
            "observe_start": _color(0.75, 0.4, 1.0),
        }
        markers: list[Marker] = []
        for index, (name, xyz) in enumerate(preview.base_waypoints):
            sphere = self._sphere(
                header,
                "task_waypoints",
                40 + index,
                xyz,
                colors.get(name, _color(1.0, 1.0, 1.0)),
            )
            sphere.scale.x = 0.045
            sphere.scale.y = 0.045
            sphere.scale.z = 0.045
            markers.append(sphere)

            text = self._marker(
                header,
                "task_waypoint_labels",
                50 + index,
                Marker.TEXT_VIEW_FACING,
                _color(1.0, 1.0, 1.0),
            )
            text.pose.position = _point((xyz[0], xyz[1], xyz[2] + 0.055))
            text.scale.z = 0.04
            text.text = name.replace("_", " ")
            markers.append(text)
        return markers

    def _path_playback_markers(self, header: Header, preview: GraspPreview) -> list[Marker]:
        if not preview.arm_trajectory_frames:
            return []
        cursor_xyz = self._current_grasp_position_base(preview)
        if cursor_xyz is None:
            return []
        progress = self._playback_progress(preview)

        cursor = self._sphere(
            header, "path_playback", 70, cursor_xyz, _color(1.0, 0.0, 1.0)
        )
        cursor.scale.x = 0.06
        cursor.scale.y = 0.06
        cursor.scale.z = 0.06

        label = self._marker(
            header, "path_playback", 71, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0)
        )
        label.pose.position = _point((cursor_xyz[0], cursor_xyz[1], cursor_xyz[2] + 0.07))
        label.scale.z = 0.045
        label.text = f"path playback {progress * 100.0:04.1f}%"
        return [cursor, label]

    def _playback_progress(self, preview: GraspPreview) -> float:
        duration = (
            max(float(preview.arm_trajectory_frames[-1].time_from_start), 1e-6)
            if preview.arm_trajectory_frames
            else max(float(self.args.playback_period), 0.5)
        )
        period_override = float(self.args.playback_period)
        period = max(period_override, duration) if period_override > 0.0 else duration
        start_time = self.plan_lock_time or self.snapshot_time or time.monotonic()
        elapsed = max(time.monotonic() - start_time, 0.0)
        t = elapsed % max(period, 1e-6) if bool(self.args.loop_playback) else min(elapsed, duration)
        return min(max(t / duration, 0.0), 1.0)

    def _marker(
        self, header: Header, namespace: str, marker_id: int, marker_type: int, color: ColorRGBA
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color = color
        marker.pose.orientation.w = 1.0
        return marker

    def _sphere(
        self, header: Header, namespace: str, marker_id: int, xyz: tuple[float, float, float], color
    ) -> Marker:
        marker = self._marker(header, namespace, marker_id, Marker.SPHERE, color)
        marker.pose.position = _point(xyz)
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        return marker

    def _path(self, preview: GraspPreview, header: Header) -> PathMsg:
        msg = PathMsg()
        path_header = Header()
        path_header.stamp = self.get_clock().now().to_msg()
        path_header.frame_id = self.base_frame
        msg.header = path_header
        if preview.base_path_xyz:
            path_points = preview.base_path_xyz
        else:
            path_points = []
        for xyz in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = _point(xyz)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_annotated(self, image: np.ndarray, preview: GraspPreview, header: Header) -> None:
        gx, gy, gz = preview.grasp_xyz
        state = "PLAN_LOCKED / YOLO paused" if self.inference_paused else "SEARCH_2D"
        cv2.putText(
            image,
            f"{state} depth={preview.depth_m:.3f}m {preview.snapshot_reason} "
            f"grasp=({gx:.2f},{gy:.2f},{gz:.2f}) "
            f"traj_pts={len(preview.base_path_xyz)} basket={'clear' if preview.basket_safe else 'risk'}",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        self.image_pub.publish(_image_from_bgr(image, header))

    def _pointcloud_shape_payload(
        self, shape: PointCloudGraspShape | None
    ) -> dict[str, object] | None:
        if shape is None:
            return None
        return {
            "center_base": shape.center_base,
            "major_axis_base": shape.major_axis_base,
            "minor_axis_base": shape.minor_axis_base,
            "visual_axis_base": shape.visual_axis_base,
            "extent_major_m": shape.extent_major_m,
            "extent_minor_m": shape.extent_minor_m,
            "extent_z_m": shape.extent_z_m,
            "axis_confidence": shape.axis_confidence,
            "visual_axis_confidence": shape.visual_axis_confidence,
            "point_count": shape.point_count,
            "visible_upper_half_z_bias_m": shape.z_bias_m,
        }

    def _save_latest(
        self, annotated: np.ndarray, raw: np.ndarray, preview: GraspPreview
    ) -> None:
        cv2.imwrite(str(self.save_dir / "latest_raw.jpg"), raw)
        cv2.imwrite(str(self.save_dir / "latest_annotated.jpg"), annotated)
        payload = {
            "label": preview.detection.label,
            "class_id": preview.detection.class_id,
            "confidence": preview.detection.confidence,
            "bbox_xyxy": preview.detection.xyxy,
            "has_mask": preview.detection.mask_xy is not None,
            "mask_area_px": preview.detection.mask_area_px,
            "roi_source": preview.roi_source,
            "depth_m": preview.depth_m,
            "grasp_camera_xyz": preview.grasp_xyz,
            "path_camera_xyz": [
                preview.pregrasp_xyz,
                preview.grasp_xyz,
                preview.lift_xyz,
                preview.retreat_xyz,
            ],
            "depth_projection": {
                "flip_x": bool(self.args.depth_projection_flip_x),
                "flip_y": bool(self.args.depth_projection_flip_y),
            },
            "grasp_base_offset_xyz": self.grasp_base_offset,
            "pointcloud_grasp_shape": self._pointcloud_shape_payload(preview.pointcloud_shape),
            "planned_grasp_path_base_xyz": preview.base_path_xyz,
            "planned_grasp_path_source": (
                "fk_from_arm_trajectory_frames" if preview.arm_trajectory_frames else "pending"
                if not preview.base_path_xyz
                else "cartesian_reference"
            ),
            "trajectory_segments_base": [
                {
                    "kind": segment.kind,
                    "start": segment.start,
                    "control": segment.control,
                    "end": segment.end,
                    "duration": segment.duration,
                }
                for segment in preview.base_trajectory_segments
            ],
            "waypoints_base": [
                {"name": name, "xyz": xyz} for name, xyz in preview.base_waypoints
            ],
            "planning_key_waypoints": [
                {"name": name, "xyz": xyz}
                for name, xyz, _progress in self._planning_target_samples(preview)
            ],
            "trajectory_mode": "constraint_generated_joint_trajectory",
            "end_effector_frame": END_EFFECTOR_FRAME,
            "planning_tcp": {
                "source": (
                    "tool0_virtual_z_offset"
                    if float(self.args.grasp_tcp_offset_m) > 0.0
                    else "urdf_grasp_frame"
                ),
                "tool0_z_offset_m": max(float(self.args.grasp_tcp_offset_m), 0.0),
                "tool_to_grasp_frames": self.tool_to_grasp_frames,
            },
            "path_playback": True,
            "path_playback_loop": bool(self.args.loop_playback),
            "path_playback_period": (
                max(float(self.args.playback_period), 0.5)
                if float(self.args.playback_period) > 0.0
                else (
                    float(preview.arm_trajectory_frames[-1].time_from_start)
                    if preview.arm_trajectory_frames
                    else 0.0
                )
            ),
            "path_playback_rate_hz": max(float(self.args.playback_rate), 1.0),
            "planner_backend": str(self.args.planner_backend),
            "moveit_plan_service": str(self.args.moveit_plan_service),
            "moveit_planners": [
                token.strip()
                for token in str(self.args.moveit_planners).split(",")
                if token.strip()
            ],
            "ik_sampling": "moveit_goal_waypoints" if str(self.args.planner_backend) == "moveit" else "offline_reference_points_before_constraint_generation",
            "ik_mode": "moveit_ompl_motion_plan" if str(self.args.planner_backend) == "moveit" else "local_pose_ik_then_constrained_joint_generation",
            "joint_stream_limits": {
                "max_speed_rad_sec": max(float(self.args.preview_max_joint_speed), 0.01),
                "max_accel_rad_sec2": max(float(self.args.preview_max_joint_accel), 0.05),
                "max_jerk_rad_sec3": max(float(self.args.preview_max_joint_jerk), 0.1),
                "smoothing_tau_sec": max(float(self.args.preview_smoothing_tau), 0.02),
            },
            "ground_safety": {
                "ground_min_z_base": float(self.args.ground_min_z_base),
                "ground_clearance": float(self.args.ground_clearance),
                "tool_ground_clearance": float(self.args.tool_ground_clearance),
                "grasp_topdown_max_tilt_deg": float(self.args.grasp_topdown_max_tilt_deg),
            },
            "gripper_preview": {
                "gripper_type": str(self.args.gripper_type),
                "close_waypoint": str(self.args.gripper_close_waypoint),
                "open_waypoint": str(self.args.gripper_open_waypoint),
                "transition_progress": float(self.args.gripper_transition_progress),
                "close_progress": [
                    float(self.args.gripper_close_progress_start),
                    float(self.args.gripper_close_progress_end),
                ],
                "open_progress": [
                    float(self.args.gripper_open_progress_start),
                    float(self.args.gripper_open_progress_end),
                ],
            },
            "real_execution": {
                "enabled": bool(self.args.execute_real),
                "confirmed": str(self.args.execute_real_confirm).strip().upper() == "YES",
                "backend": str(self.args.real_execute_backend),
                "action": str(self.args.real_follow_action),
                "sdk_rpc": f"{self.args.real_sdk_ip}:{int(self.args.real_sdk_rpc_port)}",
                "sdk_move_speed_rad_sec": float(self.args.real_sdk_move_speed),
                "sdk_move_accel_rad_sec2": float(self.args.real_sdk_move_accel),
                "joint_states_topic": str(self.args.real_joint_states_topic),
                "arm_joint_names": self._real_arm_joint_names()
                if bool(self.args.execute_real)
                else list(REAL_ARM_JOINT_NAMES),
                "allow_partial": bool(self.args.real_allow_partial),
                "trajectory_point_stride": max(int(self.args.real_trajectory_point_stride), 1),
                "execute_gripper": bool(self.args.real_execute_gripper),
                "gripper_command_topic": str(self.args.real_gripper_command_topic),
            },
            "arm_trajectory_frame_count": len(preview.arm_trajectory_frames),
            "arm_trajectory_joint_angle_convention": "continuous_unwrapped",
            "arm_trajectory_duration_sec": (
                float(preview.arm_trajectory_frames[-1].time_from_start)
                if preview.arm_trajectory_frames
                else 0.0
            ),
            "arm_trajectory_frames": [
                {
                    "time_from_start": frame.time_from_start,
                    "positions": frame.positions,
                    "velocities": frame.velocities,
                    "accelerations": frame.accelerations,
                }
                for frame in preview.arm_trajectory_frames
            ],
            "depth_snapshot_policy": "snapshot_once_then_lock_until_restart",
            "snapshot_reason": preview.snapshot_reason,
            "snapshot_count": self.snapshot_count,
            "state": "PLAN_LOCKED" if self.inference_paused else "SEARCH_2D",
            "inference_paused": self.inference_paused,
            "restart_search_topic": self.args.restart_search_topic,
            "base_grasp_xyz": preview.base_grasp_xyz,
            "arm_joint_names": list(ARM_JOINT_NAMES),
            "arm_joint_path": [],
            "arm_joint_path_precomputed": False,
            "ik_message": self.preview_ik_message,
            "yolo_inference_count": self.inference_count,
            "basket_release_base": self.basket_release_base,
            "basket_keepout_min_base": self.basket_keepout_min,
            "basket_keepout_max_base": self.basket_keepout_max,
            "basket_safe": preview.basket_safe,
            "roi_points": len(preview.roi_points),
        }
        (self.save_dir / "latest_grasp_preview.json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )

    def _clear_markers(self, header: Header) -> None:
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        self.markers_pub.publish(MarkerArray(markers=[clear]))

    def _throttled_log(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_log_time > 1.5:
            self.last_log_time = now
            self.get_logger().info(message)


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = GraspPreviewNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
