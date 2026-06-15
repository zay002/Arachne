from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
from typing import Iterable

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Twist
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

try:
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
except ModuleNotFoundError:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "arachne_operator"
        if (candidate / "arachne_operator" / "real_hardware_acceptance_test.py").exists():
            sys.path.insert(0, str(candidate))
            break
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics


ARM_JOINTS = [
    "aubo_shoulder_joint",
    "aubo_upperArm_joint",
    "aubo_foreArm_joint",
    "aubo_wrist1_joint",
    "aubo_wrist2_joint",
    "aubo_wrist3_joint",
]

GRIPPER_JOINTS = ["ms42dc_left_finger_joint", "ms42dc_right_finger_joint"]

HOME = [-1.5707963267949, 0.201570428261868, 1.65970467002488, 0.485178041391533, 1.67675136677345, 0.76432946885334]
SCAN_CENTER = [-1.72, -0.44, 1.66, 0.92, 1.68, -0.05]
SCAN_LEFT = [-1.96, -0.48, 1.62, 0.98, 1.70, -0.26]
SCAN_RIGHT = [-1.48, -0.42, 1.70, 0.84, 1.66, 0.18]
GRASP_SEED = [1.20, -0.26, -1.26, 0.34, -1.44, 0.0]

ARM_MOUNT_XYZ = (0.22, 0.0, 0.155)
ARM_MOUNT_RPY = (0.0, 0.0, math.pi / 2.0)
TOOL_ADAPTER_RPY = (0.0, 0.0, math.pi / 4.0)
GRASP_FRAME_OFFSET_Z = 0.138691938
EE_CAMERA_XYZ = (0.025, -0.069, 0.03077)
EE_CAMERA_RPY = (0.0, -math.pi / 2.0, -math.pi / 2.0)

ROAD_LENGTH = 2.0
PATROL_SPEED = 0.10
REACH_X = (0.46, 0.96)
REACH_Y = (-0.55, 0.22)
BASKET_BASE = (0.545, 0.0, 0.20)


@dataclass(frozen=True)
class TrashSpec:
    name: str
    class_name: str
    taco_class: str
    odom_xyz: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]
    environment: str
    grasp_close: tuple[float, float]
    approach_height: float
    lift_height: float
    material: str = "mixed"
    grasp_style: str = "top_pinch"
    yaw: float = 0.0
    confidence: float = 0.91


@dataclass(frozen=True)
class Stage:
    label: str
    target: list[float]
    gripper: tuple[float, float]


class UrbanTrashSortingDemo(Node):
    """RViz-oriented urban trash sorting patrol demo.

    The demo intentionally mirrors the real pipeline at a semantic level:
    base patrol -> wrist-mounted scan -> synthetic YOLO lock -> ROI cloud ->
    class/environment grasp strategy -> MoveIt playback -> basket release.
    """

    def __init__(self) -> None:
        super().__init__("urban_trash_sorting_demo")
        self.declare_parameter("plan_service", "/plan_kinematic_path")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("playback_speed", 0.85)
        self.declare_parameter("loop", True)

        self.plan_client = self.create_client(
            GetMotionPlan, str(self.get_parameter("plan_service").value)
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_pub = self.create_publisher(JointState, "/arachne/grasp_preview/joint_states", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/arachne/urban_trash/markers", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/arachne/urban_trash/roi_cloud", 10)
        self.path_pub = self.create_publisher(PathMsg, "/arachne/urban_trash/base_path", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.planner_id = str(self.get_parameter("planner_id").value)
        self.playback_speed = max(float(self.get_parameter("playback_speed").value), 0.05)
        self.loop = bool(self.get_parameter("loop").value)

        self.kinematics = AuboI5Kinematics()
        self.base_from_aubo = self._transform(ARM_MOUNT_XYZ, ARM_MOUNT_RPY)
        self.aubo_from_base = self._invert_rigid(self.base_from_aubo)
        self.tool_to_grasp = self._transform((0.0, 0.0, GRASP_FRAME_OFFSET_Z), TOOL_ADAPTER_RPY)
        self.tool_to_camera = self._transform((0.0, 0.0, 0.0), TOOL_ADAPTER_RPY) @ self._transform(
            EE_CAMERA_XYZ, EE_CAMERA_RPY
        )
        self.grasp_rotation_base = self._grasp_frame_in_base(np.asarray(GRASP_SEED, dtype=float))[:3, :3]

        self.trash = self._make_trash_scene()
        self.collected: set[str] = set()
        self.failed: set[str] = set()
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.direction = 1.0
        self.mode = "patrol"
        self.scan_poses = [SCAN_LEFT, SCAN_CENTER, SCAN_RIGHT, SCAN_CENTER]
        self.scan_index = 1
        self.scan_started = self.get_clock().now()
        self.scan_duration = 1.15
        self.scan_from = list(SCAN_CENTER)
        self.scan_target = list(SCAN_CENTER)
        self.current_arm = list(SCAN_CENTER)
        self.current_gripper = (0.0, 0.0)
        self.candidate_name = ""
        self.candidate_count = 0
        self.locked: TrashSpec | None = None
        self.pipeline_note = "patrol: base moving, wrist camera scanning"
        self.strategy_note = "waiting for YOLO lock"
        self.samples: list[tuple[list[float], tuple[float, float], str]] = []
        self.sample_index = 0
        self.planning_thread: threading.Thread | None = None

        self.timer = self.create_timer(1.0 / 45.0, self._tick)
        self.get_logger().info(
            "Urban trash sorting demo ready: moving patrol + wrist-camera scan + synthetic YOLO/pointcloud + grasp"
        )

    def _make_trash_scene(self) -> list[TrashSpec]:
        return [
            TrashSpec("bottle_01", "plastic_bottle", "Clear plastic bottle", (0.72, -0.36, -0.22), (0.06, 0.06, 0.18), (0.1, 0.55, 1.0), "flat_ground", (0.58, -0.58), 0.13, 0.20, "PET/light", "body_clamp", math.radians(8.0), 0.93),
            TrashSpec("banana_01", "banana_peel", "Food waste", (1.18, -0.30, -0.22), (0.13, 0.045, 0.025), (1.0, 0.85, 0.08), "curb_edge", (0.45, -0.45), 0.10, 0.16, "soft/slippery", "soft_scoop", math.radians(-18.0), 0.88),
            TrashSpec("can_01", "can", "Drink can", (1.52, -0.42, -0.22), (0.065, 0.065, 0.11), (0.86, 0.86, 0.82), "flat_ground", (0.55, -0.55), 0.12, 0.18, "aluminum/rigid", "cylindrical_clamp", math.radians(11.0), 0.94),
            TrashSpec("newspaper_01", "curled_newspaper", "Normal paper", (0.95, 0.18, -0.22), (0.18, 0.07, 0.055), (0.92, 0.88, 0.72), "curb_edge", (0.50, -0.50), 0.11, 0.17, "paper/deformable", "wide_pinch", math.radians(24.0), 0.86),
            TrashSpec("battery_01", "battery_1", "Battery", (1.74, -0.20, -0.22), (0.045, 0.045, 0.13), (0.12, 0.12, 0.12), "gap_or_crevice", (0.62, -0.62), 0.14, 0.20, "dense/hazard", "vertical_pull", math.radians(-5.0), 0.90),
            TrashSpec("cup_01", "paper_cup", "Paper cup", (1.34, 0.10, -0.22), (0.075, 0.075, 0.10), (0.95, 0.95, 0.88), "flat_ground", (0.50, -0.50), 0.12, 0.17, "paper/light", "rim_clamp", math.radians(7.0), 0.89),
            TrashSpec("straw_01", "plastic_straw", "Plastic straw", (1.88, -0.47, -0.22), (0.16, 0.014, 0.014), (0.95, 0.12, 0.22), "gap_or_crevice", (0.42, -0.42), 0.13, 0.18, "plastic/thin", "edge_pick", math.radians(32.0), 0.84),
        ]

    def _odom_cb(self, msg: Odometry) -> None:
        self.base_x = float(msg.pose.pose.position.x)
        self.base_y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.orientation.z)
        w = float(msg.pose.pose.orientation.w)
        self.base_yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _tick(self) -> None:
        self._update_scan_pose()
        self._publish_markers()
        self._publish_base_path()
        self._publish_roi_cloud()

        if self.mode == "executing":
            self._publish_stop()
            self._play_execution()
            return
        if self.mode == "planning":
            self._publish_stop()
            self._publish_joint_state(self.current_arm, self.current_gripper)
            return

        self._patrol_step()
        self._publish_joint_state(self.current_arm, self.current_gripper)
        target = self._synthetic_yolo_scan()
        if target is not None:
            self.candidate_name = target.name
            self.candidate_count += 1
            self.pipeline_note = f"YOLO tracking {target.class_name}: {self.candidate_count}/3 frames"
            if self.candidate_count >= 3:
                self.locked = target
                self.mode = "planning"
                self.pipeline_note = f"locked {target.class_name}: stop base, crop ROI pointcloud"
                self.strategy_note = self._strategy_note(target)
                self._publish_stop()
                self.get_logger().info(
                    f"TACO segmentation lock while moving: {target.taco_class} at odom=({target.odom_xyz[0]:.2f},{target.odom_xyz[1]:.2f}) "
                    f"env={target.environment} material={target.material} strategy={target.grasp_style}; stopping for ROI cloud + grasp"
                )
                self.planning_thread = threading.Thread(target=self._plan_locked_target, daemon=True)
                self.planning_thread.start()
        else:
            self.candidate_count = 0
            self.candidate_name = ""
            self.pipeline_note = "patrol: base moving, wrist camera scanning"
            self.strategy_note = "waiting for YOLO lock"

    def _update_scan_pose(self) -> None:
        if self.mode != "patrol":
            return
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.scan_started.nanoseconds) * 1e-9
        if elapsed >= self.scan_duration:
            self.scan_index = (self.scan_index + 1) % len(self.scan_poses)
            self.scan_from = list(self.current_arm)
            self.scan_target = list(self.scan_poses[self.scan_index])
            self.scan_started = now
            elapsed = 0.0
        ratio = min(max(elapsed / self.scan_duration, 0.0), 1.0)
        eased = 0.5 - 0.5 * math.cos(math.pi * ratio)
        self.current_arm = [
            start + (target - start) * eased
            for start, target in zip(self.scan_from, self.scan_target)
        ]

    def _patrol_step(self) -> None:
        if self.base_x >= ROAD_LENGTH:
            self.direction = -1.0
        elif self.base_x <= 0.0:
            self.direction = 1.0
        msg = Twist()
        msg.linear.x = PATROL_SPEED * self.direction
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _synthetic_yolo_scan(self) -> TrashSpec | None:
        best: tuple[float, TrashSpec] | None = None
        scan_y_center = {
            0: -0.38,
            1: -0.14,
            2: 0.16,
            3: -0.14,
        }.get(self.scan_index, -0.14)
        for obj in self.trash:
            if obj.name in self.collected or obj.name in self.failed:
                continue
            base_xyz = self._odom_point_to_base(obj.odom_xyz)
            reachable = REACH_X[0] <= base_xyz[0] <= REACH_X[1] and REACH_Y[0] <= base_xyz[1] <= REACH_Y[1]
            in_scan_lane = abs(base_xyz[1] - scan_y_center) <= 0.25
            in_camera_depth = 0.30 <= base_xyz[0] <= 1.18
            if reachable and in_scan_lane and in_camera_depth:
                score = abs(base_xyz[1] - scan_y_center) + 0.15 * abs(base_xyz[0] - 0.72)
                if best is None or score < best[0]:
                    best = (score, obj)
        if best is None:
            return None
        return best[1]

    def _plan_locked_target(self) -> None:
        target = self.locked
        if target is None:
            self.mode = "patrol"
            return
        if not self.plan_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().error("MoveIt service unavailable; cannot execute urban trash demo")
            self.mode = "patrol"
            return
        self.pipeline_note = f"ROI extracted: {target.taco_class}, pointcloud -> grasp pose"
        self.strategy_note = self._strategy_note(target)
        stages = self._make_grasp_stages(target)
        if stages is None:
            self.get_logger().warning(f"Skipping {target.name}: no valid grasp plan")
            self.failed.add(target.name)
            self.pipeline_note = f"failed: {target.taco_class}, continue patrol"
            self.mode = "patrol"
            return
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        current = list(stages[0].target)
        for stage in stages[1:]:
            if stage.label in {"close", "drop_open"}:
                samples.extend(self._hold_samples(current, stage.gripper, stage.label, 0.6))
                continue
            trajectory = self._request_joint_plan(current, stage.target, stage.label)
            if trajectory is None:
                self.get_logger().error(f"MoveIt failed at {stage.label}; aborting target {target.name}")
                self.failed.add(target.name)
                self.pipeline_note = f"planning failed at {stage.label}: skip {target.taco_class}"
                self.mode = "patrol"
                return
            samples.extend(self._trajectory_samples(trajectory, stage.gripper, stage.label))
            current = list(stage.target)
        self.samples = samples
        self.sample_index = 0
        self.mode = "executing"
        self.pipeline_note = f"executing: grasp {target.taco_class} -> basket"
        self.get_logger().info(
            f"Executing {target.taco_class}: TACO mask -> ROI cloud -> strategy -> MoveIt ({len(samples)} frames)"
        )

    def _make_grasp_stages(self, obj: TrashSpec) -> list[Stage] | None:
        approach, grasp, lift = self._grasp_points_base(obj)
        q_current = np.asarray(self.current_arm, dtype=float)
        stages = [Stage("scan_lock", list(q_current), (0.0, 0.0))]
        for label, point, gripper in [
            ("approach", approach, (0.0, 0.0)),
            ("grasp", grasp, (0.0, 0.0)),
            ("lift", lift, obj.grasp_close),
        ]:
            solved = self._solve_grasp_frame_target(q_current, point, label)
            if solved is None:
                return None
            q_current = solved
            stages.append(Stage(label, [float(v) for v in q_current], gripper))
            if label == "grasp":
                stages.append(Stage("close", [float(v) for v in q_current], obj.grasp_close))
        release = self._solve_release_frame_target(q_current, BASKET_BASE)
        if release is None:
            return None
        q_current = release
        stages.append(Stage("basket_over", [float(v) for v in q_current], obj.grasp_close))
        stages.append(Stage("drop_open", [float(v) for v in q_current], (0.0, 0.0)))
        stages.append(Stage("resume_scan", list(SCAN_CENTER), (0.0, 0.0)))
        return stages

    def _grasp_points_base(
        self, obj: TrashSpec
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        base_xyz = self._odom_point_to_base(obj.odom_xyz)
        grasp_z = base_xyz[2] + 0.06
        y_bias = 0.0
        x_backoff = 0.04
        if obj.environment == "curb_edge":
            y_bias = -0.025
            x_backoff = 0.055
        elif obj.environment == "gap_or_crevice":
            y_bias = 0.018
            x_backoff = 0.025
            grasp_z += 0.018
        if obj.grasp_style in {"soft_scoop", "wide_pinch"}:
            grasp_z += 0.012
        elif obj.grasp_style == "edge_pick":
            y_bias += 0.018
            grasp_z += 0.02

        grasp = (base_xyz[0], base_xyz[1] + y_bias, grasp_z)
        approach = (grasp[0] - x_backoff, grasp[1], grasp[2] + obj.approach_height)
        lift = (grasp[0], grasp[1], grasp[2] + obj.lift_height)
        return approach, grasp, lift

    def _strategy_note(self, obj: TrashSpec) -> str:
        return (
            f"TACO={obj.taco_class}; material={obj.material}; "
            f"{obj.environment}; grasp={obj.grasp_style}; close={obj.grasp_close[0]:.2f}"
        )

    def _solve_grasp_frame_target(
        self, q_start: np.ndarray, point_base: tuple[float, float, float], label: str
    ) -> np.ndarray | None:
        grasp_in_base = np.eye(4, dtype=np.float64)
        grasp_in_base[:3, :3] = self.grasp_rotation_base
        grasp_in_base[:3, 3] = np.asarray(point_base, dtype=float)
        tool_in_aubo = self.aubo_from_base @ (grasp_in_base @ self._invert_rigid(self.tool_to_grasp))
        ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
            np.asarray(q_start, dtype=float),
            tool_in_aubo,
            position_tolerance=0.018,
            orientation_tolerance=0.45,
            damping=0.08,
            max_iterations=260,
            max_step=0.10,
            orientation_weight=0.22,
        )
        q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
        achieved = self._grasp_frame_in_base(q_goal)
        error = float(np.linalg.norm(achieved[:3, 3] - np.asarray(point_base, dtype=float)))
        self.get_logger().info(f"IK {label}: target={self._fmt(point_base)} err={error:.3f}m ori={orientation_error:.3f}rad")
        if not ok or error > 0.035:
            return None
        return q_goal

    def _solve_release_frame_target(
        self, q_start: np.ndarray, point_base: tuple[float, float, float]
    ) -> np.ndarray | None:
        current_rotation = self._grasp_frame_in_base(q_start)[:3, :3]
        best: tuple[float, np.ndarray, float] | None = None
        for yaw in (0.0, math.radians(8.0), math.radians(-8.0)):
            grasp_in_base = np.eye(4, dtype=np.float64)
            grasp_in_base[:3, :3] = self._orthonormalize_rotation(current_rotation @ self._rpy_matrix(0.0, 0.0, yaw))
            grasp_in_base[:3, 3] = np.asarray(point_base, dtype=float)
            tool_in_aubo = self.aubo_from_base @ (grasp_in_base @ self._invert_rigid(self.tool_to_grasp))
            ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
                np.asarray(q_start, dtype=float),
                tool_in_aubo,
                position_tolerance=0.025,
                orientation_tolerance=0.70,
                damping=0.08,
                max_iterations=300,
                max_step=0.08,
                orientation_weight=0.12,
            )
            q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
            achieved = self._grasp_frame_in_base(q_goal)
            error = float(np.linalg.norm(achieved[:3, 3] - np.asarray(point_base, dtype=float)))
            cost = self._release_motion_cost(q_goal, q_start)
            score = 160.0 * error + 2.0 * float(orientation_error) + cost
            if ok and error <= 0.04 and (best is None or score < best[0]):
                best = (score, q_goal, error)
        if best is None:
            return None
        delta = np.abs(self._joint_delta(best[1], q_start))
        self.get_logger().info(
            f"release IK: target={self._fmt(point_base)} err={best[2]:.3f}m "
            f"shoulder_elbow_delta={np.linalg.norm(delta[:3]):.3f}rad wrist_delta={np.linalg.norm(delta[3:]):.3f}rad"
        )
        return best[1]

    def _request_joint_plan(
        self, start: list[float], target: list[float], label: str
    ) -> JointTrajectory | None:
        request = GetMotionPlan.Request()
        motion = MotionPlanRequest()
        motion.group_name = "aubo_arm"
        motion.pipeline_id = "ompl"
        motion.planner_id = self.planner_id
        motion.num_planning_attempts = 6
        motion.allowed_planning_time = 3.0
        motion.max_velocity_scaling_factor = 0.32
        motion.max_acceleration_scaling_factor = 0.32
        motion.start_state = RobotState()
        motion.start_state.joint_state.name = list(ARM_JOINTS)
        motion.start_state.joint_state.position = list(start)
        constraints = Constraints()
        constraints.name = f"urban_{label}_joint_goal"
        for name, value in zip(ARM_JOINTS, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.012
            joint.tolerance_below = 0.012
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        motion.goal_constraints.append(constraints)
        request.motion_plan_request = motion
        future = self.plan_client.call_async(request)
        deadline = time.monotonic() + 7.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return None
        response = future.result().motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(f"MoveIt failed for {label}: error_code={response.error_code.val}")
            return None
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            return None
        self.get_logger().info(f"MoveIt planned {label}: {len(trajectory.points)} points")
        return trajectory

    def _play_execution(self) -> None:
        if not self.samples:
            self._finish_execution()
            return
        joints, gripper, _label = self.samples[self.sample_index]
        self.current_arm = list(joints)
        self.current_gripper = gripper
        self._publish_joint_state(joints, gripper)
        self.sample_index += 1
        if self.sample_index >= len(self.samples):
            self._finish_execution()

    def _finish_execution(self) -> None:
        if self.locked is not None:
            self.collected.add(self.locked.name)
        self.locked = None
        self.samples = []
        self.sample_index = 0
        self.mode = "patrol"
        self.scan_index = 1
        self.scan_from = list(self.current_arm)
        self.scan_target = list(SCAN_CENTER)
        self.scan_started = self.get_clock().now()
        self.current_gripper = (0.0, 0.0)
        self.candidate_count = 0

    def _trajectory_samples(
        self, trajectory: JointTrajectory, gripper: tuple[float, float], label: str
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        index_by_name = {name: i for i, name in enumerate(trajectory.joint_names)}
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        last_positions: list[float] | None = None
        for point in trajectory.points:
            positions = list(last_positions or self.current_arm)
            for joint_index, joint_name in enumerate(ARM_JOINTS):
                source_index = index_by_name.get(joint_name)
                if source_index is not None and source_index < len(point.positions):
                    positions[joint_index] = float(point.positions[source_index])
            samples.append((positions, gripper, label))
            last_positions = positions
        return self._densify(samples, label)

    def _densify(
        self, samples: list[tuple[list[float], tuple[float, float], str]], label: str
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        if len(samples) < 2:
            return samples
        dense: list[tuple[list[float], tuple[float, float], str]] = []
        for previous, current in zip(samples, samples[1:]):
            a, gripper, _ = previous
            b, _, _ = current
            steps = max(int(max(abs(x - y) for x, y in zip(a, b)) / 0.018 / self.playback_speed), 1)
            dense.extend(self._interpolate(a, b, gripper, label, steps))
        dense.append(samples[-1])
        return dense

    def _hold_samples(
        self, joints: list[float], gripper: tuple[float, float], label: str, seconds: float
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        return [(list(joints), gripper, label) for _ in range(max(int(seconds * 45.0), 1))]

    def _interpolate(
        self,
        start: Iterable[float],
        target: Iterable[float],
        gripper: tuple[float, float],
        label: str,
        steps: int,
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        a = list(start)
        b = list(target)
        output = []
        for i in range(max(steps, 1)):
            t = (i + 1) / max(steps, 1)
            eased = 0.5 - 0.5 * math.cos(math.pi * t)
            output.append(([x + (y - x) * eased for x, y in zip(a, b)], gripper, label))
        return output

    def _publish_joint_state(self, arm: list[float], gripper: tuple[float, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
        msg.position = list(arm) + [float(gripper[0]), float(gripper[1])]
        self.joint_pub.publish(msg)

    def _publish_markers(self) -> None:
        markers = []
        markers.extend(self._scene_markers())
        markers.extend(self._scan_markers())
        target = self.locked or self._object_by_name(self.candidate_name)
        if target is not None:
            markers.extend(self._target_markers(target))
            markers.extend(self._segmentation_markers(target))
            markers.extend(self._grasp_plan_markers(target))
        markers.extend(self._pipeline_markers())
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _scene_markers(self) -> list[Marker]:
        markers = [
            self._box(1, "road", (1.0, 0.0, -0.225), (2.3, 0.92, 0.01), 0.0, self._color(0.13, 0.14, 0.15, 0.72)),
            self._box(2, "curb", (1.0, 0.42, -0.15), (2.3, 0.12, 0.16), 0.0, self._color(0.48, 0.50, 0.47, 0.75)),
            self._box(3, "front_basket_keepout", (self.base_x + 0.5435, 0.0, -0.030), (0.204, 0.180, 0.087), self.base_yaw, self._color(1.0, 0.35, 0.05, 0.20)),
            self._box(4, "rear_rack_keepout", (self.base_x - 0.160, 0.0, 0.416), (0.274, 0.329, 0.622), self.base_yaw + math.pi / 2.0, self._color(1.0, 0.05, 0.05, 0.13)),
            self._box(5, "sidewalk", (1.0, 0.62, -0.105), (2.3, 0.28, 0.07), 0.0, self._color(0.32, 0.33, 0.31, 0.55)),
            self._box(6, "lane_marking", (1.0, -0.02, -0.218), (2.0, 0.018, 0.006), 0.0, self._color(1.0, 0.92, 0.20, 0.60)),
            self._box(7, "curb_gap", (1.78, -0.31, -0.214), (0.30, 0.035, 0.012), 0.0, self._color(0.02, 0.02, 0.02, 0.85)),
        ]
        markers.extend(self._base_outline_markers())
        for index, obj in enumerate(self.trash, start=20):
            if obj.name in self.collected:
                continue
            markers.append(self._trash_marker(index, obj))
        return markers

    def _scan_markers(self) -> list[Marker]:
        camera = self._camera_pose_odom()
        origin = camera[:3, 3]
        forward = camera[:3, 0]
        right = camera[:3, 1]
        up = -camera[:3, 2]
        near = origin + forward * 0.10
        far = origin + forward * 1.05
        half_w = 0.36
        half_h = 0.24
        corners = [
            far + right * half_w + up * half_h,
            far - right * half_w + up * half_h,
            far - right * half_w - up * half_h,
            far + right * half_w - up * half_h,
        ]
        marker = self._base_marker(70, "ee_camera_frustum")
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.01
        marker.color = self._color(0.0, 0.75, 1.0, 0.9)
        for corner in corners:
            marker.points.extend([self._point(origin), self._point(corner)])
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            marker.points.extend([self._point(corners[a]), self._point(corners[b])])
        axis = self._base_marker(72, "ee_camera_center_ray")
        axis.type = Marker.LINE_LIST
        axis.scale.x = 0.018
        axis.color = self._color(0.1, 1.0, 0.25, 0.95)
        axis.points.extend([self._point(origin), self._point(origin + forward * 0.75)])
        label = self._text(71, f"{self.mode}\nscan_{self.scan_index}", tuple(near + np.array([0.0, 0.0, 0.16])))
        return [marker, axis, label]

    def _base_outline_markers(self) -> list[Marker]:
        chassis = self._box(
            12,
            "base_visual_fallback",
            (self.base_x, self.base_y, -0.075),
            (0.93, 0.62, 0.11),
            self.base_yaw,
            self._color(0.08, 0.18, 0.22, 0.18),
        )
        chassis.type = Marker.CUBE
        outline = self._base_marker(13, "base_visual_fallback")
        outline.type = Marker.LINE_STRIP
        outline.scale.x = 0.018
        outline.color = self._color(0.0, 0.85, 1.0, 0.85)
        half_x = 0.465
        half_y = 0.31
        c = math.cos(self.base_yaw)
        s = math.sin(self.base_yaw)
        for lx, ly in [
            (half_x, half_y),
            (-half_x, half_y),
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
        ]:
            outline.points.append(
                self._point(np.array([self.base_x + c * lx - s * ly, self.base_y + s * lx + c * ly, -0.005]))
            )
        heading = self._base_marker(14, "base_visual_fallback")
        heading.type = Marker.ARROW
        heading.scale.x = 0.035
        heading.scale.y = 0.075
        heading.scale.z = 0.075
        heading.color = self._color(0.0, 0.95, 1.0, 0.78)
        heading.points.extend(
            [
                self._point(np.array([self.base_x, self.base_y, 0.02])),
                self._point(np.array([self.base_x + c * 0.42, self.base_y + s * 0.42, 0.02])),
            ]
        )
        return [chassis, outline, heading]

    def _target_markers(self, obj: TrashSpec) -> list[Marker]:
        base_xyz = self._odom_point_to_base(obj.odom_xyz)
        text = (
            f"TACO mask {obj.taco_class} {obj.confidence:.2f}\n"
            f"{obj.material} | {obj.environment}\n"
            f"{obj.grasp_style} base=({base_xyz[0]:.2f},{base_xyz[1]:.2f})"
        )
        return [self._text(90, text, (obj.odom_xyz[0], obj.odom_xyz[1], obj.odom_xyz[2] + 0.24))]

    def _segmentation_markers(self, obj: TrashSpec) -> list[Marker]:
        bbox = self._box(100, "taco_bbox", obj.odom_xyz, obj.size, obj.yaw, self._color(0.0, 0.85, 1.0, 0.16))
        bbox.type = Marker.CUBE
        mask = self._base_marker(101, "taco_mask_contour")
        mask.type = Marker.LINE_STRIP
        mask.scale.x = 0.012
        mask.color = self._color(0.0, 0.95, 1.0, 0.95)
        cx, cy, cz = obj.odom_xyz
        sx, sy, sz = obj.size
        c = math.cos(obj.yaw)
        s = math.sin(obj.yaw)
        for i in range(33):
            a = 2.0 * math.pi * i / 32.0
            lx = math.cos(a) * sx * 0.62
            ly = math.sin(a) * sy * 0.62
            mask.points.append(self._point(np.array([cx + c * lx - s * ly, cy + s * lx + c * ly, cz + sz * 0.56])))
        centroid = self._sphere(102, "roi_centroid", obj.odom_xyz, 0.035, self._color(0.0, 1.0, 0.45, 0.95))
        return [bbox, mask, centroid]

    def _grasp_plan_markers(self, obj: TrashSpec) -> list[Marker]:
        approach, grasp, lift = self._grasp_points_base(obj)
        odom_points = [self._base_point_to_odom(p) for p in (approach, grasp, lift, BASKET_BASE)]
        path = self._base_marker(110, "grasp_strategy_path")
        path.type = Marker.LINE_STRIP
        path.scale.x = 0.018
        path.color = self._color(0.1, 1.0, 0.35, 0.92)
        for point in odom_points:
            path.points.append(self._point(np.asarray(point, dtype=float)))
        labels = [
            self._sphere(111, "approach_point", odom_points[0], 0.035, self._color(0.2, 0.8, 1.0, 0.9)),
            self._sphere(112, "grasp_point", odom_points[1], 0.040, self._color(0.0, 1.0, 0.35, 0.95)),
            self._sphere(113, "lift_point", odom_points[2], 0.032, self._color(1.0, 0.9, 0.1, 0.9)),
        ]
        return [path] + labels

    def _pipeline_markers(self) -> list[Marker]:
        text = (
            f"{self.pipeline_note}\n"
            f"{self.strategy_note}\n"
            f"done={len(self.collected)} failed={len(self.failed)} base_x={self.base_x:.2f}"
        )
        return [self._text(150, text, (self.base_x + 0.18, -0.68, 0.34))]

    def _publish_roi_cloud(self) -> None:
        target = self.locked or self._object_by_name(self.candidate_name) or self._synthetic_yolo_scan()
        if target is None:
            return
        points = self._roi_points(target)
        header = self._header()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _roi_points(self, obj: TrashSpec) -> list[tuple[float, float, float]]:
        cx, cy, cz = obj.odom_xyz
        sx, sy, sz = obj.size
        points: list[tuple[float, float, float]] = []
        for i in range(42):
            a = (i * 2.399963229728653) % (2.0 * math.pi)
            r = 0.35 + 0.65 * ((i % 7) / 6.0)
            x = cx + math.cos(a) * sx * 0.5 * r
            y = cy + math.sin(a) * sy * 0.5 * r
            z = cz + sz * ((i % 5) / 5.0)
            points.append((x, y, z))
        for i in range(18):
            x = cx + (i - 9) * 0.018
            y = cy - sy * 0.65
            points.append((x, y, cz - 0.002))
        if obj.environment != "flat_ground":
            for i in range(16):
                points.append((cx + (i - 8) * 0.012, cy + 0.055, cz + 0.055))
        if obj.environment == "gap_or_crevice":
            for i in range(12):
                points.append((cx + (i - 6) * 0.01, cy + 0.025, cz - 0.025))
        return points

    def _publish_base_path(self) -> None:
        msg = PathMsg()
        msg.header = self._header()
        for x in np.linspace(0.0, ROAD_LENGTH, 24):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _trash_marker(self, marker_id: int, obj: TrashSpec) -> Marker:
        alpha = 0.36 if obj.name in self.failed else 0.92
        marker = self._box(marker_id, obj.class_name, obj.odom_xyz, obj.size, obj.yaw, self._color(*obj.color, alpha))
        if obj.class_name in {"plastic_bottle", "can", "battery_1", "paper_cup"}:
            marker.type = Marker.CYLINDER
        elif obj.class_name == "banana_peel":
            marker.type = Marker.SPHERE
        return marker

    def _box(
        self,
        marker_id: int,
        ns: str,
        center: tuple[float, float, float],
        size: tuple[float, float, float],
        yaw: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, ns)
        marker.type = Marker.CUBE
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.color = color
        return marker

    def _sphere(
        self,
        marker_id: int,
        ns: str,
        center: tuple[float, float, float],
        diameter: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, ns)
        marker.type = Marker.SPHERE
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = color
        return marker

    def _text(self, marker_id: int, text: str, xyz: tuple[float, float, float]) -> Marker:
        marker = self._base_marker(marker_id, "status_label")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.scale.z = 0.06
        marker.color = self._color(0.96, 0.96, 0.96, 0.95)
        marker.text = text
        return marker

    def _base_marker(self, marker_id: int, ns: str) -> Marker:
        marker = Marker()
        marker.header = self._header()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=1)
        marker.pose.orientation.w = 1.0
        return marker

    def _header(self):
        from std_msgs.msg import Header

        header = Header()
        header.frame_id = "odom"
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _camera_pose_odom(self) -> np.ndarray:
        base = self._transform((self.base_x, self.base_y, 0.0), (0.0, 0.0, self.base_yaw))
        tool_in_base = self.base_from_aubo @ self.kinematics.fk(np.asarray(self.current_arm, dtype=float))
        return base @ tool_in_base @ self.tool_to_camera

    def _object_by_name(self, name: str) -> TrashSpec | None:
        if not name:
            return None
        for obj in self.trash:
            if obj.name == name and obj.name not in self.collected and obj.name not in self.failed:
                return obj
        return None

    def _odom_point_to_base(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        dx = point[0] - self.base_x
        dy = point[1] - self.base_y
        c = math.cos(-self.base_yaw)
        s = math.sin(-self.base_yaw)
        return (c * dx - s * dy, s * dx + c * dy, point[2])

    def _base_point_to_odom(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        c = math.cos(self.base_yaw)
        s = math.sin(self.base_yaw)
        return (
            self.base_x + c * point[0] - s * point[1],
            self.base_y + s * point[0] + c * point[1],
            point[2],
        )

    def _grasp_frame_in_base(self, joints: np.ndarray) -> np.ndarray:
        return self.base_from_aubo @ self.kinematics.fk(np.asarray(joints, dtype=float)) @ self.tool_to_grasp

    def _release_motion_cost(self, target: np.ndarray, start: np.ndarray) -> float:
        delta = np.abs(self._joint_delta(target, start))
        weights = np.asarray([4.0, 4.0, 3.2, 0.35, 0.35, 0.35], dtype=float)
        return float(np.linalg.norm(delta * weights)) + 4.0 * float(np.linalg.norm(delta[:3])) + 0.15 * float(np.linalg.norm(delta[3:]))

    def _joint_delta(self, target: np.ndarray, start: np.ndarray) -> np.ndarray:
        raw = np.asarray(target, dtype=float) - np.asarray(start, dtype=float)
        return np.arctan2(np.sin(raw), np.cos(raw))

    def _transform(
        self, xyz: tuple[float, float, float], rpy: tuple[float, float, float]
    ) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = self._rpy_matrix(*rpy)
        transform[:3, 3] = np.asarray(xyz, dtype=float)
        return transform

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

    def _invert_rigid(self, transform: np.ndarray) -> np.ndarray:
        inverse = np.eye(4, dtype=np.float64)
        rotation = np.asarray(transform[:3, :3], dtype=np.float64)
        translation = np.asarray(transform[:3, 3], dtype=np.float64)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def _orthonormalize_rotation(self, rotation: np.ndarray) -> np.ndarray:
        u, _s, vh = np.linalg.svd(np.asarray(rotation, dtype=np.float64))
        output = u @ vh
        if np.linalg.det(output) < 0.0:
            u[:, -1] *= -1.0
            output = u @ vh
        return output

    def _point(self, xyz: np.ndarray) -> Point:
        point = Point()
        point.x = float(xyz[0])
        point.y = float(xyz[1])
        point.z = float(xyz[2])
        return point

    def _color(self, r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

    def _fmt(self, xyz: tuple[float, float, float]) -> str:
        return f"({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UrbanTrashSortingDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
