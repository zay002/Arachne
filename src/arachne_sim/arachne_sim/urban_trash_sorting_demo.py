from __future__ import annotations

import ast
from dataclasses import dataclass
import math
from pathlib import Path
import random
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
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

try:
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
    from arachne_operator.grasp_geometry import pointcloud_grasp_geometry
except ModuleNotFoundError:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "arachne_operator"
        if (candidate / "arachne_operator" / "real_hardware_acceptance_test.py").exists():
            sys.path.insert(0, str(candidate))
            break
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
    from arachne_operator.grasp_geometry import pointcloud_grasp_geometry


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
# tool0 vertical down; basket_over at basket center +20cm z, safe_mid = basket_over +36cm x +12cm z.
FIXED_SAFE_MID = [-1.392228627, -0.587456810, 1.402798238, 0.420158124, 1.570706911, 0.178573568]
FIXED_BASKET_OVER = [-1.187131238, -0.087444694, 2.606213310, 1.122582998, 1.570733434, 0.383692391]

ARM_MOUNT_XYZ = (0.22, 0.0, 0.105)
ARM_MOUNT_RPY = (0.0, 0.0, math.pi / 2.0)
TOOL_ADAPTER_RPY = (0.0, 0.0, math.pi / 4.0)
GRASP_FRAME_OFFSET_Z = 0.143691938
EE_CAMERA_XYZ = (0.0, -0.0741, 0.005)
EE_CAMERA_RPY = (0.0, -math.pi / 2.0, -math.pi / 2.0)

DEFAULT_PATROL_DISTANCE_M = 1.2
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
        self.declare_parameter("use_moveit", True)
        self.declare_parameter("playback_speed", 0.85)
        self.declare_parameter("loop", True)
        self.declare_parameter("patrol_pattern", "line")
        self.declare_parameter("patrol_distance_m", DEFAULT_PATROL_DISTANCE_M)
        self.declare_parameter("patrol_box_width_m", 1.0)
        self.declare_parameter("patrol_box_height_m", 1.2)
        self.declare_parameter("patrol_entry_m", 0.3)
        self.declare_parameter("show_keepout_markers", False)
        self.declare_parameter("slam_map_yaml", "")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("trash_seed", 26)
        self.declare_parameter("trash_count", 10)
        self.declare_parameter("synthetic_grasp_benchmark", False)
        self.declare_parameter("synthetic_grasp_trials", 60)
        self.declare_parameter("synthetic_ground_z_m", -0.22)
        self.declare_parameter("scan_arc_radius_m", 0.32)
        self.declare_parameter("scan_arc_angle_deg", 72.0)
        self.declare_parameter("scan_arc_samples", 9)
        self.declare_parameter("scan_cycle_duration_sec", 4.2)
        self.declare_parameter("detection_lock_frames", 1)
        self.declare_parameter("fixed_safe_mid_joints", ",".join(str(v) for v in FIXED_SAFE_MID))
        self.declare_parameter("fixed_basket_over_joints", ",".join(str(v) for v in FIXED_BASKET_OVER))
        self.declare_parameter("fixed_search_joints", "")

        self.plan_client = self.create_client(
            GetMotionPlan, str(self.get_parameter("plan_service").value)
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_pub = self.create_publisher(JointState, "/arachne/grasp_preview/joint_states", 10)
        self.display_joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/arachne/urban_trash/markers", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/arachne/urban_trash/roi_cloud", 10)
        self.path_pub = self.create_publisher(PathMsg, "/arachne/urban_trash/base_path", 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_service(Trigger, "/arachne/urban_trash/return_home", self._return_home_cb)

        self.planner_id = str(self.get_parameter("planner_id").value)
        self.use_moveit = bool(self.get_parameter("use_moveit").value)
        self.playback_speed = max(float(self.get_parameter("playback_speed").value), 0.05)
        self.loop = bool(self.get_parameter("loop").value)
        self.patrol_pattern = str(self.get_parameter("patrol_pattern").value).strip().lower()
        self.road_length = max(float(self.get_parameter("patrol_distance_m").value), 0.2)
        self.show_keepout_markers = bool(self.get_parameter("show_keepout_markers").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.map_origin = (0.0, 0.0)
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_data: list[int] = []
        self.map_msg = self._load_slam_map(str(self.get_parameter("slam_map_yaml").value).strip())
        benchmark_value = self.get_parameter("synthetic_grasp_benchmark").value
        self.synthetic_grasp_benchmark = (
            str(benchmark_value).strip().lower() in {"1", "true", "yes", "on"}
            if isinstance(benchmark_value, str)
            else bool(benchmark_value)
        )
        self.synthetic_ground_z = float(self.get_parameter("synthetic_ground_z_m").value)

        self.kinematics = AuboI5Kinematics()
        self.base_from_aubo = self._transform(ARM_MOUNT_XYZ, ARM_MOUNT_RPY)
        self.aubo_from_base = self._invert_rigid(self.base_from_aubo)
        self.tool_to_grasp = self._transform((0.0, 0.0, GRASP_FRAME_OFFSET_Z), TOOL_ADAPTER_RPY)
        self.tool_to_camera = self._transform((0.0, 0.0, 0.0), TOOL_ADAPTER_RPY) @ self._transform(
            EE_CAMERA_XYZ, EE_CAMERA_RPY
        )
        self.grasp_rotation_base = self._grasp_frame_in_base(np.asarray(GRASP_SEED, dtype=float))[
            :3, :3
        ] @ self._rpy_matrix(0.0, 0.0, -math.pi / 4.0)

        self.patrol_loop_start_index = 0
        self.patrol_waypoints = self._make_patrol_waypoints()
        self.patrol_index = 1 if len(self.patrol_waypoints) > 1 else 0
        self.trash = self._make_trash_scene()
        self.collected: set[str] = set()
        self.failed: set[str] = set()
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.direction = 1.0
        self.mode = "benchmark" if self.synthetic_grasp_benchmark else "patrol"
        self.scan_poses, self.scan_camera_arc_points_base = self._make_camera_scan_arc()
        self.scan_center_index = min(len(self.scan_poses) // 2, len(self.scan_poses) - 1)
        self.scan_center_joints = list(self.scan_poses[self.scan_center_index])
        self.fixed_safe_mid_joints = self._joint_param("fixed_safe_mid_joints", FIXED_SAFE_MID)
        self.fixed_basket_over_joints = self._joint_param("fixed_basket_over_joints", FIXED_BASKET_OVER)
        self.fixed_search_joints = self._joint_param("fixed_search_joints", self.scan_center_joints)
        self.scan_index = self.scan_center_index
        self.scan_started = self.get_clock().now()
        self.scan_cycle_duration = max(float(self.get_parameter("scan_cycle_duration_sec").value), 1.2)
        self.current_arm = list(self.scan_center_joints)
        self.current_gripper = (0.0, 0.0)
        self.candidate_name = ""
        self.candidate_count = 0
        self.detection_lock_frames = max(int(self.get_parameter("detection_lock_frames").value), 1)
        self.locked: TrashSpec | None = None
        self.pipeline_note = "patrol: base moving, wrist camera scanning"
        self.strategy_note = "waiting for YOLO lock"
        self.camera_detection: tuple[TrashSpec, tuple[int, int, int, int]] | None = None
        self.benchmark_records: list[dict[str, object]] = []
        self.benchmark_running = False
        self.benchmark_2d_ms: list[float] = []
        self.benchmark_3d_ms: list[float] = []
        self.samples: list[tuple[list[float], tuple[float, float], str]] = []
        self.sample_index = 0
        self.planning_thread: threading.Thread | None = None
        self.return_requested = False
        self.return_generation = 0
        self.return_arm_from = list(self.current_arm)
        self.return_started = self.get_clock().now()
        self.return_arm_duration = 2.4
        if self.synthetic_grasp_benchmark:
            self.mode = "benchmark"
            self.locked = self.trash[0] if self.trash else None
            self.pipeline_note = "benchmark: random 16x8x6cm cuboid ROI clouds"
            self.strategy_note = "running 20 direct geometric grasp solves"
            self.planning_thread = threading.Thread(target=self._run_synthetic_grasp_benchmark, daemon=True)
            self.planning_thread.start()

        self.timer = self.create_timer(1.0 / 45.0, self._tick)
        self.create_timer(1.0, self._publish_slam_map)
        self.get_logger().info(
            f"Urban trash sorting demo ready: {self.patrol_pattern} cruise ({len(self.patrol_waypoints)} waypoints) + "
            f"wrist-camera scan + random TACO trash ({len(self.trash)} objects); "
            f"synthetic_grasp_benchmark={self.synthetic_grasp_benchmark}"
        )

    def _return_home_cb(self, _request, response):
        self.return_requested = True
        self.return_generation += 1
        self.locked = None
        self.samples = []
        self.sample_index = 0
        self.mode = "return_home"
        self.return_arm_from = list(self.current_arm)
        self.return_started = self.get_clock().now()
        self.current_gripper = (0.0, 0.0)
        self.candidate_count = 0
        self.candidate_name = ""
        self.pipeline_note = "return: base to start, gripper open, arm to scan midpoint"
        self.strategy_note = "return_home requested"
        self._publish_stop()
        response.success = True
        response.message = "return_home started"
        return response

    def _joint_param(self, name: str, default: list[float]) -> list[float]:
        raw = str(self.get_parameter(name).value).strip()
        if not raw:
            return list(default)
        values = [float(token) for token in raw.replace(",", " ").split()]
        if len(values) != 6:
            raise ValueError(f"{name} must contain 6 joint values")
        return values

    def _make_camera_scan_arc(self) -> tuple[list[list[float]], list[tuple[float, float, float]]]:
        radius = max(float(self.get_parameter("scan_arc_radius_m").value), 0.06)
        max_angle = math.radians(max(min(float(self.get_parameter("scan_arc_angle_deg").value), 85.0), 8.0))
        samples = max(int(self.get_parameter("scan_arc_samples").value), 5)
        if samples % 2 == 0:
            samples += 1

        center_q = np.asarray(SCAN_CENTER, dtype=float)
        center_camera = self._camera_pose_base(center_q)
        center_position = np.asarray(center_camera[:3, 3], dtype=float)
        center_rotation = np.asarray(center_camera[:3, :3], dtype=float)
        pivot = center_position + np.array([-radius, 0.0, 0.0], dtype=float)

        joints: list[list[float]] = []
        camera_points: list[tuple[float, float, float]] = []
        q_seed = center_q.copy()
        for theta in np.linspace(-max_angle, max_angle, samples):
            position = pivot + np.array([radius * math.cos(float(theta)), radius * math.sin(float(theta)), 0.0])
            position[2] = center_position[2]
            camera_target = np.eye(4, dtype=np.float64)
            camera_target[:3, :3] = center_rotation
            camera_target[:3, 3] = position
            tool_target = self.aubo_from_base @ (camera_target @ self._invert_rigid(self.tool_to_camera))
            ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
                q_seed,
                tool_target,
                position_tolerance=0.012,
                orientation_tolerance=0.18,
                damping=0.07,
                max_iterations=240,
                max_step=0.07,
                orientation_weight=0.28,
            )
            q_goal = q_seed + self._joint_delta(q_solution, q_seed)
            achieved = self._camera_pose_base(q_goal)
            achieved_error = float(np.linalg.norm(achieved[:3, 3] - position))
            if not ok or achieved_error > 0.025 or orientation_error > 0.35:
                self.get_logger().warning(
                    "camera arc IK failed; falling back to joint-space arc "
                    f"(err={achieved_error:.3f}m ori={orientation_error:.3f}rad)"
                )
                return self._fallback_scan_arc(samples)
            joints.append([float(v) for v in q_goal])
            camera_points.append((float(position[0]), float(position[1]), float(position[2])))
            q_seed = q_goal

        z_values = [point[2] for point in camera_points]
        z_span = max(z_values) - min(z_values) if z_values else 0.0
        self.get_logger().info(
            f"Camera scan arc ready: radius={radius:.2f}m angle={math.degrees(max_angle):.1f}deg "
            f"keypoints={len(joints)} z_span={z_span * 1000.0:.1f}mm"
        )
        return joints, camera_points

    def _fallback_scan_arc(self, samples: int) -> tuple[list[list[float]], list[tuple[float, float, float]]]:
        joints: list[list[float]] = []
        camera_points: list[tuple[float, float, float]] = []
        left = np.asarray(SCAN_LEFT, dtype=float)
        center = np.asarray(SCAN_CENTER, dtype=float)
        right = np.asarray(SCAN_RIGHT, dtype=float)
        for t in np.linspace(0.0, 1.0, max(samples, 5)):
            if t <= 0.5:
                q = left + self._joint_delta(center, left) * (0.5 - 0.5 * math.cos(math.pi * t * 2.0))
            else:
                q = center + self._joint_delta(right, center) * (0.5 - 0.5 * math.cos(math.pi * (t - 0.5) * 2.0))
            joints.append([float(value) for value in q])
            camera = self._camera_pose_base(q)
            camera_points.append(tuple(float(value) for value in camera[:3, 3]))
        return joints, camera_points

    def _make_trash_scene(self) -> list[TrashSpec]:
        if self.synthetic_grasp_benchmark:
            return self._make_synthetic_grasp_trials()
        templates = [
            ("plastic_bottle", "Clear plastic bottle", (0.06, 0.06, 0.18), (0.1, 0.55, 1.0), "flat_ground", (0.58, -0.58), 0.13, 0.20, "PET/light", "body_clamp", 0.93),
            ("banana_peel", "Food waste", (0.13, 0.045, 0.025), (1.0, 0.85, 0.08), "curb_edge", (0.45, -0.45), 0.10, 0.16, "soft/slippery", "soft_scoop", 0.88),
            ("can", "Drink can", (0.065, 0.065, 0.11), (0.86, 0.86, 0.82), "flat_ground", (0.55, -0.55), 0.12, 0.18, "aluminum/rigid", "cylindrical_clamp", 0.94),
            ("curled_newspaper", "Normal paper", (0.18, 0.07, 0.055), (0.92, 0.88, 0.72), "curb_edge", (0.50, -0.50), 0.11, 0.17, "paper/deformable", "wide_pinch", 0.86),
            ("battery_1", "Battery", (0.045, 0.045, 0.13), (0.12, 0.12, 0.12), "gap_or_crevice", (0.62, -0.62), 0.14, 0.20, "dense/hazard", "vertical_pull", 0.90),
            ("paper_cup", "Paper cup", (0.075, 0.075, 0.10), (0.95, 0.95, 0.88), "flat_ground", (0.50, -0.50), 0.12, 0.17, "paper/light", "rim_clamp", 0.89),
            ("plastic_straw", "Plastic straw", (0.16, 0.014, 0.014), (0.95, 0.12, 0.22), "gap_or_crevice", (0.42, -0.42), 0.13, 0.18, "plastic/thin", "edge_pick", 0.84),
        ]
        rng = random.Random(int(self.get_parameter("trash_seed").value))
        count = max(int(self.get_parameter("trash_count").value), 1)
        segments = self._patrol_segments()
        trash: list[TrashSpec] = []
        for index in range(count):
            class_name, taco_class, size, color, environment, close, approach, lift, material, style, confidence = templates[index % len(templates)]
            x, y, yaw = self._random_trash_pose_near_patrol(rng, segments)
            trash.append(
                TrashSpec(
                    f"{class_name}_{index + 1:02d}",
                    class_name,
                    taco_class,
                    (x, y, -0.22),
                    size,
                    color,
                    environment,
                    close,
                    approach,
                    lift,
                    material,
                    style,
                    yaw,
                    confidence,
                )
            )
        return trash

    def _make_synthetic_grasp_trials(self) -> list[TrashSpec]:
        rng = random.Random(int(self.get_parameter("trash_seed").value))
        count = max(int(self.get_parameter("synthetic_grasp_trials").value), 1)
        trash: list[TrashSpec] = []
        size = (0.16, 0.08, 0.06)
        for index in range(count):
            zone = rng.choices(("front", "left", "right"), weights=(0.50, 0.25, 0.25), k=1)[0]
            if zone == "front":
                x = rng.uniform(0.34, 1.20)
                y = rng.uniform(-0.68, 0.48)
            elif zone == "left":
                x = rng.uniform(0.20, 1.08)
                y = rng.uniform(0.24, 0.86)
            else:
                x = rng.uniform(0.20, 1.08)
                y = rng.uniform(-0.86, -0.24)
            if index < 3:
                x = 0.52 + 0.12 * index
                y = -0.10 + 0.10 * index
            bottom_z = self.synthetic_ground_z + rng.uniform(0.0, 0.03)
            trash.append(
                TrashSpec(
                    f"synthetic_box_{index + 1:02d}",
                    "synthetic_box",
                    "Synthetic 16x8x6cm cuboid",
                    (x, y, bottom_z + size[2] * 0.5),
                    size,
                    (0.15, 0.72, 1.0),
                    "flat_ground",
                    (0.52, -0.52),
                    0.08,
                    0.12,
                    "synthetic/random-cloud",
                    "top_pinch",
                    rng.uniform(-math.pi, math.pi),
                    1.0,
                )
            )
        return trash

    def _make_patrol_waypoints(self) -> list[tuple[float, float]]:
        if self.patrol_pattern in {"line", "straight"}:
            self.patrol_loop_start_index = 0
            return [(0.0, 0.0), (self.road_length, 0.0)]
        if self.patrol_pattern in {"box_entry", "rectangle_entry", "real_box"}:
            return self._box_entry_patrol_waypoints()
        if self.patrol_pattern in {"local", "local_loop"}:
            return self._local_patrol_waypoints()

        fallback = self._local_patrol_waypoints()
        if not self.map_data or self.map_width <= 0 or self.map_height <= 0:
            return fallback

        stride = max(self.road_length, 0.9)
        margin = 0.40
        min_x, max_x, min_y, max_y = self._free_map_bounds()
        xs = np.arange(min_x + margin, max_x - margin, stride)
        ys = np.arange(min_y + margin, max_y - margin, stride)
        candidates: dict[tuple[int, int], tuple[float, float]] = {}
        for yi, y in enumerate(ys):
            for xi, x in enumerate(xs):
                if self._map_is_clear(float(x), float(y), 0.28):
                    candidates[(xi, yi)] = (float(x), float(y))
        if len(candidates) < 3:
            return fallback

        adjacency: dict[tuple[int, int], list[tuple[int, int]]] = {key: [] for key in candidates}
        for key, point in candidates.items():
            xi, yi = key
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                neighbor = (xi + dx, yi + dy)
                if neighbor in candidates and self._line_is_clear(point, candidates[neighbor], 0.22):
                    adjacency[key].append(neighbor)
            adjacency[key].sort(key=lambda item: (item[1], item[0]))

        start = min(candidates, key=lambda key: math.hypot(candidates[key][0], candidates[key][1]))
        route_keys = self._dfs_coverage_route(start, adjacency)
        route = [candidates[key] for key in route_keys]
        if len(route) < 3:
            return fallback
        self.get_logger().info(
            f"Generated map coverage patrol: {len(route)} route points over {len(set(route_keys))} free samples"
        )
        return route

    def _patrol_segments(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        if len(self.patrol_waypoints) < 2:
            return [((0.0, 0.0), (self.road_length, 0.0))]
        segments = list(zip(self.patrol_waypoints, self.patrol_waypoints[1:]))
        loop_start = min(max(int(self.patrol_loop_start_index), 0), len(self.patrol_waypoints) - 1)
        if loop_start < len(self.patrol_waypoints) - 1:
            segments.append((self.patrol_waypoints[-1], self.patrol_waypoints[loop_start]))
        return segments

    def _box_entry_patrol_waypoints(self) -> list[tuple[float, float]]:
        width = max(float(self.get_parameter("patrol_box_width_m").value), 0.2)
        height = max(float(self.get_parameter("patrol_box_height_m").value), 0.2)
        entry = max(float(self.get_parameter("patrol_entry_m").value), 0.0)
        half_width = width * 0.5
        bottom_x = entry
        top_x = entry + height
        route = [
            (0.0, 0.0),
            (bottom_x, 0.0),
            (bottom_x, half_width),
            (top_x, half_width),
            (top_x, -half_width),
            (bottom_x, -half_width),
        ]
        self.patrol_loop_start_index = 2
        self.get_logger().info(
            f"Generated box-entry patrol: entry={entry:.2f}m box={width:.2f}x{height:.2f}m "
            f"route={len(route)} waypoints"
        )
        return route

    def _local_patrol_waypoints(self) -> list[tuple[float, float]]:
        step = self.road_length
        return [
            (0.0, 0.0),
            (step, 0.0),
            (step, step * 0.85),
            (0.0, step * 0.85),
            (-step * 0.75, step * 0.45),
            (-step * 0.75, -step * 0.45),
            (0.0, -step * 0.70),
            (step * 0.75, -step * 0.45),
        ]

    def _dfs_coverage_route(
        self,
        start: tuple[int, int],
        adjacency: dict[tuple[int, int], list[tuple[int, int]]],
    ) -> list[tuple[int, int]]:
        route = [start]
        visited = {start}
        stack: list[tuple[tuple[int, int], int]] = [(start, 0)]
        while stack:
            current, next_index = stack[-1]
            neighbors = adjacency.get(current, [])
            if next_index >= len(neighbors):
                stack.pop()
                if stack:
                    route.append(stack[-1][0])
                continue
            neighbor = neighbors[next_index]
            stack[-1] = (current, next_index + 1)
            if neighbor in visited:
                continue
            visited.add(neighbor)
            route.append(neighbor)
            stack.append((neighbor, 0))
        return route

    def _free_map_bounds(self) -> tuple[float, float, float, float]:
        xs: list[float] = []
        ys: list[float] = []
        for row in range(self.map_height):
            for col in range(self.map_width):
                if self.map_data[row * self.map_width + col] != 0:
                    continue
                xs.append(self.map_origin[0] + (col + 0.5) * self.map_resolution)
                ys.append(self.map_origin[1] + (row + 0.5) * self.map_resolution)
        if not xs:
            return (-self.road_length, self.road_length, -self.road_length, self.road_length)
        return (min(xs), max(xs), min(ys), max(ys))

    def _random_trash_pose_near_patrol(
        self,
        rng: random.Random,
        segments: list[tuple[tuple[float, float], tuple[float, float]]],
    ) -> tuple[float, float, float]:
        for _attempt in range(40):
            a, b = rng.choice(segments)
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length = max(math.hypot(dx, dy), 1e-6)
            ux = dx / length
            uy = dy / length
            nx = -uy
            ny = ux
            t = rng.uniform(0.18, 0.82)
            lateral = rng.choice([-1.0, 1.0]) * rng.uniform(0.32, 0.54)
            x = a[0] + dx * t + nx * lateral
            y = a[1] + dy * t + ny * lateral
            if self._map_is_free(x, y):
                return (x, y, rng.uniform(-math.pi, math.pi))
        a, b = rng.choice(segments)
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5 - 0.42, rng.uniform(-math.pi, math.pi))

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

        if self.mode == "benchmark":
            self._publish_stop()
            self._publish_joint_state(self.current_arm, self.current_gripper)
            return
        if self.mode == "return_home":
            self._return_home_step()
            self._publish_joint_state(self.current_arm, self.current_gripper)
            return
        if self.mode == "executing":
            self._publish_stop()
            self._play_execution()
            return
        if self.mode == "planning":
            self._publish_stop()
            self._publish_joint_state(self.current_arm, self.current_gripper)
            return
        if self.mode == "home":
            self._publish_stop()
            self.current_gripper = (0.0, 0.0)
            self._publish_joint_state(self.current_arm, self.current_gripper)
            return

        self._patrol_step()
        self._publish_joint_state(self.current_arm, self.current_gripper)
        target = self._synthetic_yolo_scan()
        if target is not None:
            self.candidate_name = target.name
            self.candidate_count += 1
            self.pipeline_note = f"camera YOLO tracking {target.class_name}: {self.candidate_count}/{self.detection_lock_frames} frames"
            if self.candidate_count >= self.detection_lock_frames:
                self.locked = target
                self.mode = "planning"
                self.pipeline_note = f"locked {target.class_name}: brake scan/base, crop ROI pointcloud"
                self.strategy_note = self._strategy_note(target)
                self._publish_stop()
                self.get_logger().info(
                    f"camera YOLO lock while moving: {target.taco_class} at odom=({target.odom_xyz[0]:.2f},{target.odom_xyz[1]:.2f}) "
                    f"env={target.environment} material={target.material} strategy={target.grasp_style}; stopping for ROI cloud + grasp"
                )
                self.planning_thread = threading.Thread(target=self._plan_locked_target, daemon=True)
                self.planning_thread.start()
        else:
            self.candidate_count = 0
            self.candidate_name = ""
            self.pipeline_note = "patrol: base moving, wrist camera scanning"
            self.strategy_note = "waiting for YOLO lock"

    def _return_home_step(self) -> None:
        self.current_gripper = (0.0, 0.0)
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.return_started.nanoseconds) * 1e-9
        ratio = min(max(elapsed / self.return_arm_duration, 0.0), 1.0)
        eased = 0.5 - 0.5 * math.cos(math.pi * ratio)
        self.current_arm = self._interpolate_joint_pose(
            self.return_arm_from,
            self.scan_center_joints,
            eased,
        )

        dx = -self.base_x
        dy = -self.base_y
        distance = math.hypot(dx, dy)
        if distance <= 0.05:
            self._publish_stop()
            self.base_x = 0.0 if abs(self.base_x) < 0.02 else self.base_x
            self.base_y = 0.0 if abs(self.base_y) < 0.02 else self.base_y
            if ratio >= 1.0:
                self.mode = "home"
                self.return_requested = False
                self.current_arm = list(self.scan_center_joints)
                self.scan_index = self.scan_center_index
                self.pipeline_note = "home: base at start, gripper open, arm at scan midpoint"
                self.strategy_note = "ready to restart patrol"
            return

        msg = Twist()
        if self.patrol_pattern in {"line", "straight"}:
            # ponytail: straight-road return is reverse gear, not a turn-around route.
            msg.linear.x = -min(0.12, max(0.035, abs(self.base_x) * 0.45))
            msg.angular.z = max(min(-1.2 * self.base_yaw - 0.8 * self.base_y, 0.35), -0.35)
        else:
            desired_yaw = math.atan2(dy, dx)
            yaw_error = self._normalize_angle(desired_yaw - self.base_yaw)
            msg.angular.z = max(min(1.8 * yaw_error, 0.70), -0.70)
            if abs(yaw_error) < 1.15:
                msg.linear.x = min(0.12, max(0.035, distance * 0.45)) * max(0.12, 1.0 - abs(yaw_error) / 1.15)
        self.cmd_pub.publish(msg)

    def _update_scan_pose(self) -> None:
        if self.mode != "patrol":
            return
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.scan_started.nanoseconds) * 1e-9
        phase = (elapsed / self.scan_cycle_duration) % 1.0
        sweep = 0.5 + 0.5 * math.sin(2.0 * math.pi * phase)
        scaled = sweep * (len(self.scan_poses) - 1)
        lower = min(int(math.floor(scaled)), len(self.scan_poses) - 2)
        upper = lower + 1
        ratio = scaled - lower
        self.scan_index = lower if ratio < 0.5 else upper
        self.current_arm = self._interpolate_joint_pose(
            self.scan_poses[lower],
            self.scan_poses[upper],
            ratio,
        )

    def _interpolate_joint_pose(self, start: list[float], target: list[float], ratio: float) -> list[float]:
        t = min(max(float(ratio), 0.0), 1.0)
        a = np.asarray(start, dtype=float)
        delta = self._joint_delta(np.asarray(target, dtype=float), a)
        return [float(v) for v in a + delta * t]

    def _patrol_step(self) -> None:
        if not self.patrol_waypoints:
            self._publish_stop()
            return
        target = self.patrol_waypoints[self.patrol_index]
        dx = target[0] - self.base_x
        dy = target[1] - self.base_y
        distance = math.hypot(dx, dy)
        if distance < 0.12:
            if self.patrol_index >= len(self.patrol_waypoints) - 1:
                if self.patrol_pattern in {"line", "straight"} or not self.loop:
                    self.mode = "return_home"
                    self.return_arm_from = list(self.current_arm)
                    self.return_started = self.get_clock().now()
                    self.pipeline_note = "return: base to start, scanning disabled"
                    self.strategy_note = "forward line complete"
                    self._publish_stop()
                    return
                self.patrol_index = min(self.patrol_loop_start_index, len(self.patrol_waypoints) - 1)
            else:
                self.patrol_index += 1
            target = self.patrol_waypoints[self.patrol_index]
            dx = target[0] - self.base_x
            dy = target[1] - self.base_y
            distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw - self.base_yaw)
        msg = Twist()
        msg.angular.z = max(min(1.7 * yaw_error, 0.65), -0.65)
        if abs(yaw_error) < 1.0:
            msg.linear.x = PATROL_SPEED * max(0.15, 1.0 - abs(yaw_error) / 1.0)
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _synthetic_yolo_scan(self) -> TrashSpec | None:
        best: tuple[float, TrashSpec, tuple[int, int, int, int]] | None = None
        self.camera_detection = None
        for obj in self.trash:
            if obj.name in self.collected or obj.name in self.failed:
                continue
            bbox = self._camera_bbox(obj)
            if bbox is None:
                continue
            base_xyz = self._odom_point_to_base(obj.odom_xyz)
            reachable = REACH_X[0] <= base_xyz[0] <= REACH_X[1] and REACH_Y[0] <= base_xyz[1] <= REACH_Y[1]
            if reachable:
                cx = (bbox[0] + bbox[2]) * 0.5
                area = max((bbox[2] - bbox[0]) * (bbox[3] - bbox[1]), 1)
                score = abs(cx - 320.0) - 0.002 * area
                if best is None or score < best[0]:
                    best = (score, obj, bbox)
        if best is None:
            return None
        self.camera_detection = (best[1], best[2])
        return best[1]

    def _plan_locked_target(self) -> None:
        return_generation = self.return_generation
        target = self.locked
        if target is None:
            self.mode = "patrol"
            return
        if self._return_interrupted(return_generation):
            return
        if not self.use_moveit:
            stages = self._make_grasp_stages(target)
            if stages is None:
                self.get_logger().warning(f"Skipping {target.name}: no valid sim grasp plan")
                self.failed.add(target.name)
                self.pipeline_note = f"failed: {target.taco_class}, continue patrol"
                self.mode = "patrol"
                return
            self.samples = self._sim_action_samples(stages)
            self.sample_index = 0
            self.mode = "executing"
            self.pipeline_note = f"executing: grasp {target.taco_class} -> fixed basket"
            self.strategy_note = "sim action: grasp IK, fixed safe_mid/basket_over"
            self.get_logger().info(
                f"Executing {target.taco_class}: sim action stages ({len(self.samples)} frames)"
            )
            return
        if not self.plan_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().error("MoveIt service unavailable; cannot execute urban trash demo")
            self.mode = "patrol"
            return
        if self._return_interrupted(return_generation):
            return
        self.pipeline_note = f"ROI extracted: {target.taco_class}, pointcloud -> grasp pose"
        self.strategy_note = self._strategy_note(target)
        stages = self._make_grasp_stages(target)
        if self._return_interrupted(return_generation):
            return
        if stages is None:
            self.get_logger().warning(f"Skipping {target.name}: no valid grasp plan")
            self.failed.add(target.name)
            self.pipeline_note = f"failed: {target.taco_class}, continue patrol"
            self.mode = "patrol"
            return
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        current = list(stages[0].target)
        for stage in stages[1:]:
            if self._return_interrupted(return_generation):
                return
            if stage.label in {"close", "drop_open"}:
                samples.extend(self._hold_samples(current, stage.gripper, stage.label, 0.6))
                continue
            trajectory = self._request_joint_plan(current, stage.target, stage.label)
            if self._return_interrupted(return_generation):
                return
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
        if self._return_interrupted(return_generation):
            return
        self.mode = "executing"
        self.pipeline_note = f"executing: grasp {target.taco_class} -> basket"
        self.get_logger().info(
            f"Executing {target.taco_class}: TACO mask -> ROI cloud -> strategy -> MoveIt ({len(samples)} frames)"
        )

    def _run_synthetic_grasp_benchmark(self) -> None:
        self.benchmark_running = True
        successes = 0
        timings: list[float] = []
        trials = max(int(self.get_parameter("synthetic_grasp_trials").value), 1)
        while rclpy.ok():
            for obj in self.trash[:trials]:
                self.locked = obj
                self.current_arm = list(self.scan_center_joints)
                t0 = time.perf_counter()
                self._synthetic_2d_detection(obj)
                t1 = time.perf_counter()
                cloud_base = self._roi_points_base(obj)
                t2 = time.perf_counter()
                stages = self._make_fast_grasp_stages(obj, cloud_base)
                t3 = time.perf_counter()
                self.benchmark_2d_ms.append((t1 - t0) * 1000.0)
                self.benchmark_3d_ms.append((t2 - t1) * 1000.0)
                elapsed_ms = (t3 - t2) * 1000.0
                ok = stages is not None
                successes += int(ok)
                timings.append(elapsed_ms)
                self.benchmark_records.append({"name": obj.name, "ok": ok, "ms": elapsed_ms})
                if ok and stages:
                    self._execute_sim_grasp_stages(stages, obj)
                self.strategy_note = (
                    f"benchmark {len(self.benchmark_records)}/{len(self.trash)} "
                    f"success={successes} last={elapsed_ms:.1f}ms"
                )
                time.sleep(0.05)
            if not self.loop:
                break
            self.collected.clear()
            self.failed.clear()
            self.benchmark_records.clear()
            self.locked = self.trash[0] if self.trash else None
        avg = sum(timings) / max(len(timings), 1)
        avg_2d = sum(self.benchmark_2d_ms) / max(len(self.benchmark_2d_ms), 1)
        avg_3d = sum(self.benchmark_3d_ms) / max(len(self.benchmark_3d_ms), 1)
        self.pipeline_note = f"oracle benchmark complete: {successes}/{len(timings)} grasp poses"
        self.strategy_note = f"2D={avg_2d:.1f}ms 3D={avg_3d:.1f}ms pose={avg:.1f}ms; sim-only motion"
        self.get_logger().info(
            f"Synthetic grasp oracle benchmark complete: {successes}/{len(timings)} poses, "
            f"2d_avg={avg_2d:.2f}ms 3d_avg={avg_3d:.2f}ms "
            f"pose_only_avg={avg:.2f}ms max={max(timings, default=0.0):.2f}ms; not real end-to-end timing"
        )
        self.benchmark_running = False

    def _synthetic_2d_detection(self, obj: TrashSpec) -> tuple[float, float, float, float]:
        cx, cy, _cz = obj.odom_xyz
        sx, sy, _sz = obj.size
        scale = 420.0
        return (
            320.0 + (cx - sx * 0.5) * scale,
            240.0 + (cy - sy * 0.5) * scale,
            320.0 + (cx + sx * 0.5) * scale,
            240.0 + (cy + sy * 0.5) * scale,
        )

    def _make_fast_grasp_stages(
        self, obj: TrashSpec, cloud_base: list[tuple[float, float, float]] | None = None
    ) -> list[Stage] | None:
        result = pointcloud_grasp_geometry(cloud_base or self._roi_points_base(obj), reach_x=REACH_X, reach_y=REACH_Y)
        if result is None or not result.reachable:
            return None
        _approach, grasp, _lift = self._grasp_points_base(obj)
        start = list(self.current_arm)
        reach = self._sim_grasp_joint_target(start, grasp)
        return [
            Stage("scan_lock", start, (0.0, 0.0)),
            Stage("grasp", reach, (0.0, 0.0)),
            Stage("close", reach, obj.grasp_close),
            Stage("safe_mid", list(self.fixed_safe_mid_joints), obj.grasp_close),
            Stage("basket_over", list(self.fixed_basket_over_joints), obj.grasp_close),
            Stage("drop_open", list(self.fixed_basket_over_joints), (0.0, 0.0)),
            Stage("resume_scan", list(self.fixed_search_joints), (0.0, 0.0)),
        ]

    def _sim_grasp_joint_target(
        self, start: list[float], grasp: tuple[float, float, float]
    ) -> list[float]:
        reach = list(start)
        reach[0] += max(min(grasp[1] * 0.42, 0.26), -0.26)
        reach[1] -= max(min((grasp[0] - 0.55) * 0.42, 0.20), -0.18)
        reach[2] += max(min((grasp[0] - 0.70) * 0.35, 0.18), -0.18)
        reach[3] -= 0.22
        reach[5] += max(min(grasp[1] * 0.55, 0.30), -0.30)
        return reach

    def _execute_sim_grasp_stages(self, stages: list[Stage], obj: TrashSpec) -> None:
        self.samples = self._sim_action_samples(stages)
        self.sample_index = 0
        self.mode = "executing"
        self.pipeline_note = f"sim action: grasp {obj.taco_class}"
        self.strategy_note = "same stages as real executor; sim publishes joint_states"
        while rclpy.ok() and self.samples:
            time.sleep(0.02)
        self.mode = "benchmark"

    def _sim_action_samples(self, stages: list[Stage]) -> list[tuple[list[float], tuple[float, float], str]]:
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        current = list(stages[0].target)
        for stage in stages[1:]:
            if max(abs(a - b) for a, b in zip(current, stage.target)) < 1e-6:
                samples.extend(self._hold_samples(current, stage.gripper, stage.label, 0.28))
            else:
                steps = 72 if stage.label != "resume_scan" else 84
                samples.extend(self._interpolate(current, stage.target, stage.gripper, stage.label, steps))
                current = list(stage.target)
        return samples

    def _return_interrupted(self, generation: int) -> bool:
        return (
            self.return_requested
            or generation != self.return_generation
            or self.mode in {"return_home", "home"}
        )

    def _make_grasp_stages(self, obj: TrashSpec) -> list[Stage] | None:
        _approach, grasp, _lift = self._grasp_points_base(obj)
        q_current = np.asarray(self.current_arm, dtype=float)
        stages = [Stage("scan_lock", list(q_current), (0.0, 0.0))]
        solved = self._solve_grasp_frame_target(q_current, grasp, "grasp")
        if solved is None:
            return None
        q_current = solved
        stages.append(Stage("grasp", [float(v) for v in q_current], (0.0, 0.0)))
        stages.append(Stage("close", [float(v) for v in q_current], obj.grasp_close))
        stages.append(Stage("safe_mid", list(self.fixed_safe_mid_joints), obj.grasp_close))
        stages.append(Stage("basket_over", list(self.fixed_basket_over_joints), obj.grasp_close))
        stages.append(Stage("drop_open", list(self.fixed_basket_over_joints), (0.0, 0.0)))
        stages.append(Stage("resume_scan", list(self.fixed_search_joints), (0.0, 0.0)))
        return stages

    def _grasp_points_base(
        self, obj: TrashSpec
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        if self.synthetic_grasp_benchmark or obj.class_name == "synthetic_box":
            return self._cloud_oracle_grasp_points_base(obj)
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

    def _cloud_oracle_grasp_points_base(
        self, obj: TrashSpec
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        result = pointcloud_grasp_geometry(self._roi_points_base(obj), reach_x=REACH_X, reach_y=REACH_Y)
        if result is None:
            base_xyz = self._odom_point_to_base(obj.odom_xyz)
            return (base_xyz, base_xyz, base_xyz)
        return result.approach, result.grasp, result.lift

    def _roi_points_base(self, obj: TrashSpec) -> list[tuple[float, float, float]]:
        return [self._odom_point_to_base(point) for point in self._roi_points(obj)]

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
            max_iterations=90,
            max_step=0.14,
            orientation_weight=0.16,
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
                max_iterations=110,
                max_step=0.12,
                orientation_weight=0.10,
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
        self.mode = "benchmark" if self.synthetic_grasp_benchmark else "patrol"
        self.scan_index = self.scan_center_index
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
        self.display_joint_pub.publish(msg)

    def _publish_markers(self) -> None:
        markers = []
        markers.extend(self._scene_markers())
        markers.extend(self._scan_markers())
        target = self.locked or self._object_by_name(self.candidate_name)
        if target is not None:
            markers.extend(self._target_markers(target))
            markers.extend(self._segmentation_markers(target))
            markers.extend(self._grasp_plan_markers(target))
        markers.extend(self._carried_markers())
        markers.extend(self._pipeline_markers())
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _carried_markers(self) -> list[Marker]:
        if self.locked is None or max(abs(value) for value in self.current_gripper) <= 1e-3:
            return []
        grasp = self._grasp_frame_in_base(np.asarray(self.current_arm, dtype=float))
        position = grasp[:3, 3]
        return [
            self._box(
                19,
                "carried_object",
                (float(position[0]), float(position[1]), float(position[2])),
                self.locked.size,
                0.0,
                self._color(0.1, 1.0, 0.35, 0.95),
            )
        ]

    def _scene_markers(self) -> list[Marker]:
        markers: list[Marker] = []
        if self.show_keepout_markers:
            markers.extend(
                [
                    self._box(3, "front_basket_keepout", (self.base_x + 0.5435, 0.0, -0.030), (0.204, 0.180, 0.087), self.base_yaw, self._color(1.0, 0.35, 0.05, 0.20)),
                    self._box(4, "rear_rack_keepout", (self.base_x - 0.160, 0.0, 0.416), (0.274, 0.329, 0.622), self.base_yaw + math.pi / 2.0, self._color(1.0, 0.05, 0.05, 0.13)),
                ]
            )
        markers.extend(self._base_outline_markers())
        for index, obj in enumerate(self.trash, start=20):
            if obj.name in self.collected:
                continue
            if self.locked is not None and obj.name == self.locked.name and max(abs(value) for value in self.current_gripper) > 1e-3:
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
        arc = self._base_marker(73, "ee_camera_scan_arc")
        arc.type = Marker.LINE_STRIP
        arc.scale.x = 0.014
        arc.color = self._color(1.0, 0.78, 0.12, 0.92)
        for point in self.scan_camera_arc_points_base:
            arc.points.append(self._point(np.asarray(self._base_point_to_odom(point), dtype=float)))
        label = self._text(71, f"{self.mode}\nscan_{self.scan_index}", tuple(near + np.array([0.0, 0.0, 0.16])))
        return [marker, axis, arc, label]

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
            f"done={len(self.collected)} failed={len(self.failed)} "
            f"wp={self.patrol_index + 1}/{len(self.patrol_waypoints)} "
            f"base=({self.base_x:.2f},{self.base_y:.2f})"
        )
        return [self._text(150, text, (self.base_x + 0.18, self.base_y - 0.68, 0.34))]

    def _publish_roi_cloud(self) -> None:
        target = self.locked or self._object_by_name(self.candidate_name)
        if target is None:
            return
        points = self._roi_points(target)
        header = self._header()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _camera_bbox(self, obj: TrashSpec) -> tuple[int, int, int, int] | None:
        camera_from_odom = self._invert_rigid(self._camera_pose_odom())
        cx, cy, cz = obj.odom_xyz
        sx, sy, sz = obj.size
        c = math.cos(obj.yaw)
        s = math.sin(obj.yaw)
        pixels: list[tuple[float, float]] = []
        for lx in (-sx * 0.5, sx * 0.5):
            for ly in (-sy * 0.5, sy * 0.5):
                for lz in (-sz * 0.5, sz * 0.5):
                    odom = np.array([cx + c * lx - s * ly, cy + s * lx + c * ly, cz + lz, 1.0])
                    cam = camera_from_odom @ odom
                    depth = float(cam[0])
                    if depth <= 0.15 or depth > 1.15:
                        continue
                    pixels.append((320.0 + 360.0 * float(cam[1]) / depth, 180.0 + 360.0 * float(cam[2]) / depth))
        if not pixels:
            return None
        x0 = max(int(min(x for x, _ in pixels)), 0)
        y0 = max(int(min(y for _, y in pixels)), 0)
        x1 = min(int(max(x for x, _ in pixels)), 639)
        y1 = min(int(max(y for _, y in pixels)), 359)
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        return (x0, y0, x1, y1)

    def _roi_points(self, obj: TrashSpec) -> list[tuple[float, float, float]]:
        if self.synthetic_grasp_benchmark or obj.class_name == "synthetic_box":
            rng = random.Random(f"{obj.name}:{int(self.get_parameter('trash_seed').value)}")
            cx, cy, cz = obj.odom_xyz
            sx, sy, sz = obj.size
            c = math.cos(obj.yaw)
            s = math.sin(obj.yaw)
            points: list[tuple[float, float, float]] = []
            for _ in range(360):
                lx = rng.uniform(-sx * 0.5, sx * 0.5)
                ly = rng.uniform(-sy * 0.5, sy * 0.5)
                lz = rng.uniform(-sz * 0.5, sz * 0.5)
                points.append((cx + c * lx - s * ly, cy + s * lx + c * ly, cz + lz))
            return points
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
        path_points = list(self.patrol_waypoints)
        if path_points and self.patrol_loop_start_index <= len(path_points) - 1:
            path_points.append(path_points[self.patrol_loop_start_index])
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _publish_slam_map(self) -> None:
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def _load_slam_map(self, yaml_path: str) -> OccupancyGrid | None:
        if not yaml_path:
            return None
        path = Path(yaml_path).expanduser()
        if not path.exists():
            self.get_logger().warning(f"SLAM map yaml not found: {path}")
            return None
        try:
            meta = self._read_simple_yaml(path)
            image_path = Path(str(meta.get("image", "")))
            if not image_path.is_absolute():
                image_path = path.parent / image_path
            width, height, pixels = self._read_pgm(image_path)
            resolution = float(meta.get("resolution", 0.05))
            origin = list(meta.get("origin", [0.0, 0.0, 0.0]))
            negate = int(meta.get("negate", 0))
            occupied_thresh = float(meta.get("occupied_thresh", 0.65))
            free_thresh = float(meta.get("free_thresh", 0.25))
        except (OSError, ValueError, SyntaxError) as exc:
            self.get_logger().warning(f"failed to load SLAM map {path}: {exc}")
            return None

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame_id
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.orientation.w = 1.0
        data: list[int] = []
        for y in range(height):
            row_start = (height - 1 - y) * width
            for x in range(width):
                color = pixels[row_start + x]
                if negate:
                    occ = color / 255.0
                else:
                    occ = (255 - color) / 255.0
                if occ > occupied_thresh:
                    data.append(100)
                elif occ < free_thresh:
                    data.append(0)
                else:
                    data.append(-1)
        grid.data = data
        self.map_origin = (float(origin[0]), float(origin[1]))
        self.map_resolution = resolution
        self.map_width = width
        self.map_height = height
        self.map_data = data
        self.get_logger().info(f"Loaded SLAM map for sim: {path} ({width}x{height}, {resolution:.3f}m)")
        return grid

    def _map_is_free(self, x: float, y: float) -> bool:
        if not self.map_data or self.map_width <= 0 or self.map_height <= 0:
            return True
        col = int((x - self.map_origin[0]) / self.map_resolution)
        row = int((y - self.map_origin[1]) / self.map_resolution)
        if col < 0 or col >= self.map_width or row < 0 or row >= self.map_height:
            return False
        return self.map_data[row * self.map_width + col] == 0

    def _map_is_clear(self, x: float, y: float, radius: float) -> bool:
        if not self._map_is_free(x, y):
            return False
        if not self.map_data:
            return True
        cell_radius = max(int(math.ceil(radius / self.map_resolution)), 1)
        center_col = int((x - self.map_origin[0]) / self.map_resolution)
        center_row = int((y - self.map_origin[1]) / self.map_resolution)
        for row in range(center_row - cell_radius, center_row + cell_radius + 1):
            for col in range(center_col - cell_radius, center_col + cell_radius + 1):
                if col < 0 or col >= self.map_width or row < 0 or row >= self.map_height:
                    return False
                dx = (col - center_col) * self.map_resolution
                dy = (row - center_row) * self.map_resolution
                if math.hypot(dx, dy) <= radius and self.map_data[row * self.map_width + col] != 0:
                    return False
        return True

    def _line_is_clear(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
        radius: float,
    ) -> bool:
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(int(distance / 0.12), 1)
        for index in range(steps + 1):
            t = index / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            if not self._map_is_clear(x, y, radius):
                return False
        return True

    def _read_simple_yaml(self, path: Path) -> dict[str, object]:
        values: dict[str, object] = {}
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            value = value.strip()
            if value.startswith("["):
                values[key.strip()] = ast.literal_eval(value)
            elif value:
                try:
                    values[key.strip()] = ast.literal_eval(value)
                except (ValueError, SyntaxError):
                    values[key.strip()] = value
        return values

    def _read_pgm(self, path: Path) -> tuple[int, int, bytes]:
        with path.open("rb") as file:
            magic = file.readline().strip()
            if magic != b"P5":
                raise ValueError(f"unsupported PGM format {magic!r}")
            line = file.readline().strip()
            while line.startswith(b"#"):
                line = file.readline().strip()
            width, height = [int(part) for part in line.split()]
            max_value = int(file.readline().strip())
            if max_value != 255:
                raise ValueError(f"unsupported PGM max value {max_value}")
            pixels = file.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f"PGM size mismatch: expected {width * height}, got {len(pixels)}")
        return width, height, pixels

    def _trash_marker(self, marker_id: int, obj: TrashSpec) -> Marker:
        alpha = 0.36 if obj.name in self.failed else 0.92
        if self.synthetic_grasp_benchmark:
            record = next((item for item in self.benchmark_records if item.get("name") == obj.name), None)
            if record is not None:
                color = (0.0, 0.95, 0.35) if record.get("ok") else (1.0, 0.12, 0.08)
                return self._box(marker_id, obj.class_name, obj.odom_xyz, obj.size, obj.yaw, self._color(*color, alpha))
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
        header.frame_id = self.map_frame_id
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _camera_pose_odom(self) -> np.ndarray:
        base = self._transform((self.base_x, self.base_y, 0.0), (0.0, 0.0, self.base_yaw))
        return base @ self._camera_pose_base()

    def _camera_pose_base(self, joints: np.ndarray | list[float] | None = None) -> np.ndarray:
        q = np.asarray(self.current_arm if joints is None else joints, dtype=float)
        return self.base_from_aubo @ self.kinematics.fk(q) @ self.tool_to_camera

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

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
