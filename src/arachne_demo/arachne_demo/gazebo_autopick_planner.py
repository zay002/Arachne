from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


ARM_JOINT_NAMES = [
    "aubo_shoulder_joint",
    "aubo_upperArm_joint",
    "aubo_foreArm_joint",
    "aubo_wrist1_joint",
    "aubo_wrist2_joint",
    "aubo_wrist3_joint",
]

HOME_POSE = [
    -1.5707963267949,
    0.201570428261868,
    1.65970467002488,
    0.485178041391533,
    1.67675136677345,
    0.76432946885334,
]
READY_POSE = [1.38, -0.22, -1.10, 0.12, -1.38, 0.0]
IK_MAX_STEP = 0.08
IK_DAMPING = 0.08
IK_EPSILON = 1.0e-4
GRASP_FRAME_OFFSET = 0.165

ARM_MOUNT_XYZ = (0.22, 0.0, 0.105)
ARM_MOUNT_RPY = (0.0, 0.0, math.pi / 2.0)
AUBO_JOINT_ORIGINS = [
    ((0.0, 0.0, 0.122), (0.0, 0.0, math.pi)),
    ((0.0, 0.1215, 0.0), (-math.pi / 2.0, -math.pi / 2.0, 0.0)),
    ((0.408, 0.0, 0.0), (-math.pi, 0.0, 0.0)),
    ((0.376, 0.0, 0.0), (math.pi, 0.0, math.pi / 2.0)),
    ((0.0, 0.1025, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, -0.094, 0.0), (math.pi / 2.0, 0.0, 0.0)),
]
TOOL0_RPY = (0.0, 0.0, math.pi / 2.0)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class Obstacle:
    name: str
    x: float
    y: float
    radius: float


class GazeboAutoPickPlanner(Node):
    """Known-world autonomy validation for the Gazebo showroom.

    This node intentionally keeps the planner dependency-free. The base uses a
    continuously refreshed grid A* path over the known SDF showroom geometry,
    then follows it with a pure-pursuit style controller. The arm does not play
    a prewritten trajectory: every control tick computes a target joint pose
    from the current base pose and the target object pose, then rate-limits the
    command toward that pose. This is a simulation validation layer; the same
    launch contract can later be backed by MoveIt2 and ros2_control.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_autopick_planner")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/gz/odom")
        self.declare_parameter("arm_joint_state_topic", "/arachne/gui_joint_states")
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("target_name", "pick_bottle")
        self.declare_parameter("target_x", 3.40)
        self.declare_parameter("target_y", -2.35)
        self.declare_parameter("target_z", 0.38)
        self.declare_parameter("approach_x", 2.62)
        self.declare_parameter("approach_y", -2.35)
        self.declare_parameter("approach_standoff", 0.76)
        self.declare_parameter("grid_resolution", 0.16)
        self.declare_parameter("robot_radius", 0.48)
        self.declare_parameter("obstacle_clearance", 0.18)
        self.declare_parameter("lookahead_distance", 0.46)
        self.declare_parameter("linear_speed", 0.42)
        self.declare_parameter("angular_speed", 1.05)
        self.declare_parameter("position_tolerance", 0.12)
        self.declare_parameter("yaw_tolerance", 0.16)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("replan_period", 0.75)
        self.declare_parameter("arm_joint_speed", 0.55)
        self.declare_parameter("arm_tolerance", 0.045)
        self.declare_parameter("auto_start", True)

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.arm_pub = self.create_publisher(
            JointState, str(self.get_parameter("arm_joint_state_topic").value), 10
        )
        self.arm_command_pubs = {
            joint: self.create_publisher(Float64, f"/arachne/gazebo/{joint}/command", 10)
            for joint in ARM_JOINT_NAMES
        }
        self.gripper_pub = self.create_publisher(
            String, str(self.get_parameter("gripper_command_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            10,
        )

        self.target = (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
        )
        self.approach = (
            float(self.get_parameter("approach_x").value),
            float(self.get_parameter("approach_y").value),
        )
        self.grid_resolution = float(self.get_parameter("grid_resolution").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.obstacle_clearance = float(self.get_parameter("obstacle_clearance").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.replan_period = float(self.get_parameter("replan_period").value)
        self.arm_joint_speed = float(self.get_parameter("arm_joint_speed").value)
        self.arm_tolerance = float(self.get_parameter("arm_tolerance").value)
        self.approach_standoff = float(self.get_parameter("approach_standoff").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.bounds = (-5.4, 5.4, -3.55, 3.55)
        self.obstacles = self._known_obstacles()
        self.pose: Pose2D | None = None
        self.state = "wait_odom"
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.current_arm = list(HOME_POSE)
        self.arm_target = list(HOME_POSE)
        self.arm_phase = "home"
        self.arm_phase_elapsed = 0.0
        self.last_ee_error = math.inf
        self.last_cartesian_target = (0.0, 0.0, 0.0)
        self.last_target_distance = math.inf
        self.gripper_closed_sent = False
        self.replan_elapsed = 0.0
        self.last_cmd = Twist()
        self.last_update = self.get_clock().now()
        self.last_status_log = self.get_clock().now()
        self.started = False

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)
        self.get_logger().info(
            "Gazebo auto-pick planner ready for target "
            f"{self.get_parameter('target_name').value} at "
            f"({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})"
        )

    def _known_obstacles(self) -> list[Obstacle]:
        return [
            Obstacle("demo_pedestal", -1.20, -1.40, 0.58),
            Obstacle("soft_target_box", 1.00, -1.50, 0.45),
            Obstacle("marker_column", -2.00, 1.40, 0.38),
            Obstacle("slalom_marker_a", -1.40, 2.10, 0.32),
            Obstacle("slalom_marker_b", -0.40, 1.55, 0.32),
            Obstacle("slalom_marker_c", 0.60, 2.10, 0.32),
            Obstacle("work_table", 2.45, 1.70, 0.62),
            Obstacle("inspection_crate", 3.45, 0.25, 0.50),
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = Pose2D(position.x, position.y, position.z, yaw)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now.nanoseconds - self.last_update.nanoseconds) * 1e-9, 0.0)
        self.last_update = now

        if self.pose is None:
            self._publish_arm()
            return

        if self.state == "wait_odom":
            self._publish_gripper("open")
            self._plan_base_path(force_log=True)
            self.state = "navigate" if self.auto_start else "idle"
            self.started = self.auto_start
            self._publish_arm()
            return

        if self.state == "navigate":
            self._drive_path(dt)
        elif self.state == "align":
            self._align_to_target()
        elif self.state in ("pre_grasp", "grasp", "close", "lift", "return"):
            self._run_realtime_arm(dt)
        elif self.state == "done":
            self._stop_base()
        self._publish_arm()
        self._log_status()

    def _plan_base_path(self, force_log: bool = False) -> None:
        if self.pose is None:
            return
        start = (self.pose.x, self.pose.y)
        goal = self.approach
        raw_path = self._astar(start, goal)
        if not raw_path:
            if self.path:
                if force_log:
                    self.get_logger().warning("A* failed; keeping previous path")
                return
            self.get_logger().warning("A* failed; falling back to sparse known-safe route")
            raw_path = [start, (-0.20, 0.30), (0.60, 0.88), goal]
        self.path = self._smooth_path(raw_path)
        self.path_index = 0
        if force_log:
            self.get_logger().info(f"Planned {len(self.path)} base waypoints toward target")

    def _astar(self, start: tuple[float, float], goal: tuple[float, float]) -> list[tuple[float, float]]:
        start_cell = self._world_to_cell(start)
        goal_cell = self._world_to_cell(goal)
        if self._blocked(start_cell):
            start_cell = self._nearest_free_cell(start_cell)
        if self._blocked(goal_cell):
            goal_cell = self._nearest_free_cell(goal_cell)
        open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score = {start_cell: 0.0}
        visited: set[tuple[int, int]] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)
            if current == goal_cell:
                return [self._cell_to_world(cell) for cell in self._reconstruct(came_from, current)]

            for neighbor, cost in self._neighbors(current):
                if neighbor in visited or self._blocked(neighbor):
                    continue
                tentative = g_score[current] + cost
                if tentative >= g_score.get(neighbor, math.inf):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                priority = tentative + self._heuristic(neighbor, goal_cell)
                heapq.heappush(open_heap, (priority, neighbor))
        return []

    def _world_to_cell(self, point: tuple[float, float]) -> tuple[int, int]:
        min_x, _, min_y, _ = self.bounds
        return (
            int(round((point[0] - min_x) / self.grid_resolution)),
            int(round((point[1] - min_y) / self.grid_resolution)),
        )

    def _cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        min_x, _, min_y, _ = self.bounds
        return (
            min_x + cell[0] * self.grid_resolution,
            min_y + cell[1] * self.grid_resolution,
        )

    def _blocked(self, cell: tuple[int, int]) -> bool:
        x, y = self._cell_to_world(cell)
        min_x, max_x, min_y, max_y = self.bounds
        if x < min_x or x > max_x or y < min_y or y > max_y:
            return True
        for obstacle in self.obstacles:
            clearance = obstacle.radius + self.robot_radius + self.obstacle_clearance
            if math.hypot(x - obstacle.x, y - obstacle.y) < clearance:
                return True
        return False

    def _nearest_free_cell(self, cell: tuple[int, int]) -> tuple[int, int]:
        for radius in range(1, 14):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if not self._blocked(candidate):
                        return candidate
        return cell

    def _neighbors(self, cell: tuple[int, int]) -> Iterable[tuple[tuple[int, int], float]]:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                yield (cell[0] + dx, cell[1] + dy), cost

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _smooth_path(self, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        anchor_index = 0
        test_index = 2
        while test_index < len(path):
            if self._segment_is_clear(path[anchor_index], path[test_index]):
                test_index += 1
                continue
            smoothed.append(path[test_index - 1])
            anchor_index = test_index - 1
            test_index = anchor_index + 2
        smoothed.append(path[-1])
        return smoothed

    def _segment_is_clear(self, start: tuple[float, float], end: tuple[float, float]) -> bool:
        distance = math.dist(start, end)
        steps = max(int(distance / (self.grid_resolution * 0.5)), 1)
        for i in range(steps + 1):
            t = i / steps
            point = (start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t)
            if self._blocked(self._world_to_cell(point)):
                return False
        return True

    def _drive_path(self, dt: float) -> None:
        if self.pose is None or not self.path:
            return
        self.replan_elapsed += dt
        if self.replan_elapsed >= self.replan_period:
            self.replan_elapsed = 0.0
            self._plan_base_path(force_log=False)
        while self.path_index < len(self.path) - 1:
            waypoint = self.path[self.path_index]
            if math.hypot(waypoint[0] - self.pose.x, waypoint[1] - self.pose.y) > self.lookahead_distance:
                break
            self.path_index += 1

        goal = self.path[-1]
        goal_distance = math.hypot(goal[0] - self.pose.x, goal[1] - self.pose.y)
        if goal_distance < self.position_tolerance:
            self._stop_base()
            self.state = "align"
            self.get_logger().info("Base reached approach point; aligning to target")
            return

        target = self.path[min(self.path_index, len(self.path) - 1)]
        target_yaw = math.atan2(target[1] - self.pose.y, target[0] - self.pose.x)
        yaw_error = self._normalize_angle(target_yaw - self.pose.yaw)
        twist = Twist()
        if abs(yaw_error) > 0.30:
            twist.linear.x = 0.0
        else:
            heading_scale = max(math.cos(yaw_error), 0.0)
            twist.linear.x = self.linear_speed * min(goal_distance / 0.7, 1.0) * heading_scale
        twist.angular.z = self._clamp(yaw_error * 2.6, -self.angular_speed, self.angular_speed)
        self.last_cmd = twist
        self.cmd_pub.publish(twist)

    def _align_to_target(self) -> None:
        if self.pose is None:
            return
        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y
        target_distance = math.hypot(dx, dy)
        self.last_target_distance = target_distance
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.pose.yaw)
        distance_error = target_distance - self.approach_standoff
        if abs(yaw_error) < self.yaw_tolerance and abs(distance_error) < 0.055:
            self._stop_base()
            self.state = "pre_grasp"
            self.arm_phase = "pre_grasp"
            self.arm_phase_elapsed = 0.0
            self.gripper_closed_sent = False
            self._publish_gripper("open")
            self.get_logger().info("Base aligned; starting realtime arm planning")
            return
        twist = Twist()
        if abs(yaw_error) < 0.38:
            twist.linear.x = self._clamp(distance_error * 0.72, -0.20, 0.24)
        twist.angular.z = self._clamp(yaw_error * 2.2, -0.64, 0.64)
        self.last_cmd = twist
        self.cmd_pub.publish(twist)

    def _local_target_point(self) -> tuple[float, float, float]:
        if self.pose is None:
            return (0.70, 0.0, 0.35)
        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y
        self.last_target_distance = math.hypot(dx, dy)
        local_x = math.cos(self.pose.yaw) * dx + math.sin(self.pose.yaw) * dy
        local_y = -math.sin(self.pose.yaw) * dx + math.cos(self.pose.yaw) * dy
        local_z = self.target[2] - self.pose.z
        return (
            self._clamp(local_x, 0.48, 0.88),
            self._clamp(local_y, -0.26, 0.26),
            self._clamp(local_z, 0.12, 0.64),
        )

    def _run_realtime_arm(self, dt: float) -> None:
        self.arm_phase_elapsed += dt
        self.arm_target = self._realtime_arm_target(self.state)
        self._step_arm_toward_target(dt)
        error = self._arm_error(self.arm_target)

        if self.state == "pre_grasp" and (
            (self.arm_phase_elapsed > 1.4 and error < self.arm_tolerance)
            or self.arm_phase_elapsed > 3.2
        ):
            self._enter_arm_state("grasp")
        elif self.state == "grasp" and (
            (self.arm_phase_elapsed > 1.2 and error < self.arm_tolerance)
            or self.arm_phase_elapsed > 2.8
        ):
            self._enter_arm_state("close")
        elif self.state == "close":
            if not self.gripper_closed_sent:
                self.gripper_closed_sent = True
                self._publish_gripper("close")
            if self.arm_phase_elapsed > 0.85:
                self._enter_arm_state("lift")
        elif self.state == "lift" and (
            (self.arm_phase_elapsed > 1.4 and error < self.arm_tolerance)
            or self.arm_phase_elapsed > 3.0
        ):
            self._enter_arm_state("return")
        elif self.state == "return" and (
            (self.arm_phase_elapsed > 1.8 and error < self.arm_tolerance)
            or self.arm_phase_elapsed > 3.6
        ):
            self.state = "done"
            self.arm_phase = "done"
            self._publish_gripper("close")
            self.get_logger().info("Gazebo realtime auto-pick validation sequence complete")

    def _enter_arm_state(self, state: str) -> None:
        self.state = state
        self.arm_phase = state
        self.arm_phase_elapsed = 0.0
        self.get_logger().info(f"Realtime arm phase: {state}")

    def _realtime_arm_target(self, phase: str) -> list[float]:
        pick_point = self._local_target_point()
        if phase == "pre_grasp":
            target = (pick_point[0] - 0.16, pick_point[1], pick_point[2] + 0.16)
        elif phase in ("grasp", "close"):
            target = pick_point
        elif phase == "lift":
            target = (pick_point[0] - 0.04, pick_point[1], pick_point[2] + 0.28)
        elif phase == "return":
            self.last_cartesian_target = self._fk_grasp_position(HOME_POSE)
            self.last_ee_error = self._position_error(
                self.last_cartesian_target,
                self._fk_grasp_position(self.current_arm),
            )
            return list(HOME_POSE)
        else:
            self.last_cartesian_target = self._fk_grasp_position(READY_POSE)
            self.last_ee_error = self._position_error(
                self.last_cartesian_target,
                self._fk_grasp_position(self.current_arm),
            )
            return list(READY_POSE)
        self.last_cartesian_target = target
        return self._solve_ik_position(target, self.current_arm)

    def _solve_ik_position(self, target: tuple[float, float, float], seed: list[float]) -> list[float]:
        q = list(seed)
        best_q = list(q)
        best_error = math.inf
        for _ in range(58):
            position = self._fk_grasp_position(q)
            error_vector = [target[index] - position[index] for index in range(3)]
            error = self._vector_norm(error_vector)
            if error < best_error:
                best_error = error
                best_q = list(q)
            if error < 0.012:
                break

            jacobian = self._finite_difference_jacobian(q, position)
            normal = [
                [
                    sum(jacobian[col][row] * jacobian[col][column] for col in range(6))
                    + (IK_DAMPING * IK_DAMPING if row == column else 0.0)
                    for column in range(3)
                ]
                for row in range(3)
            ]
            inverse = self._inverse_3x3(normal)
            if inverse is None:
                break
            task_step = [
                sum(inverse[row][column] * error_vector[column] for column in range(3))
                for row in range(3)
            ]
            for joint_index in range(6):
                delta = sum(jacobian[joint_index][axis] * task_step[axis] for axis in range(3))
                delta += 0.004 * (HOME_POSE[joint_index] - q[joint_index])
                q[joint_index] += self._clamp(delta, -IK_MAX_STEP, IK_MAX_STEP)
                q[joint_index] = self._normalize_angle(q[joint_index])

        self.last_ee_error = best_error
        return best_q

    def _finite_difference_jacobian(
        self,
        q: list[float],
        current_position: tuple[float, float, float],
    ) -> list[list[float]]:
        jacobian: list[list[float]] = []
        for joint_index in range(6):
            perturbed = list(q)
            perturbed[joint_index] += IK_EPSILON
            next_position = self._fk_grasp_position(perturbed)
            jacobian.append(
                [
                    (next_position[axis] - current_position[axis]) / IK_EPSILON
                    for axis in range(3)
                ]
            )
        return jacobian

    def _fk_grasp_position(self, q: list[float]) -> tuple[float, float, float]:
        transform = self._transform(ARM_MOUNT_XYZ, ARM_MOUNT_RPY)
        for joint_position, (xyz, rpy) in zip(q, AUBO_JOINT_ORIGINS):
            transform = self._matmul(transform, self._transform(xyz, rpy))
            transform = self._matmul(transform, self._rotation_z(joint_position))
        transform = self._matmul(transform, self._transform((0.0, 0.0, 0.0), TOOL0_RPY))
        transform = self._matmul(transform, self._translation((0.0, 0.0, GRASP_FRAME_OFFSET)))
        return (transform[0][3], transform[1][3], transform[2][3])

    @staticmethod
    def _identity() -> list[list[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
        return [
            [sum(left[row][inner] * right[inner][column] for inner in range(4)) for column in range(4)]
            for row in range(4)
        ]

    @classmethod
    def _translation(cls, xyz: tuple[float, float, float]) -> list[list[float]]:
        transform = cls._identity()
        transform[0][3] = xyz[0]
        transform[1][3] = xyz[1]
        transform[2][3] = xyz[2]
        return transform

    @classmethod
    def _transform(
        cls,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float],
    ) -> list[list[float]]:
        return cls._matmul(cls._translation(xyz), cls._rpy_matrix(rpy))

    @classmethod
    def _rpy_matrix(cls, rpy: tuple[float, float, float]) -> list[list[float]]:
        return cls._matmul(
            cls._matmul(cls._rotation_z(rpy[2]), cls._rotation_y(rpy[1])),
            cls._rotation_x(rpy[0]),
        )

    @staticmethod
    def _rotation_x(angle: float) -> list[list[float]]:
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cosine, -sine, 0.0],
            [0.0, sine, cosine, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rotation_y(angle: float) -> list[list[float]]:
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return [
            [cosine, 0.0, sine, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-sine, 0.0, cosine, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rotation_z(angle: float) -> list[list[float]]:
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return [
            [cosine, -sine, 0.0, 0.0],
            [sine, cosine, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _inverse_3x3(matrix: list[list[float]]) -> list[list[float]] | None:
        a, b, c = matrix[0]
        d, e, f = matrix[1]
        g, h, i = matrix[2]
        cofactor_a = e * i - f * h
        cofactor_b = -(d * i - f * g)
        cofactor_c = d * h - e * g
        cofactor_d = -(b * i - c * h)
        cofactor_e = a * i - c * g
        cofactor_f = -(a * h - b * g)
        cofactor_g = b * f - c * e
        cofactor_h = -(a * f - c * d)
        cofactor_i = a * e - b * d
        determinant = a * cofactor_a + b * cofactor_b + c * cofactor_c
        if abs(determinant) < 1.0e-9:
            return None
        return [
            [cofactor_a / determinant, cofactor_d / determinant, cofactor_g / determinant],
            [cofactor_b / determinant, cofactor_e / determinant, cofactor_h / determinant],
            [cofactor_c / determinant, cofactor_f / determinant, cofactor_i / determinant],
        ]

    def _step_arm_toward_target(self, dt: float) -> None:
        max_step = self.arm_joint_speed * max(dt, 0.0)
        next_arm: list[float] = []
        for current, target in zip(self.current_arm, self.arm_target):
            delta = target - current
            if abs(delta) <= max_step:
                next_arm.append(target)
            else:
                next_arm.append(current + math.copysign(max_step, delta))
        self.current_arm = next_arm

    def _arm_error(self, target: list[float]) -> float:
        if not target:
            return math.inf
        return max(abs(current - goal) for current, goal in zip(self.current_arm, target))

    def _publish_arm(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = list(self.current_arm)
        self.arm_pub.publish(msg)
        for joint, position in zip(ARM_JOINT_NAMES, self.current_arm):
            command = Float64()
            command.data = float(position)
            self.arm_command_pubs[joint].publish(command)

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _stop_base(self) -> None:
        self.last_cmd = Twist()
        self.cmd_pub.publish(self.last_cmd)

    def _log_status(self) -> None:
        now = self.get_clock().now()
        if (now.nanoseconds - self.last_status_log.nanoseconds) * 1e-9 < 2.0:
            return
        self.last_status_log = now
        if self.pose is None:
            return
        base_error = math.hypot(self.approach[0] - self.pose.x, self.approach[1] - self.pose.y)
        arm_error = self._arm_error(self.arm_target)
        self.get_logger().info(
            f"state={self.state} base=({self.pose.x:.2f}, {self.pose.y:.2f}, {self.pose.yaw:.2f}) "
            f"approach_error={base_error:.2f} target_distance={self.last_target_distance:.2f} "
            f"cmd=({self.last_cmd.linear.x:.2f}, {self.last_cmd.angular.z:.2f}) "
            f"arm_error={arm_error:.2f} ee_error={self.last_ee_error:.3f}"
        )

    @classmethod
    def _position_error(
        cls,
        target: tuple[float, float, float],
        position: tuple[float, float, float],
    ) -> float:
        return cls._vector_norm([target[index] - position[index] for index in range(3)])

    @staticmethod
    def _vector_norm(values: list[float]) -> float:
        return math.sqrt(sum(value * value for value in values))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _lerp(start: float, end: float, t: float) -> float:
        return start + (end - start) * t


def main() -> None:
    rclpy.init()
    node = GazeboAutoPickPlanner()
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
            rclpy.shutdown()


if __name__ == "__main__":
    main()
