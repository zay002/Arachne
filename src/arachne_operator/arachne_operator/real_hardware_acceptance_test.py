from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arachne_operator.sequence_executor import ARM_JOINTS


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RevoluteJoint:
    name: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


class AuboI5Kinematics:
    """Position-only FK/IK for the Aubo i5 chain from aubo_base_link to tool0."""

    JOINTS = (
        RevoluteJoint(
            "aubo_shoulder_joint",
            (0.0, 0.0, 0.122),
            (0.0, 0.0, math.pi),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_upperArm_joint",
            (0.0, 0.1215, 0.0),
            (-math.pi / 2.0, -math.pi / 2.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_foreArm_joint",
            (0.408, 0.0, 0.0),
            (-math.pi, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist1_joint",
            (0.376, 0.0, 0.0),
            (math.pi, 0.0, math.pi / 2.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist2_joint",
            (0.0, 0.1025, 0.0),
            (-math.pi / 2.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist3_joint",
            (0.0, -0.094, 0.0),
            (math.pi / 2.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    TOOL0_FIXED_RPY = (0.0, 0.0, math.pi / 2.0)

    def fk(self, q: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        for joint, angle in zip(self.JOINTS, q):
            transform = transform @ self._origin(joint.xyz, joint.rpy)
            transform = transform @ self._axis_rotation(joint.axis, float(angle))
        return transform @ self._origin((0.0, 0.0, 0.0), self.TOOL0_FIXED_RPY)

    def solve_position(
        self,
        q_start: np.ndarray,
        target_position: np.ndarray,
        *,
        tolerance: float,
        damping: float,
        max_iterations: int,
        max_step: float,
    ) -> tuple[bool, np.ndarray, float, int]:
        q = np.array(q_start, dtype=float)
        best_q = np.array(q_start, dtype=float)
        best_error = float(np.linalg.norm(self.fk(q)[:3, 3] - target_position))

        for iteration in range(max_iterations):
            current = self.fk(q)[:3, 3]
            error = target_position - current
            norm = float(np.linalg.norm(error))
            if norm < best_error:
                best_q = np.array(q, dtype=float)
                best_error = norm
            if norm <= tolerance:
                return True, q, norm, iteration

            jacobian = self._numeric_position_jacobian(q)
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(3)
            step = jacobian.T @ np.linalg.solve(lhs, error)
            step = np.clip(step, -max_step, max_step)
            q = self._wrap(q + step)

        return False, best_q, best_error, max_iterations

    def _numeric_position_jacobian(self, q: np.ndarray) -> np.ndarray:
        eps = 1e-5
        base = self.fk(q)[:3, 3]
        jacobian = np.zeros((3, len(q)))
        for index in range(len(q)):
            q_eps = np.array(q, dtype=float)
            q_eps[index] += eps
            jacobian[:, index] = (self.fk(q_eps)[:3, 3] - base) / eps
        return jacobian

    def _origin(
        self, xyz: tuple[float, float, float], rpy: tuple[float, float, float]
    ) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = self._rpy_matrix(*rpy)
        transform[:3, 3] = np.array(xyz, dtype=float)
        return transform

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
        return rz @ ry @ rx

    def _axis_rotation(self, axis: tuple[float, float, float], angle: float) -> np.ndarray:
        vector = np.array(axis, dtype=float)
        vector = vector / np.linalg.norm(vector)
        x, y, z = vector
        c = math.cos(angle)
        s = math.sin(angle)
        c1 = 1.0 - c
        rotation = np.array(
            [
                [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
                [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
                [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
            ],
            dtype=float,
        )
        transform = np.eye(4)
        transform[:3, :3] = rotation
        return transform

    def _wrap(self, q: np.ndarray) -> np.ndarray:
        return np.array([math.atan2(math.sin(value), math.cos(value)) for value in q])


class RealHardwareAcceptanceTest(Node):
    def __init__(self) -> None:
        super().__init__("arachne_real_hardware_acceptance_test")
        self.declare_parameter("confirm_motion", False)
        self.declare_parameter("run_base_test", True)
        self.declare_parameter("run_arm_test", True)
        self.declare_parameter("run_gripper_test", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory")
        self.declare_parameter(
            "legacy_arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("arm_state_joint_names", ",".join(ARM_JOINTS))
        self.declare_parameter("arm_command_joint_names", ",".join(ARM_JOINTS))
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("base_distance_m", 0.2)
        self.declare_parameter("base_linear_speed", 0.06)
        self.declare_parameter("base_yaw_deg", 30.0)
        self.declare_parameter("base_angular_speed", 0.22)
        self.declare_parameter("base_distance_tolerance", 0.015)
        self.declare_parameter("base_yaw_tolerance_deg", 1.5)
        self.declare_parameter("base_settle_sec", 0.8)
        self.declare_parameter("arm_z_delta_m", 0.2)
        self.declare_parameter("arm_z_frame", "aubo_base")
        self.declare_parameter("arm_duration_sec", 4.0)
        self.declare_parameter("arm_settle_sec", 1.0)
        self.declare_parameter("arm_position_tolerance", 0.008)
        self.declare_parameter("arm_ik_damping", 0.08)
        self.declare_parameter("arm_ik_max_iterations", 180)
        self.declare_parameter("arm_ik_max_step", 0.06)
        self.declare_parameter("arm_max_joint_delta", 1.0)
        self.declare_parameter("gripper_cycles", 5)
        self.declare_parameter("gripper_pause_sec", 0.8)
        self.declare_parameter("gripper_final_state", "open")
        self.declare_parameter("feedback_timeout_sec", 8.0)

        self.confirm_motion = bool(self.get_parameter("confirm_motion").value)
        self.latest_odom: Odometry | None = None
        self.latest_joint_state: JointState | None = None
        self.current_arm: dict[str, float] = {}
        self.arm_state_joint_names = self._parse_joint_names(
            str(self.get_parameter("arm_state_joint_names").value)
        )
        self.arm_command_joint_names = self._parse_joint_names(
            str(self.get_parameter("arm_command_joint_names").value)
        )
        if len(self.arm_state_joint_names) != 6 or len(self.arm_command_joint_names) != 6:
            raise ValueError("arm_state_joint_names and arm_command_joint_names must list 6 joints")
        self.kinematics = AuboI5Kinematics()

        arm_topics = []
        for topic in (
            str(self.get_parameter("arm_trajectory_topic").value),
            str(self.get_parameter("legacy_arm_trajectory_topic").value),
        ):
            if topic and topic not in arm_topics:
                arm_topics.append(topic)

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.arm_publishers = [
            self.create_publisher(JointTrajectory, topic, 10) for topic in arm_topics
        ]
        self.gripper_pub = self.create_publisher(
            String, str(self.get_parameter("gripper_command_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/arachne/acceptance/status", 10)

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )

    def run(self) -> bool:
        self._status("acceptance test loaded")
        self._status(f"plan: {self._plan_summary()}")
        if not self.confirm_motion:
            self._status("dry run only: set confirm_motion:=true to command real hardware", warn=True)
            return True

        try:
            self._status("REAL MOTION CONFIRMED")
            if bool(self.get_parameter("run_base_test").value):
                self._wait_for_odom()
                self._run_base_sequence()
            if bool(self.get_parameter("run_arm_test").value):
                self._wait_for_arm_state()
                self._run_arm_sequence()
            if bool(self.get_parameter("run_gripper_test").value):
                self._run_gripper_sequence()
            self._publish_stop()
            self._status("acceptance test complete")
            return True
        except Exception as exc:
            self._publish_stop()
            self._publish_gripper("stop")
            self._status(f"acceptance test failed: {exc}", warn=True)
            return False

    def _plan_summary(self) -> str:
        steps: list[str] = []
        if bool(self.get_parameter("run_base_test").value):
            steps.append("base +0.2m/-0.2m, left 30deg/return, right 30deg/return")
        if bool(self.get_parameter("run_arm_test").value):
            z_delta = float(self.get_parameter("arm_z_delta_m").value)
            frame = str(self.get_parameter("arm_z_frame").value)
            steps.append(f"tool0 z +{z_delta:.3f}m/return in {frame}")
        if bool(self.get_parameter("run_gripper_test").value):
            steps.append("gripper open-close x5")
        return "; ".join(steps) if steps else "no subsystem selected"

    def _run_base_sequence(self) -> None:
        distance = float(self.get_parameter("base_distance_m").value)
        yaw = math.radians(float(self.get_parameter("base_yaw_deg").value))
        self._status("base test: forward")
        self._drive_relative(distance)
        self._settle()
        self._status("base test: backward")
        self._drive_relative(-distance)
        self._settle()
        self._status("base test: left yaw")
        self._turn_relative(yaw)
        self._settle()
        self._status("base test: return from left yaw")
        self._turn_relative(-yaw)
        self._settle()
        self._status("base test: right yaw")
        self._turn_relative(-yaw)
        self._settle()
        self._status("base test: return from right yaw")
        self._turn_relative(yaw)
        self._settle()

    def _run_arm_sequence(self) -> None:
        q_start = self._current_arm_vector()
        start_transform = self.kinematics.fk(q_start)
        start_position = start_transform[:3, 3]
        frame = str(self.get_parameter("arm_z_frame").value).strip().lower()
        z_delta = float(self.get_parameter("arm_z_delta_m").value)
        if frame in ("tool", "tool0", "ee"):
            direction = start_transform[:3, 2]
        else:
            direction = np.array([0.0, 0.0, 1.0], dtype=float)
        target_position = start_position + direction * z_delta

        ok, q_up, error, iterations = self.kinematics.solve_position(
            q_start,
            target_position,
            tolerance=float(self.get_parameter("arm_position_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_up - q_start)))
        if not ok:
            raise RuntimeError(f"arm IK failed: best_error={error:.4f} after {iterations} iterations")
        if max_delta > float(self.get_parameter("arm_max_joint_delta").value):
            raise RuntimeError(f"arm IK joint delta too large: {max_delta:.3f} rad")

        self._status(
            "arm test: tool0 z up "
            f"{z_delta:.3f}m, ik_error={error:.4f}, max_joint_delta={max_delta:.3f}"
        )
        self._publish_arm(q_up)
        self._sleep(float(self.get_parameter("arm_duration_sec").value))
        self._sleep(float(self.get_parameter("arm_settle_sec").value))
        self._status("arm test: return to start")
        self._publish_arm(q_start)
        self._sleep(float(self.get_parameter("arm_duration_sec").value))
        self._sleep(float(self.get_parameter("arm_settle_sec").value))

    def _run_gripper_sequence(self) -> None:
        cycles = int(self.get_parameter("gripper_cycles").value)
        pause = float(self.get_parameter("gripper_pause_sec").value)
        for index in range(cycles):
            self._status(f"gripper test: cycle {index + 1}/{cycles} open")
            self._publish_gripper("open")
            self._sleep(pause)
            self._status(f"gripper test: cycle {index + 1}/{cycles} close")
            self._publish_gripper("close")
            self._sleep(pause)
        final_state = str(self.get_parameter("gripper_final_state").value).strip().lower()
        if final_state in ("open", "close", "stop"):
            self._publish_gripper(final_state)
            self._status(f"gripper test: final_state={final_state}")

    def _drive_relative(self, distance: float) -> None:
        start = self._current_pose2d()
        heading = np.array([math.cos(start.yaw), math.sin(start.yaw)])
        speed = float(self.get_parameter("base_linear_speed").value)
        tolerance = float(self.get_parameter("base_distance_tolerance").value)
        timeout = abs(distance) / max(abs(speed), 1e-3) + 6.0
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            pose = self._current_pose2d()
            offset = np.array([pose.x - start.x, pose.y - start.y])
            progress = float(offset @ heading)
            if sign * progress >= abs(distance) - tolerance:
                self._publish_stop()
                self._status(f"base distance reached: {progress:.3f}m target={distance:.3f}m")
                return
            twist = Twist()
            twist.linear.x = sign * abs(speed)
            self.cmd_vel_pub.publish(twist)
            self._spin_sleep(0.05)
        raise TimeoutError(f"base distance timeout target={distance:.3f}m")

    def _turn_relative(self, angle: float) -> None:
        start = self._current_pose2d()
        speed = float(self.get_parameter("base_angular_speed").value)
        tolerance = math.radians(float(self.get_parameter("base_yaw_tolerance_deg").value))
        timeout = abs(angle) / max(abs(speed), 1e-3) + 6.0
        sign = 1.0 if angle >= 0.0 else -1.0
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            pose = self._current_pose2d()
            delta = self._angle_diff(pose.yaw, start.yaw)
            if sign * delta >= abs(angle) - tolerance:
                self._publish_stop()
                self._status(
                    f"base yaw reached: {math.degrees(delta):.2f}deg "
                    f"target={math.degrees(angle):.2f}deg"
                )
                return
            twist = Twist()
            twist.angular.z = sign * abs(speed)
            self.cmd_vel_pub.publish(twist)
            self._spin_sleep(0.05)
        raise TimeoutError(f"base yaw timeout target={math.degrees(angle):.2f}deg")

    def _publish_arm(self, positions: np.ndarray) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_command_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        duration = float(self.get_parameter("arm_duration_sec").value)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]
        for publisher in self.arm_publishers:
            publisher.publish(trajectory)

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _settle(self) -> None:
        self._publish_stop()
        self._sleep(float(self.get_parameter("base_settle_sec").value))

    def _sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(seconds, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    def _spin_sleep(self, seconds: float) -> None:
        rclpy.spin_once(self, timeout_sec=max(seconds, 0.0))

    def _wait_for_odom(self) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_odom is None and time.monotonic() < deadline:
            self._spin_sleep(0.05)
        if self.latest_odom is None:
            raise TimeoutError("no /odom feedback")

    def _wait_for_arm_state(self) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.current_arm for name in self.arm_state_joint_names):
                return
            self._spin_sleep(0.05)
        missing = [name for name in self.arm_state_joint_names if name not in self.current_arm]
        raise TimeoutError(f"missing arm joint states: {missing}")

    def _current_pose2d(self) -> Pose2D:
        if self.latest_odom is None:
            raise RuntimeError("odom is not available")
        pose = self.latest_odom.pose.pose
        yaw = self._yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        return Pose2D(pose.position.x, pose.position.y, yaw)

    def _current_arm_vector(self) -> np.ndarray:
        return np.array([self.current_arm[name] for name in self.arm_state_joint_names], dtype=float)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        for name, position in zip(msg.name, msg.position):
            if name in self.arm_state_joint_names:
                self.current_arm[name] = float(position)

    def _parse_joint_names(self, value: str) -> tuple[str, ...]:
        names = tuple(item.strip() for item in value.split(",") if item.strip())
        return names

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _angle_diff(self, angle: float, reference: float) -> float:
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    def _status(self, text: str, warn: bool = False) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RealHardwareAcceptanceTest()
    exit_code = 0
    try:
        if not node.run():
            exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        node._publish_stop()
        node._publish_gripper("stop")
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
