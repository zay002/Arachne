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
from geometry_msgs.msg import Point
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
READY = [1.50, -0.30, -1.10, 0.40, -1.55, 0.0]
GRASP = [1.20, -0.26, -1.26, 0.34, -1.44, 0.0]

TRASH_X = 0.84
TRASH_Y = -0.38
TRASH_GROUND_Z = -0.22
TRASH_APPROACH_Z = 0.12
TRASH_GRASP_Z = -0.16
TRASH_LIFT_Z = 0.20
BASKET_X = 0.545
BASKET_Y = 0.0
BASKET_OVER_Z = 0.20
GRASP_FRAME_OFFSET_Z = 0.138691938
ARM_MOUNT_XYZ = (0.22, 0.0, 0.155)
ARM_MOUNT_RPY = (0.0, 0.0, math.pi / 2.0)
TOOL_ADAPTER_RPY = (0.0, 0.0, math.pi / 4.0)
RELEASE_SHOULDER_ELBOW_WEIGHT = 4.0
RELEASE_WRIST_WEIGHT = 0.35
RELEASE_CONFIGURATION_WEIGHT = 4.0


@dataclass(frozen=True)
class GraspTarget:
    label: str
    grasp_xyz_base: tuple[float, float, float]
    gripper: tuple[float, float]


@dataclass(frozen=True)
class Stage:
    label: str
    target: list[float]
    gripper: tuple[float, float]


class MoveItGraspPlanningDemo(Node):
    """Synthetic vision-to-grasp demo for RViz and MoveIt planning.

    The node publishes a fake recognized target and task waypoints, asks MoveIt
    for joint-space plans between named grasp stages, then plays the returned
    trajectories on the display joint-state topic used by Arachne's RViz model.
    """

    def __init__(self) -> None:
        super().__init__("moveit_grasp_planning_demo")
        self.declare_parameter("plan_service", "/plan_kinematic_path")
        self.declare_parameter("joint_state_topic", "/arachne/grasp_preview/joint_states")
        self.declare_parameter("marker_topic", "/arachne/grasp_preview/markers")
        self.declare_parameter("group_name", "aubo_arm")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("planning_time", 4.0)
        self.declare_parameter("planning_attempts", 8)
        self.declare_parameter("publish_rate", 60.0)
        self.declare_parameter("playback_speed", 0.8)
        self.declare_parameter("loop", True)
        self.declare_parameter("allow_interpolation_fallback", False)

        self.plan_client = self.create_client(
            GetMotionPlan, str(self.get_parameter("plan_service").value)
        )
        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10
        )

        self.group_name = str(self.get_parameter("group_name").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.planning_attempts = int(self.get_parameter("planning_attempts").value)
        self.playback_speed = max(float(self.get_parameter("playback_speed").value), 0.05)
        self.loop = bool(self.get_parameter("loop").value)
        self.allow_fallback = bool(self.get_parameter("allow_interpolation_fallback").value)

        self.kinematics = AuboI5Kinematics()
        self.base_from_aubo = self._transform(ARM_MOUNT_XYZ, ARM_MOUNT_RPY)
        self.aubo_from_base = self._invert_rigid(self.base_from_aubo)
        self.tool_to_grasp = self._transform(
            (0.0, 0.0, GRASP_FRAME_OFFSET_Z), TOOL_ADAPTER_RPY
        )
        self.grasp_rotation_base = self._nominal_grasp_rotation_base()
        self.stages: list[Stage] = []
        self.samples: list[tuple[list[float], tuple[float, float], str]] = []
        self.sample_index = 0
        self.ready = False
        self.planning_started = False
        self.last_marker_publish = self.get_clock().now()

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)
        self.get_logger().info(
            "MoveIt grasp planning demo ready: synthetic target -> grasp_frame IK -> MoveIt -> RViz playback"
        )

    def _tick(self) -> None:
        self._publish_markers()
        if not self.planning_started:
            self.planning_started = True
            threading.Thread(target=self._build_plan, daemon=True).start()
            return
        if not self.ready:
            self._publish_joint_state(HOME, (0.0, 0.0))
            return
        if not self.samples:
            return
        joints, gripper, _label = self.samples[self.sample_index]
        self._publish_joint_state(joints, gripper)
        self.sample_index += 1
        if self.sample_index >= len(self.samples):
            if self.loop:
                self.sample_index = 0
            else:
                self.sample_index = len(self.samples) - 1

    def _build_plan(self) -> None:
        stages = self._make_grasp_frame_stages()
        if stages is None:
            return
        self.stages = stages

        self.get_logger().info("Waiting for MoveIt /plan_kinematic_path...")
        moveit_available = self.plan_client.wait_for_service(timeout_sec=8.0)
        if not moveit_available:
            message = "MoveIt planning service unavailable"
            if not self.allow_fallback:
                self.get_logger().error(message)
                return
            self.get_logger().warning(f"{message}; using interpolation fallback for RViz only")
            self.samples = self._fallback_samples()
            self.ready = True
            return

        samples: list[tuple[list[float], tuple[float, float], str]] = []
        current = list(self.stages[0].target)
        for stage in self.stages[1:]:
            if stage.label == "close" or stage.label == "drop_open":
                samples.extend(self._hold_samples(current, stage.gripper, stage.label, 0.8))
                continue
            trajectory = self._request_joint_plan(current, stage.target, stage.label)
            if trajectory is None:
                if not self.allow_fallback:
                    self.get_logger().error(f"Planning failed at stage {stage.label}")
                    return
                self.get_logger().warning(
                    f"Planning failed at stage {stage.label}; interpolating this segment"
                )
                samples.extend(self._interpolate(current, stage.target, stage.gripper, stage.label, 60))
            else:
                samples.extend(self._trajectory_samples(trajectory, stage.gripper, stage.label))
            current = list(stage.target)

        self.samples = samples or self._fallback_samples()
        self.ready = True
        self.get_logger().info(
            f"Prepared {len(self.samples)} playback frames. "
            "Green/blue markers show synthetic vision target and semantic waypoints."
        )

    def _make_grasp_frame_stages(self) -> list[Stage] | None:
        targets = [
            GraspTarget(
                "synthetic_yolo_lock",
                (TRASH_X - 0.12, TRASH_Y, TRASH_APPROACH_Z + 0.12),
                (0.0, 0.0),
            ),
            GraspTarget("approach", (TRASH_X - 0.04, TRASH_Y, TRASH_APPROACH_Z), (0.0, 0.0)),
            GraspTarget("grasp", (TRASH_X, TRASH_Y, TRASH_GRASP_Z), (0.0, 0.0)),
            GraspTarget("lift", (TRASH_X, TRASH_Y, TRASH_LIFT_Z), (0.62, -0.62)),
        ]
        stages = [Stage("home", list(HOME), (0.0, 0.0))]
        q_current = np.asarray(HOME, dtype=float)
        for target in targets:
            solved = self._solve_grasp_frame_target(q_current, target)
            if solved is None:
                if self.allow_fallback:
                    self.get_logger().warning(
                        f"IK failed at {target.label}; using interpolation fallback for RViz only"
                    )
                    return [
                        Stage("home", HOME, (0.0, 0.0)),
                        Stage("synthetic_yolo_lock", READY, (0.0, 0.0)),
                        Stage("approach", GRASP, (0.0, 0.0)),
                        Stage("grasp", GRASP, (0.0, 0.0)),
                        Stage("close", GRASP, (0.62, -0.62)),
                        Stage("lift", GRASP, (0.62, -0.62)),
                        Stage("basket_over", READY, (0.62, -0.62)),
                        Stage("drop_open", READY, (0.0, 0.0)),
                        Stage("return_home", HOME, (0.0, 0.0)),
                    ]
                self.get_logger().error(
                    f"IK failed at {target.label}; refusing to publish a fake grasp trajectory"
                )
                return None
            q_current = solved
            stages.append(Stage(target.label, [float(v) for v in q_current], target.gripper))
            if target.label == "grasp":
                stages.append(Stage("close", [float(v) for v in q_current], (0.62, -0.62)))

        release_target = GraspTarget(
            "basket_over", (BASKET_X, BASKET_Y, BASKET_OVER_Z), (0.62, -0.62)
        )
        release_joints = self._solve_release_frame_target(q_current, release_target)
        if release_joints is None:
            if self.allow_fallback:
                self.get_logger().warning(
                    "Release IK failed; using interpolation fallback for RViz only"
                )
                stages.append(Stage("basket_over", list(q_current), (0.62, -0.62)))
                stages.append(Stage("drop_open", list(q_current), (0.0, 0.0)))
                stages.append(Stage("return_home", list(HOME), (0.0, 0.0)))
                return stages
            self.get_logger().error(
                "Release IK failed; refusing to publish a fake basket trajectory"
            )
            return None
        q_current = release_joints
        stages.append(Stage("basket_over", [float(v) for v in q_current], (0.62, -0.62)))
        stages.append(Stage("drop_open", [float(v) for v in q_current], (0.0, 0.0)))
        stages.append(Stage("return_home", list(HOME), (0.0, 0.0)))
        return stages

    def _solve_grasp_frame_target(
        self, q_start: np.ndarray, target: GraspTarget
    ) -> np.ndarray | None:
        grasp_in_base = np.eye(4, dtype=np.float64)
        grasp_in_base[:3, :3] = self.grasp_rotation_base
        grasp_in_base[:3, 3] = np.asarray(target.grasp_xyz_base, dtype=float)
        tool_in_base = grasp_in_base @ self._invert_rigid(self.tool_to_grasp)
        tool_in_aubo = self.aubo_from_base @ tool_in_base
        ok, q_solution, position_error, orientation_error, iterations = self.kinematics.solve_pose(
            np.asarray(q_start, dtype=float),
            tool_in_aubo,
            position_tolerance=0.012,
            orientation_tolerance=0.35,
            damping=0.08,
            max_iterations=300,
            max_step=0.12,
            orientation_weight=0.25,
        )
        q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
        achieved = self._grasp_frame_in_base(q_goal)
        desired = np.asarray(target.grasp_xyz_base, dtype=float)
        grasp_error = float(np.linalg.norm(achieved[:3, 3] - desired))
        x, y, z = desired
        ax, ay, az = achieved[:3, 3]
        self.get_logger().info(
            f"IK {target.label}: desired grasp_frame=({x:.3f},{y:.3f},{z:.3f}) "
            f"achieved=({ax:.3f},{ay:.3f},{az:.3f}) "
            f"err={grasp_error:.3f}m pose_err={position_error:.3f}m "
            f"ori_err={orientation_error:.3f}rad iterations={iterations}"
        )
        if not ok or grasp_error > 0.025:
            return None
        return q_goal

    def _solve_release_frame_target(
        self, q_start: np.ndarray, target: GraspTarget
    ) -> np.ndarray | None:
        current_grasp = self._grasp_frame_in_base(q_start)
        target_rotations = [
            current_grasp[:3, :3],
            current_grasp[:3, :3] @ self._rpy_matrix(0.0, 0.0, math.radians(8.0)),
            current_grasp[:3, :3] @ self._rpy_matrix(0.0, 0.0, math.radians(-8.0)),
        ]
        best: tuple[float, np.ndarray, float, float, float] | None = None
        desired = np.asarray(target.grasp_xyz_base, dtype=float)
        for candidate_index, rotation in enumerate(target_rotations):
            grasp_in_base = np.eye(4, dtype=np.float64)
            grasp_in_base[:3, :3] = self._orthonormalize_rotation(rotation)
            grasp_in_base[:3, 3] = desired
            tool_in_aubo = self.aubo_from_base @ (grasp_in_base @ self._invert_rigid(self.tool_to_grasp))
            ok, q_solution, position_error, orientation_error, iterations = self.kinematics.solve_pose(
                np.asarray(q_start, dtype=float),
                tool_in_aubo,
                position_tolerance=0.020,
                orientation_tolerance=0.65,
                damping=0.08,
                max_iterations=320,
                max_step=0.08,
                orientation_weight=0.12,
            )
            q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
            achieved = self._grasp_frame_in_base(q_goal)
            grasp_error = float(np.linalg.norm(achieved[:3, 3] - desired))
            motion_cost = self._release_joint_motion_cost(q_goal, q_start)
            score = 180.0 * grasp_error + 2.0 * float(orientation_error) + motion_cost + 0.1 * candidate_index
            if ok and grasp_error <= 0.030 and (best is None or score < best[0]):
                best = (score, q_goal, grasp_error, float(orientation_error), motion_cost)
            self.get_logger().info(
                f"IK release candidate {candidate_index}: ok={ok} "
                f"err={grasp_error:.3f}m ori_err={orientation_error:.3f}rad "
                f"motion_cost={motion_cost:.3f} iterations={iterations}"
            )
        if best is None:
            return None
        _score, q_goal, grasp_error, orientation_error, motion_cost = best
        delta = np.abs(self._joint_delta(q_goal, q_start))
        self.get_logger().info(
            "IK basket_over selected: "
            f"desired grasp_frame=({desired[0]:.3f},{desired[1]:.3f},{desired[2]:.3f}) "
            f"err={grasp_error:.3f}m ori_err={orientation_error:.3f}rad "
            f"weighted_motion={motion_cost:.3f} "
            f"shoulder_elbow_delta={np.linalg.norm(delta[:3]):.3f}rad "
            f"wrist_delta={np.linalg.norm(delta[3:]):.3f}rad"
        )
        return q_goal

    def _grasp_frame_in_base(self, joints: np.ndarray) -> np.ndarray:
        return self.base_from_aubo @ self.kinematics.fk(np.asarray(joints, dtype=float)) @ self.tool_to_grasp

    def _nominal_grasp_rotation_base(self) -> np.ndarray:
        return self._grasp_frame_in_base(np.asarray(GRASP, dtype=float))[:3, :3]

    def _release_joint_motion_cost(self, target: np.ndarray, start: np.ndarray) -> float:
        delta = np.abs(self._joint_delta(np.asarray(target, dtype=float), np.asarray(start, dtype=float)))
        weights = np.asarray(
            [
                RELEASE_SHOULDER_ELBOW_WEIGHT,
                RELEASE_SHOULDER_ELBOW_WEIGHT,
                RELEASE_SHOULDER_ELBOW_WEIGHT * 0.8,
                RELEASE_WRIST_WEIGHT,
                RELEASE_WRIST_WEIGHT,
                RELEASE_WRIST_WEIGHT,
            ],
            dtype=float,
        )
        shoulder_elbow = float(np.linalg.norm(delta[:3]))
        wrist = float(np.linalg.norm(delta[3:]))
        return float(np.linalg.norm(delta * weights)) + RELEASE_CONFIGURATION_WEIGHT * shoulder_elbow + 0.15 * wrist

    def _joint_delta(self, target: np.ndarray, start: np.ndarray) -> np.ndarray:
        return (np.asarray(target, dtype=float) - np.asarray(start, dtype=float) + math.pi) % (
            2.0 * math.pi
        ) - math.pi

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

    def _request_joint_plan(
        self, start: list[float], target: list[float], label: str
    ) -> JointTrajectory | None:
        request = GetMotionPlan.Request()
        motion = MotionPlanRequest()
        motion.group_name = self.group_name
        motion.pipeline_id = "ompl"
        motion.planner_id = self.planner_id
        motion.num_planning_attempts = max(self.planning_attempts, 1)
        motion.allowed_planning_time = max(self.planning_time, 0.5)
        motion.max_velocity_scaling_factor = 0.35
        motion.max_acceleration_scaling_factor = 0.35
        motion.start_state = RobotState()
        motion.start_state.joint_state.name = list(ARM_JOINTS)
        motion.start_state.joint_state.position = list(start)
        constraints = Constraints()
        constraints.name = f"synthetic_{label}_joint_goal"
        for name, value in zip(ARM_JOINTS, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        motion.goal_constraints.append(constraints)
        request.motion_plan_request = motion

        future = self.plan_client.call_async(request)
        deadline = time.monotonic() + self.planning_time + 4.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            self.get_logger().warning(f"MoveIt request timed out for {label}")
            return None
        response = future.result().motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(
                f"MoveIt failed for {label}: error_code={response.error_code.val}"
            )
            return None
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            self.get_logger().warning(f"MoveIt returned an empty trajectory for {label}")
            return None
        self.get_logger().info(
            f"MoveIt planned {label}: {len(trajectory.points)} points, "
            f"planning_time={response.planning_time:.3f}s"
        )
        return trajectory

    def _trajectory_samples(
        self, trajectory: JointTrajectory, gripper: tuple[float, float], label: str
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        index_by_name = {name: i for i, name in enumerate(trajectory.joint_names)}
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        last_positions: list[float] | None = None
        for point in trajectory.points:
            positions = list(last_positions or HOME)
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
            steps = max(int(max(abs(x - y) for x, y in zip(a, b)) / 0.015 / self.playback_speed), 1)
            dense.extend(self._interpolate(a, b, gripper, label, steps))
        dense.append(samples[-1])
        return dense

    def _fallback_samples(self) -> list[tuple[list[float], tuple[float, float], str]]:
        samples: list[tuple[list[float], tuple[float, float], str]] = []
        current = list(self.stages[0].target)
        for stage in self.stages[1:]:
            steps = 45 if stage.label not in {"close", "drop_open"} else 20
            samples.extend(self._interpolate(current, stage.target, stage.gripper, stage.label, steps))
            current = list(stage.target)
        return samples

    def _hold_samples(
        self, joints: list[float], gripper: tuple[float, float], label: str, seconds: float
    ) -> list[tuple[list[float], tuple[float, float], str]]:
        count = max(int(seconds * 60.0 / self.playback_speed), 1)
        return [(list(joints), gripper, label) for _ in range(count)]

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
        now = self.get_clock().now()
        if (now.nanoseconds - self.last_marker_publish.nanoseconds) * 1e-9 < 0.2:
            return
        self.last_marker_publish = now
        markers = [
            self._ground_plane(),
            self._collision_box(
                20,
                "front_basket_collision_box",
                (0.5435, 0.0, -0.0300),
                (0.204, 0.180, 0.087),
                0.0,
                self._color(1.0, 0.35, 0.05, 0.24),
            ),
            self._collision_box(
                21,
                "rear_rack_collision_box",
                (-0.160, 0.0, 0.416),
                (0.274, 0.329, 0.622),
                math.pi / 2.0,
                self._color(1.0, 0.05, 0.05, 0.16),
            ),
            self._sphere(0, "synthetic_yolo_trash", TRASH_X, TRASH_Y, TRASH_GROUND_Z, 0.055, self._color(0.1, 0.9, 0.25, 0.9)),
            self._text(1, "YOLO trash 0.93\nsynthetic RGB-D lock\nbase z=-0.22m", TRASH_X, TRASH_Y, TRASH_GROUND_Z + 0.16),
            self._sphere(2, "approach", TRASH_X - 0.04, TRASH_Y, TRASH_APPROACH_Z, 0.035, self._color(0.1, 0.45, 1.0, 0.9)),
            self._sphere(3, "grasp", TRASH_X, TRASH_Y, TRASH_GRASP_Z, 0.035, self._color(1.0, 0.80, 0.1, 0.9)),
            self._sphere(4, "basket_over", BASKET_X, BASKET_Y, BASKET_OVER_Z, 0.045, self._color(0.9, 0.1, 0.75, 0.9)),
            self._line_strip(),
        ]
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _ground_plane(self) -> Marker:
        marker = self._base_marker(10, "base_ground_plane")
        marker.type = Marker.CUBE
        marker.pose.position.x = 0.36
        marker.pose.position.y = 0.0
        marker.pose.position.z = TRASH_GROUND_Z - 0.003
        marker.scale.x = 1.35
        marker.scale.y = 0.85
        marker.scale.z = 0.006
        marker.color = self._color(0.12, 0.15, 0.16, 0.55)
        return marker

    def _collision_box(
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
        self, marker_id: int, ns: str, x: float, y: float, z: float, scale: float, color: ColorRGBA
    ) -> Marker:
        marker = self._base_marker(marker_id, ns)
        marker.type = Marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def _text(self, marker_id: int, text: str, x: float, y: float, z: float) -> Marker:
        marker = self._base_marker(marker_id, "synthetic_detection_label")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.z = 0.055
        marker.color = self._color(0.95, 0.95, 0.95, 0.95)
        marker.text = text
        return marker

    def _line_strip(self) -> Marker:
        marker = self._base_marker(5, "semantic_task_path")
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.012
        marker.color = self._color(0.9, 0.1, 0.75, 0.85)
        for x, y, z in [
            (TRASH_X - 0.04, TRASH_Y, TRASH_APPROACH_Z),
            (TRASH_X, TRASH_Y, TRASH_GRASP_Z),
            (TRASH_X, TRASH_Y, TRASH_LIFT_Z),
            (BASKET_X, BASKET_Y, BASKET_OVER_Z),
        ]:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)
        return marker

    def _base_marker(self, marker_id: int, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=1)
        marker.pose.orientation.w = 1.0
        return marker

    def _color(self, r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveItGraspPlanningDemo()
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
