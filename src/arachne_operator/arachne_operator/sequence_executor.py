from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = (
    "aubo_shoulder_joint",
    "aubo_upperArm_joint",
    "aubo_foreArm_joint",
    "aubo_wrist1_joint",
    "aubo_wrist2_joint",
    "aubo_wrist3_joint",
)

ARM_PRESETS = {
    "home": (1.664, 0.034, -1.324, 0.034, -1.732, 0.0),
    "ready": (1.50, -0.30, -1.10, 0.40, -1.55, 0.0),
    "reach": (1.35, -0.65, -0.85, 0.55, -1.35, 0.0),
    "grasp": (1.32, -0.78, -0.72, 0.35, -1.32, 0.0),
    "lift": (1.60, -0.20, -1.25, 0.35, -1.55, 0.0),
}


@dataclass(frozen=True)
class SequenceStep:
    kind: str
    value: str
    delay_after: float


class SequenceExecutor(Node):
    def __init__(self) -> None:
        super().__init__("arachne_sequence_executor")
        self.declare_parameter("arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory")
        self.declare_parameter("legacy_arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/arachne/sequence/command")
        self.declare_parameter("nav_goal_topic", "/arachne/navigation/goal")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("arm_motion_time", 2.5)
        self.declare_parameter("nav_wait_timeout", 1.0)

        self.arm_motion_time = float(self.get_parameter("arm_motion_time").value)
        self.nav_wait_timeout = float(self.get_parameter("nav_wait_timeout").value)

        arm_topic = self.get_parameter("arm_trajectory_topic").value
        legacy_arm_topic = self.get_parameter("legacy_arm_trajectory_topic").value
        gripper_topic = self.get_parameter("gripper_command_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        command_topic = self.get_parameter("command_topic").value
        nav_goal_topic = self.get_parameter("nav_goal_topic").value
        nav_action_name = self.get_parameter("nav_action_name").value

        self.arm_publishers = [
            self.create_publisher(JointTrajectory, arm_topic, 10),
            self.create_publisher(JointTrajectory, legacy_arm_topic, 10),
        ]
        self.gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, "/arachne/sequence/status", 10)
        self.nav_client = ActionClient(self, NavigateToPose, nav_action_name)

        self.create_subscription(String, command_topic, self._command_callback, 10)
        self.create_subscription(PoseStamped, nav_goal_topic, self._nav_goal_callback, 10)

        for preset in ARM_PRESETS:
            self._create_trigger(f"/arachne/sequence/{preset}", lambda name=preset: self._arm(preset=name))
        self._create_trigger("/arachne/sequence/open_gripper", lambda: self._gripper("open"))
        self._create_trigger("/arachne/sequence/close_gripper", lambda: self._gripper("close"))
        self._create_trigger("/arachne/sequence/stop", self._stop)
        self._create_trigger("/arachne/sequence/demo_pick", self._demo_pick)

        self.sequence: deque[SequenceStep] = deque()
        self.next_step_time_ns = 0
        self.create_timer(0.05, self._sequence_tick)

        self._status(
            "ready: commands home/ready/reach/grasp/lift/open/close/stop/demo_pick/"
            "goto x y yaw"
        )

    def _create_trigger(self, name: str, handler: Callable[[], tuple[bool, str]]) -> None:
        def callback(_request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            response.success, response.message = handler()
            return response

        self.create_service(Trigger, name, callback)

    def _command_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return

        tokens = command.split()
        verb = tokens[0].lower()
        if verb in ARM_PRESETS:
            self._arm(verb)
        elif verb == "arm" and len(tokens) >= 2:
            self._arm(tokens[1].lower())
        elif verb in ("open", "close", "stop"):
            if verb == "stop":
                self._stop()
            else:
                self._gripper(verb)
        elif verb in ("demo_pick", "pick_demo"):
            self._demo_pick()
        elif verb in ("goto", "nav"):
            self._command_nav_goal(tokens)
        else:
            self._status(f"ignored unknown command: {command}", warn=True)

    def _command_nav_goal(self, tokens: list[str]) -> tuple[bool, str]:
        if len(tokens) < 3:
            return self._status("goto requires: goto x y [yaw]", warn=True)
        try:
            x = float(tokens[1])
            y = float(tokens[2])
            yaw = float(tokens[3]) if len(tokens) >= 4 else 0.0
        except ValueError:
            return self._status("goto arguments must be numeric", warn=True)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return self._send_nav_goal(goal)

    def _nav_goal_callback(self, msg: PoseStamped) -> None:
        self._send_nav_goal(msg)

    def _send_nav_goal(self, pose: PoseStamped) -> tuple[bool, str]:
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_wait_timeout):
            return self._status("Nav2 navigate_to_pose action is not available", warn=True)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)
        return self._status(
            f"sent nav goal x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}"
        )

    def _nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._status("Nav2 goal rejected", warn=True)
            return
        self._status("Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future) -> None:
        result = future.result().result
        if result.error_code == NavigateToPose.Result.NONE:
            self._status("Nav2 goal complete")
        else:
            self._status(f"Nav2 goal failed: {result.error_code} {result.error_msg}", warn=True)

    def _arm(self, preset: str) -> tuple[bool, str]:
        if preset not in ARM_PRESETS:
            return self._status(f"unknown arm preset: {preset}", warn=True)

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(ARM_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = list(ARM_PRESETS[preset])
        point.time_from_start.sec = int(self.arm_motion_time)
        point.time_from_start.nanosec = int((self.arm_motion_time % 1.0) * 1e9)
        trajectory.points = [point]

        for publisher in self.arm_publishers:
            publisher.publish(trajectory)
        return self._status(f"arm preset: {preset}")

    def _gripper(self, command: str) -> tuple[bool, str]:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        return self._status(f"gripper: {command}")

    def _stop(self) -> tuple[bool, str]:
        self.sequence.clear()
        self.next_step_time_ns = 0
        self.cmd_vel_pub.publish(Twist())
        self._gripper("stop")
        return self._status("sequence stopped")

    def _demo_pick(self) -> tuple[bool, str]:
        self.sequence = deque(
            [
                SequenceStep("gripper", "open", 0.5),
                SequenceStep("arm", "ready", 2.5),
                SequenceStep("arm", "reach", 2.5),
                SequenceStep("arm", "grasp", 1.5),
                SequenceStep("gripper", "close", 0.8),
                SequenceStep("arm", "lift", 2.5),
                SequenceStep("arm", "home", 2.5),
            ]
        )
        self.next_step_time_ns = 0
        return self._status("demo_pick started")

    def _sequence_tick(self) -> None:
        if not self.sequence:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.next_step_time_ns and now_ns < self.next_step_time_ns:
            return

        step = self.sequence.popleft()
        if step.kind == "arm":
            self._arm(step.value)
        elif step.kind == "gripper":
            self._gripper(step.value)
        self.next_step_time_ns = now_ns + int(step.delay_after * 1e9)

    def _status(self, text: str, warn: bool = False) -> tuple[bool, str]:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
            return False, text
        self.get_logger().info(text)
        return True, text


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SequenceExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
