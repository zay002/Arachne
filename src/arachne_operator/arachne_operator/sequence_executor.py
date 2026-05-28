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
class TaskStep:
    kind: str
    value: str = ""
    duration: float = 0.0
    timeout: float = 0.0
    pose: PoseStamped | None = None


class SequenceExecutor(Node):
    def __init__(self) -> None:
        super().__init__("arachne_sequence_executor")
        self.declare_parameter("arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory")
        self.declare_parameter(
            "legacy_arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/arachne/sequence/command")
        self.declare_parameter("nav_goal_topic", "/arachne/navigation/goal")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("arm_motion_time", 2.5)
        self.declare_parameter("gripper_motion_time", 0.8)
        self.declare_parameter("nav_wait_timeout", 1.0)
        self.declare_parameter("nav_goal_timeout", 35.0)

        self.arm_motion_time = float(self.get_parameter("arm_motion_time").value)
        self.gripper_motion_time = float(self.get_parameter("gripper_motion_time").value)
        self.nav_wait_timeout = float(self.get_parameter("nav_wait_timeout").value)
        self.nav_goal_timeout = float(self.get_parameter("nav_goal_timeout").value)

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
            self._create_trigger(f"/arachne/sequence/{preset}", lambda name=preset: self._start_arm_task(name))
        self._create_trigger("/arachne/sequence/open_gripper", lambda: self._start_gripper_task("open"))
        self._create_trigger("/arachne/sequence/close_gripper", lambda: self._start_gripper_task("close"))
        self._create_trigger("/arachne/sequence/stop", self._stop)
        self._create_trigger("/arachne/sequence/demo_pick", self._demo_pick)
        self._create_trigger("/arachne/sequence/demo_nav_pick", self._demo_nav_pick)

        self.task_name = "idle"
        self.task_queue: deque[TaskStep] = deque()
        self.current_step: TaskStep | None = None
        self.step_ready_ns = 0
        self.step_timeout_ns = 0
        self.nav_goal_handle = None
        self.nav_done = False
        self.nav_success = False
        self.nav_result_text = ""
        self.nav_request_id = 0

        self.create_timer(0.05, self._task_tick)

        self._status(
            "idle: commands home/ready/reach/grasp/lift/open/close/stop/demo_pick/"
            "demo_nav_pick/goto x y yaw"
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
            self._start_arm_task(verb)
        elif verb == "arm" and len(tokens) >= 2:
            self._start_arm_task(tokens[1].lower())
        elif verb in ("open", "close"):
            self._start_gripper_task(verb)
        elif verb == "stop":
            self._stop()
        elif verb in ("demo_pick", "pick_demo"):
            self._demo_pick()
        elif verb in ("demo_nav_pick", "nav_pick"):
            self._demo_nav_pick()
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

        pose = self._make_nav_pose(x, y, yaw)
        return self._start_task("goto", [TaskStep("nav", pose=pose, timeout=self.nav_goal_timeout)])

    def _nav_goal_callback(self, msg: PoseStamped) -> None:
        self._start_task("external_nav_goal", [TaskStep("nav", pose=msg, timeout=self.nav_goal_timeout)])

    def _start_arm_task(self, preset: str) -> tuple[bool, str]:
        if preset not in ARM_PRESETS:
            return self._status(f"unknown arm preset: {preset}", warn=True)
        return self._start_task(
            f"arm:{preset}",
            [TaskStep("arm", preset, duration=self.arm_motion_time, timeout=self.arm_motion_time + 1.0)],
        )

    def _start_gripper_task(self, command: str) -> tuple[bool, str]:
        return self._start_task(
            f"gripper:{command}",
            [
                TaskStep(
                    "gripper",
                    command,
                    duration=self.gripper_motion_time,
                    timeout=self.gripper_motion_time + 0.5,
                )
            ],
        )

    def _demo_pick(self) -> tuple[bool, str]:
        return self._start_task(
            "demo_pick",
            [
                TaskStep("gripper", "open", duration=0.5, timeout=1.0),
                TaskStep("arm", "ready", duration=2.5, timeout=3.5),
                TaskStep("arm", "reach", duration=2.5, timeout=3.5),
                TaskStep("arm", "grasp", duration=1.5, timeout=2.5),
                TaskStep("gripper", "close", duration=0.8, timeout=1.5),
                TaskStep("arm", "lift", duration=2.5, timeout=3.5),
                TaskStep("arm", "home", duration=2.5, timeout=3.5),
            ],
        )

    def _demo_nav_pick(self) -> tuple[bool, str]:
        return self._start_task(
            "demo_nav_pick",
            [
                TaskStep("nav", pose=self._make_nav_pose(0.8, 0.0, 0.0), timeout=self.nav_goal_timeout),
                TaskStep("gripper", "open", duration=0.5, timeout=1.0),
                TaskStep("arm", "ready", duration=2.5, timeout=3.5),
                TaskStep("arm", "reach", duration=2.5, timeout=3.5),
                TaskStep("arm", "grasp", duration=1.5, timeout=2.5),
                TaskStep("gripper", "close", duration=0.8, timeout=1.5),
                TaskStep("arm", "lift", duration=2.5, timeout=3.5),
                TaskStep("arm", "home", duration=2.5, timeout=3.5),
            ],
        )

    def _start_task(self, name: str, steps: list[TaskStep]) -> tuple[bool, str]:
        self._cancel_current_nav()
        self.task_name = name
        self.task_queue = deque(steps)
        self.current_step = None
        self.step_ready_ns = 0
        self.step_timeout_ns = 0
        self.nav_done = False
        self.nav_success = False
        self.nav_result_text = ""
        return self._status(f"task started: {name} steps={len(steps)}")

    def _task_tick(self) -> None:
        if self.current_step is None:
            if not self.task_queue:
                return
            self._start_next_step()
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.step_timeout_ns and now_ns > self.step_timeout_ns:
            self._fail_task(f"step timeout: {self.current_step.kind} {self.current_step.value}")
            return

        if self.current_step.kind == "nav":
            if not self.nav_done:
                return
            if self.nav_success:
                self._finish_step(self.nav_result_text or "nav complete")
            else:
                self._fail_task(self.nav_result_text or "nav failed")
            return

        if self.current_step.kind in ("arm", "gripper", "wait"):
            if self.step_ready_ns and now_ns >= self.step_ready_ns:
                self._finish_step(f"{self.current_step.kind} complete")

    def _start_next_step(self) -> None:
        self.current_step = self.task_queue.popleft()
        step = self.current_step
        now_ns = self.get_clock().now().nanoseconds
        self.step_ready_ns = 0
        self.step_timeout_ns = now_ns + int(max(step.timeout or step.duration, 0.1) * 1e9)
        self.nav_done = False
        self.nav_success = False
        self.nav_result_text = ""

        self._status(f"task {self.task_name}: start {step.kind} {step.value}".strip())
        if step.kind == "arm":
            ok, text = self._publish_arm(step.value)
            if not ok:
                self._fail_task(text)
                return
            self.step_ready_ns = now_ns + int(max(step.duration, 0.1) * 1e9)
        elif step.kind == "gripper":
            self._publish_gripper(step.value)
            self.step_ready_ns = now_ns + int(max(step.duration, 0.1) * 1e9)
        elif step.kind == "wait":
            self.step_ready_ns = now_ns + int(max(step.duration, 0.1) * 1e9)
        elif step.kind == "nav":
            self.step_timeout_ns = now_ns + int(max(step.timeout, 0.1) * 1e9)
            self._send_nav_goal(step)
        else:
            self._fail_task(f"unknown task step kind: {step.kind}")

    def _send_nav_goal(self, step: TaskStep) -> None:
        if step.pose is None:
            self._fail_task("nav step missing pose")
            return
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_wait_timeout):
            self._fail_task("Nav2 navigate_to_pose action is not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose = step.pose
        self.nav_request_id += 1
        request_id = self.nav_request_id
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result, nav_id=request_id: self._nav_goal_response(result, nav_id))
        pose = step.pose.pose.position
        self._status(f"sent nav goal x={pose.x:.2f} y={pose.y:.2f}")

    def _nav_goal_response(self, future, request_id: int) -> None:
        if request_id != self.nav_request_id:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            self.nav_done = True
            self.nav_success = False
            self.nav_result_text = f"Nav2 goal request failed: {exc}"
            return

        if goal_handle is None or not goal_handle.accepted:
            self.nav_done = True
            self.nav_success = False
            self.nav_result_text = "Nav2 goal rejected"
            return

        self.nav_goal_handle = goal_handle
        self._status("Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result, nav_id=request_id: self._nav_result(result, nav_id))

    def _nav_result(self, future, request_id: int) -> None:
        if request_id != self.nav_request_id:
            return
        try:
            result = future.result().result
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            self.nav_done = True
            self.nav_success = False
            self.nav_result_text = f"Nav2 result failed: {exc}"
            return

        self.nav_goal_handle = None
        self.nav_done = True
        self.nav_success = result.error_code == NavigateToPose.Result.NONE
        if self.nav_success:
            self.nav_result_text = "Nav2 goal complete"
        else:
            self.nav_result_text = f"Nav2 goal failed: {result.error_code} {result.error_msg}"

    def _publish_arm(self, preset: str) -> tuple[bool, str]:
        if preset not in ARM_PRESETS:
            return False, f"unknown arm preset: {preset}"

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
        return True, f"arm preset: {preset}"

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _finish_step(self, text: str) -> None:
        self._status(f"task {self.task_name}: {text}")
        self.current_step = None
        self.step_ready_ns = 0
        self.step_timeout_ns = 0
        if not self.task_queue:
            finished = self.task_name
            self.task_name = "idle"
            self._status(f"task complete: {finished}")

    def _fail_task(self, reason: str) -> None:
        failed = self.task_name
        self._cancel_current_nav()
        self.task_name = "idle"
        self.task_queue.clear()
        self.current_step = None
        self.step_ready_ns = 0
        self.step_timeout_ns = 0
        self.cmd_vel_pub.publish(Twist())
        self._status(f"task failed: {failed}: {reason}", warn=True)

    def _stop(self) -> tuple[bool, str]:
        stopped = self.task_name
        self._cancel_current_nav()
        self.task_name = "idle"
        self.task_queue.clear()
        self.current_step = None
        self.step_ready_ns = 0
        self.step_timeout_ns = 0
        self.cmd_vel_pub.publish(Twist())
        self._publish_gripper("stop")
        return self._status(f"task stopped: {stopped}")

    def _cancel_current_nav(self) -> None:
        if self.nav_goal_handle is None:
            return
        try:
            self.nav_goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - defensive ROS callback guard
            self.get_logger().warning(f"failed to cancel nav goal: {exc}")
        self.nav_request_id += 1
        self.nav_goal_handle = None
        self.cmd_vel_pub.publish(Twist())

    def _make_nav_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

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
