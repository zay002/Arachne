from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class ButtonEdges:
    pressed: set[int]
    released: set[int]


class SwitchTeleop(Node):
    def __init__(self) -> None:
        super().__init__("switch_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("camera_yaw_topic", "/arachne/camera_yaw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("arm_joint_state_topic", "/arachne/gui_joint_states")
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("axis_curve", 0.45)
        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 0)
        self.declare_parameter("forward_axis_multiplier", -1.0)
        self.declare_parameter("lateral_axis_multiplier", 1.0)
        self.declare_parameter("drive_mode", "body")
        self.declare_parameter("linear_scale", 0.55)
        self.declare_parameter("angular_scale", 1.1)
        self.declare_parameter("heading_gain", 3.2)
        self.declare_parameter("linear_response", 14.0)
        self.declare_parameter("angular_response", 18.0)
        self.declare_parameter("reverse_switch_angle", 1.57079632679)
        self.declare_parameter("turn_in_place_angle", 1.05)
        self.declare_parameter("turn_in_place_scale", 0.12)
        self.declare_parameter("turbo_multiplier", 1.6)
        self.declare_parameter("joint_velocity_scale", 0.85)
        self.declare_parameter("turbo_button", 7)
        self.declare_parameter("arm_enable_button", 6)
        self.declare_parameter("arm_positive_button", 12)
        self.declare_parameter("arm_negative_button", 13)
        self.declare_parameter("previous_joint_button", 4)
        self.declare_parameter("next_joint_button", 5)
        self.declare_parameter("open_button", 0)
        self.declare_parameter("close_button", 1)
        self.declare_parameter("reset_pose_button", 9)
        self.declare_parameter("stop_button", 8)
        self.declare_parameter("reset_topic", "/arachne/demo/reset")
        self.declare_parameter("base_reset_service", "/arachne/base/reset")
        self.declare_parameter(
            "joint_names",
            [
                "aubo_shoulder_joint",
                "aubo_upperArm_joint",
                "aubo_foreArm_joint",
                "aubo_wrist1_joint",
                "aubo_wrist2_joint",
                "aubo_wrist3_joint",
            ],
        )
        self.declare_parameter(
            "default_positions",
            [
                -1.5707963267949,
                0.201570428261868,
                1.65970467002488,
                0.485178041391533,
                1.67675136677345,
                0.76432946885334,
            ],
        )
        self.declare_parameter(
            "lower_limits",
            [-6.283185307179586] * 6,
        )
        self.declare_parameter(
            "upper_limits",
            [6.283185307179586] * 6,
        )

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        camera_yaw_topic = self.get_parameter("camera_yaw_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        arm_topic = self.get_parameter("arm_joint_state_topic").value
        gripper_topic = self.get_parameter("gripper_command_topic").value
        reset_topic = str(self.get_parameter("reset_topic").value)
        base_reset_service = str(self.get_parameter("base_reset_service").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.axis_curve = float(self.get_parameter("axis_curve").value)
        self.linear_axis = int(self.get_parameter("linear_axis").value)
        self.angular_axis = int(self.get_parameter("angular_axis").value)
        self.forward_axis_multiplier = float(self.get_parameter("forward_axis_multiplier").value)
        self.lateral_axis_multiplier = float(self.get_parameter("lateral_axis_multiplier").value)
        self.drive_mode = str(self.get_parameter("drive_mode").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.linear_response = float(self.get_parameter("linear_response").value)
        self.angular_response = float(self.get_parameter("angular_response").value)
        self.reverse_switch_angle = float(self.get_parameter("reverse_switch_angle").value)
        self.turn_in_place_angle = float(self.get_parameter("turn_in_place_angle").value)
        self.turn_in_place_scale = float(self.get_parameter("turn_in_place_scale").value)
        self.turbo_multiplier = float(self.get_parameter("turbo_multiplier").value)
        self.joint_velocity_scale = float(self.get_parameter("joint_velocity_scale").value)
        self.turbo_button = int(self.get_parameter("turbo_button").value)
        self.arm_enable_button = int(self.get_parameter("arm_enable_button").value)
        self.arm_positive_button = int(self.get_parameter("arm_positive_button").value)
        self.arm_negative_button = int(self.get_parameter("arm_negative_button").value)
        self.previous_joint_button = int(self.get_parameter("previous_joint_button").value)
        self.next_joint_button = int(self.get_parameter("next_joint_button").value)
        self.open_button = int(self.get_parameter("open_button").value)
        self.close_button = int(self.get_parameter("close_button").value)
        self.reset_pose_button = int(self.get_parameter("reset_pose_button").value)
        self.stop_button = int(self.get_parameter("stop_button").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.default_positions = [float(value) for value in self.get_parameter("default_positions").value]
        self.lower_limits = [float(value) for value in self.get_parameter("lower_limits").value]
        self.upper_limits = [float(value) for value in self.get_parameter("upper_limits").value]

        if len(self.default_positions) != len(self.joint_names):
            raise ValueError("default_positions must match joint_names")

        self.positions = list(self.default_positions)
        self.selected_joint_index = 0
        self.last_joy: Joy | None = None
        self.last_joy_time = self.get_clock().now()
        self.last_buttons: list[int] = []
        self.last_update = self.get_clock().now()
        self.active_twist = Twist()
        self.camera_yaw = 0.0
        self.base_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.arm_pub = self.create_publisher(JointState, arm_topic, 10)
        self.gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self.reset_pub = self.create_publisher(Empty, reset_topic, 10)
        self.base_reset_client = self.create_client(Trigger, base_reset_service)
        self.create_subscription(Joy, "joy", self._joy_callback, 10)
        self.create_subscription(Float64, camera_yaw_topic, self._camera_yaw_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)

        self.get_logger().info(
            "Switch teleop ready. Left stick uses polar arcade drive in robot coordinates; "
            "right stick controls the demo camera; "
            "hold ZL + D-pad up/down moves the selected Aubo joint; L/R select joint; "
            "B opens, A closes."
        )
        self._log_selected_joint()

    def _joy_callback(self, msg: Joy) -> None:
        edges = self._button_edges(msg.buttons)
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()
        self.last_buttons = list(msg.buttons)

        if self.previous_joint_button in edges.pressed:
            self._select_joint(self.selected_joint_index - 1)
        if self.next_joint_button in edges.pressed:
            self._select_joint(self.selected_joint_index + 1)
        if self.open_button in edges.pressed:
            self._publish_gripper_command("open")
        if self.close_button in edges.pressed:
            self._publish_gripper_command("close")
        if self.reset_pose_button in edges.pressed:
            self._reset_demo_pose()
        if self.stop_button in edges.pressed:
            self.active_twist = Twist()
            self.cmd_pub.publish(self.active_twist)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now.nanoseconds - self.last_update.nanoseconds) * 1e-9, 0.0)
        self.last_update = now

        joy = self.last_joy
        joy_age = (now.nanoseconds - self.last_joy_time.nanoseconds) * 1e-9
        if joy is None or joy_age > self.joy_timeout:
            self.active_twist = Twist()
            self.cmd_pub.publish(self.active_twist)
            self._publish_arm(now)
            return

        turbo = self._button(joy, self.turbo_button)
        speed_multiplier = self.turbo_multiplier if turbo else 1.0

        twist = self._drive_twist(joy, speed_multiplier)
        self.active_twist = self._smooth_twist(twist, dt)
        self.cmd_pub.publish(self.active_twist)

        if self._button(joy, self.arm_enable_button):
            self._apply_arm_buttons(joy, dt, speed_multiplier)

        self._publish_arm(now)

    def _camera_yaw_callback(self, msg: Float64) -> None:
        self.camera_yaw = float(msg.data)

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.base_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _drive_twist(self, joy: Joy, speed_multiplier: float) -> Twist:
        if self.drive_mode != "third_person":
            return self._body_drive_twist(joy, speed_multiplier)

        forward = self.forward_axis_multiplier * self._axis(joy, self.linear_axis)
        lateral = self.lateral_axis_multiplier * self._axis(joy, self.angular_axis)
        magnitude = min(math.hypot(forward, lateral), 1.0)
        twist = Twist()
        if magnitude <= 0.0:
            return twist

        desired_yaw = self.camera_yaw + math.atan2(-lateral, forward)
        yaw_error = self._normalize_angle(desired_yaw - self.base_yaw)
        drive_sign = 1.0
        if abs(yaw_error) > self.reverse_switch_angle:
            drive_sign = -1.0
            yaw_error = self._normalize_angle(yaw_error - math.copysign(math.pi, yaw_error))

        alignment = max(math.cos(yaw_error), 0.0)
        linear_scale = alignment
        if abs(yaw_error) > self.turn_in_place_angle:
            linear_scale *= self.turn_in_place_scale

        twist.linear.x = drive_sign * magnitude * self.linear_scale * linear_scale * speed_multiplier
        twist.angular.z = self._clamp(
            yaw_error * self.heading_gain,
            -self.angular_scale * speed_multiplier,
            self.angular_scale * speed_multiplier,
        )
        return twist

    def _body_drive_twist(self, joy: Joy, speed_multiplier: float) -> Twist:
        forward_raw = self.forward_axis_multiplier * self._raw_axis(joy, self.linear_axis)
        lateral_raw = self.lateral_axis_multiplier * self._raw_axis(joy, self.angular_axis)
        raw_radius = math.hypot(forward_raw, lateral_raw)

        twist = Twist()
        if raw_radius < self.deadzone:
            return twist

        direction_radius = max(raw_radius, 1e-6)
        speed_radius = min((min(raw_radius, 1.0) - self.deadzone) / max(1.0 - self.deadzone, 1e-6), 1.0)
        speed_radius = (1.0 - self.axis_curve) * speed_radius + self.axis_curve * speed_radius * speed_radius

        forward_unit = forward_raw / direction_radius
        lateral_unit = lateral_raw / direction_radius
        twist.linear.x = forward_unit * speed_radius * self.linear_scale * speed_multiplier
        twist.angular.z = lateral_unit * speed_radius * self.angular_scale * speed_multiplier
        return twist

    def _smooth_twist(self, target: Twist, dt: float) -> Twist:
        if dt <= 0.0:
            return target
        linear_alpha = 1.0 - math.exp(-self.linear_response * dt)
        angular_alpha = 1.0 - math.exp(-self.angular_response * dt)
        twist = Twist()
        twist.linear.x = self.active_twist.linear.x + (target.linear.x - self.active_twist.linear.x) * linear_alpha
        twist.angular.z = self.active_twist.angular.z + (target.angular.z - self.active_twist.angular.z) * angular_alpha
        return twist

    def _apply_arm_buttons(self, joy: Joy, dt: float, speed_multiplier: float) -> None:
        value = 0.0
        if self._button(joy, self.arm_positive_button):
            value += 1.0
        if self._button(joy, self.arm_negative_button):
            value -= 1.0
        if abs(value) <= 0.0 or dt <= 0.0:
            return

        index = self.selected_joint_index
        next_position = self.positions[index] + value * self.joint_velocity_scale * speed_multiplier * dt
        self.positions[index] = min(max(next_position, self.lower_limits[index]), self.upper_limits[index])

    def _publish_arm(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.arm_pub.publish(msg)

    def _publish_gripper_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command: {command}")

    def _reset_demo_pose(self) -> None:
        self.positions = list(self.default_positions)
        self.active_twist = Twist()
        self.cmd_pub.publish(self.active_twist)
        self.reset_pub.publish(Empty())

        if self.base_reset_client.service_is_ready():
            self.base_reset_client.call_async(Trigger.Request())

        self.get_logger().info("Reset demo pose: base, Aubo arm, and Gazebo demo state")

    def _select_joint(self, index: int) -> None:
        self.selected_joint_index = index % len(self.joint_names)
        self._log_selected_joint()

    def _log_selected_joint(self) -> None:
        joint = self.joint_names[self.selected_joint_index]
        self.get_logger().info(f"Selected arm joint: {joint}")

    def _axis(self, joy: Joy, index: int) -> float:
        value = self._raw_axis(joy, index)
        magnitude = abs(value)
        if magnitude < self.deadzone:
            return 0.0
        scaled = min((magnitude - self.deadzone) / max(1.0 - self.deadzone, 1e-6), 1.0)
        curved = (1.0 - self.axis_curve) * scaled + self.axis_curve * scaled * scaled
        return math.copysign(curved, value)

    def _raw_axis(self, joy: Joy, index: int) -> float:
        if index < 0 or index >= len(joy.axes):
            return 0.0
        return self._clamp(float(joy.axes[index]), -1.0, 1.0)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _button(self, joy: Joy, index: int) -> bool:
        if index < 0 or index >= len(joy.buttons):
            return False
        return bool(joy.buttons[index])

    def _button_edges(self, buttons: list[int]) -> ButtonEdges:
        previous = self.last_buttons
        pressed = set()
        released = set()
        for index, value in enumerate(buttons):
            was_pressed = index < len(previous) and bool(previous[index])
            is_pressed = bool(value)
            if is_pressed and not was_pressed:
                pressed.add(index)
            elif was_pressed and not is_pressed:
                released.add(index)
        return ButtonEdges(pressed=pressed, released=released)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SwitchTeleop()
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
