from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class BaseSimController(Node):
    def __init__(self) -> None:
        super().__init__("base_sim_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_state_topic", "/arachne/base/joint_states")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_timeout", 0.6)
        self.declare_parameter("wheel_radius", 0.16459)
        self.declare_parameter("track_width", 0.58306)
        self.declare_parameter("max_linear_velocity", 0.8)
        self.declare_parameter("max_angular_velocity", 1.4)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("reset_service", "/arachne/base/reset")

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        reset_service = self.get_parameter("reset_service").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_update = self.get_clock().now()
        self.last_command = self.last_update

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_service(Trigger, reset_service, self._reset_service)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)

        self.get_logger().info(
            "Base sim ready: "
            f"{self.cmd_vel_topic} -> {self.odom_topic}, {self.odom_frame}->{self.base_frame}"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.linear_velocity = self._clamp(
            msg.linear.x, -self.max_linear_velocity, self.max_linear_velocity
        )
        self.angular_velocity = self._clamp(
            msg.angular.z, -self.max_angular_velocity, self.max_angular_velocity
        )
        self.last_command = self.get_clock().now()

    def _reset_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_update = self.get_clock().now()
        self.last_command = self.last_update
        response.success = True
        response.message = "Reset base simulation pose"
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now.nanoseconds - self.last_update.nanoseconds) * 1e-9, 0.0)
        self.last_update = now

        if (now.nanoseconds - self.last_command.nanoseconds) * 1e-9 > self.command_timeout:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self._integrate(dt)
        self._publish_odom(now)
        self._publish_joint_states(now)
        if self.publish_tf:
            self._publish_tf(now)

    def _integrate(self, dt: float) -> None:
        if dt <= 0.0:
            return

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw = self._normalize_angle(self.yaw + self.angular_velocity * dt)

        left_linear = self.linear_velocity - self.angular_velocity * self.track_width * 0.5
        right_linear = self.linear_velocity + self.angular_velocity * self.track_width * 0.5
        self.left_wheel_position += left_linear / self.wheel_radius * dt
        self.right_wheel_position += right_linear / self.wheel_radius * dt

    def _publish_odom(self, now) -> None:
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(msg)

    def _publish_tf(self, now) -> None:
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.yaw * 0.5)
        transform.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_joint_states(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            "front_left_wheel",
            "rear_left_wheel",
            "front_right_wheel",
            "rear_right_wheel",
        ]
        msg.position = [
            self.left_wheel_position,
            self.left_wheel_position,
            self.right_wheel_position,
            self.right_wheel_position,
        ]
        self.joint_pub.publish(msg)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BaseSimController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
