from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory


ARM_JOINTS = (
    "aubo_shoulder_joint",
    "aubo_upperArm_joint",
    "aubo_foreArm_joint",
    "aubo_wrist1_joint",
    "aubo_wrist2_joint",
    "aubo_wrist3_joint",
)

HOME = {
    "aubo_shoulder_joint": 1.664,
    "aubo_upperArm_joint": 0.034,
    "aubo_foreArm_joint": -1.324,
    "aubo_wrist1_joint": 0.034,
    "aubo_wrist2_joint": -1.732,
    "aubo_wrist3_joint": 0.0,
}


class ArachneHardwareMock(Node):
    def __init__(self) -> None:
        super().__init__("arachne_hardware_mock")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("wheel_radius", 0.16459)
        self.declare_parameter("track_width", 0.58306)

        self.period = 1.0 / max(float(self.get_parameter("publish_rate").value), 1.0)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.arm_positions = dict(HOME)
        self.gripper_position = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.base_status_pub = self.create_publisher(String, "/arachne/hardware/base_status", 10)
        self.arm_status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        self.gripper_status_pub = self.create_publisher(
            String, "/arachne/hardware/gripper_status", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", self._on_arm, 10
        )
        self.create_subscription(
            JointTrajectory, "/aubo_arm_controller/joint_trajectory", self._on_arm, 10
        )
        self.create_subscription(String, "/arachne/gripper/command", self._on_gripper, 10)
        self.create_timer(self.period, self._tick)
        self.get_logger().info("Arachne hardware mock ready")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.vx = max(min(msg.linear.x, 0.8), -0.4)
        self.wz = max(min(msg.angular.z, 1.4), -1.4)

    def _on_arm(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        point = msg.points[-1]
        for index, name in enumerate(msg.joint_names):
            if name in self.arm_positions and index < len(point.positions):
                self.arm_positions[name] = float(point.positions[index])

    def _on_gripper(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "open":
            self.gripper_position = 0.0
        elif command == "close":
            self.gripper_position = 0.6

    def _tick(self) -> None:
        self.x += self.vx * math.cos(self.yaw) * self.period
        self.y += self.vx * math.sin(self.yaw) * self.period
        self.yaw = math.atan2(
            math.sin(self.yaw + self.wz * self.period),
            math.cos(self.yaw + self.wz * self.period),
        )
        left_linear = self.vx - self.wz * self.track_width * 0.5
        right_linear = self.vx + self.wz * self.track_width * 0.5
        self.left_wheel += left_linear / self.wheel_radius * self.period
        self.right_wheel += right_linear / self.wheel_radius * self.period

        now = self.get_clock().now()
        self._publish_odom(now)
        self._publish_joint_states(now)
        self._publish_status()

    def _publish_odom(self, now) -> None:
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        self.odom_pub.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_joint_states(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            *ARM_JOINTS,
            "front_left_wheel",
            "rear_left_wheel",
            "front_right_wheel",
            "rear_right_wheel",
            "ms42dc_left_finger_joint",
            "ms42dc_right_finger_joint",
        ]
        msg.position = [
            *(self.arm_positions[name] for name in ARM_JOINTS),
            self.left_wheel,
            self.left_wheel,
            self.right_wheel,
            self.right_wheel,
            self.gripper_position,
            -self.gripper_position,
        ]
        self.joint_pub.publish(msg)

    def _publish_status(self) -> None:
        base = String()
        base.data = f"mock odom x={self.x:.2f} y={self.y:.2f} vx={self.vx:.2f} wz={self.wz:.2f}"
        self.base_status_pub.publish(base)

        arm = String()
        arm.data = "mock aubo ready"
        self.arm_status_pub.publish(arm)

        gripper = String()
        gripper.data = f"mock ms42dc position={self.gripper_position:.2f}"
        self.gripper_status_pub.publish(gripper)


def main() -> None:
    rclpy.init()
    node = ArachneHardwareMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
