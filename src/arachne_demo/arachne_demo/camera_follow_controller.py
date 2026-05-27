from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class CameraFollowController(Node):
    def __init__(self) -> None:
        super().__init__("camera_follow_controller")
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("view_frame", "arachne_view_frame")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_yaw_topic", "/arachne/camera_yaw")
        self.declare_parameter("gazebo_offset_topic", "/arachne/gazebo_camera/follow_offset")
        self.declare_parameter("publish_rate", 60.0)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("yaw_axis", 2)
        self.declare_parameter("pitch_axis", 3)
        self.declare_parameter("yaw_rate", 2.4)
        self.declare_parameter("pitch_rate", 1.0)
        self.declare_parameter("min_pitch", 0.12)
        self.declare_parameter("max_pitch", 0.82)
        self.declare_parameter("target_height", 0.58)
        self.declare_parameter("gazebo_gui_camera", False)
        self.declare_parameter("gazebo_distance", 2.0)
        self.declare_parameter("gazebo_min_height", 0.75)
        self.declare_parameter("gazebo_max_height", 2.35)

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.view_frame = str(self.get_parameter("view_frame").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        camera_yaw_topic = str(self.get_parameter("camera_yaw_topic").value)
        gazebo_offset_topic = str(self.get_parameter("gazebo_offset_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.yaw_axis = int(self.get_parameter("yaw_axis").value)
        self.pitch_axis = int(self.get_parameter("pitch_axis").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.pitch_rate = float(self.get_parameter("pitch_rate").value)
        self.min_pitch = float(self.get_parameter("min_pitch").value)
        self.max_pitch = float(self.get_parameter("max_pitch").value)
        self.target_height = float(self.get_parameter("target_height").value)
        self.gazebo_gui_camera = bool(self.get_parameter("gazebo_gui_camera").value)
        self.gazebo_distance = float(self.get_parameter("gazebo_distance").value)
        self.gazebo_min_height = float(self.get_parameter("gazebo_min_height").value)
        self.gazebo_max_height = float(self.get_parameter("gazebo_max_height").value)

        self.orbit_yaw = 0.0
        self.pitch = 0.42
        self.base_yaw = 0.0
        self.last_joy: Joy | None = None
        self.last_joy_time = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.yaw_pub = self.create_publisher(Float64, camera_yaw_topic, 10)
        self.offset_pub = self.create_publisher(Vector3, gazebo_offset_topic, 10)
        self.create_subscription(Joy, "joy", self._joy_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)
        self.get_logger().info("Camera follow ready: right stick orbits the robot")

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.base_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now.nanoseconds - self.last_update.nanoseconds) * 1e-9, 0.0)
        self.last_update = now

        joy = self.last_joy
        joy_age = (now.nanoseconds - self.last_joy_time.nanoseconds) * 1e-9
        if joy is not None and joy_age <= self.joy_timeout and dt > 0.0:
            self.orbit_yaw = self._normalize_angle(
                self.orbit_yaw + self._axis(joy, self.yaw_axis) * self.yaw_rate * dt
            )
            self.pitch = self._clamp(
                self.pitch + self._axis(joy, self.pitch_axis) * self.pitch_rate * dt,
                self.min_pitch,
                self.max_pitch,
            )

        camera_world_yaw = self._normalize_angle(self.base_yaw + self.orbit_yaw)
        self._publish_rviz_view(now)
        self._publish_camera_yaw(camera_world_yaw)
        if self.gazebo_gui_camera:
            self._publish_gazebo_offset()

    def _publish_rviz_view(self, now) -> None:
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.view_frame
        transform.transform.translation.z = self.target_height
        qx, qy, qz, qw = self._quaternion_from_euler(0.0, self.pitch, self.orbit_yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_camera_yaw(self, camera_world_yaw: float) -> None:
        msg = Float64()
        msg.data = camera_world_yaw
        self.yaw_pub.publish(msg)

    def _publish_gazebo_offset(self) -> None:
        horizontal_distance = max(self.gazebo_distance * math.cos(self.pitch), 0.7)
        msg = Vector3()
        msg.x = -horizontal_distance * math.cos(self.orbit_yaw)
        msg.y = -horizontal_distance * math.sin(self.orbit_yaw)
        msg.z = self._clamp(
            self.target_height + self.gazebo_distance * math.sin(self.pitch),
            self.gazebo_min_height,
            self.gazebo_max_height,
        )
        self.offset_pub.publish(msg)

    def _axis(self, joy: Joy, index: int) -> float:
        if index < 0 or index >= len(joy.axes):
            return 0.0
        value = float(joy.axes[index])
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _quaternion_from_euler(self, roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraFollowController()
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
