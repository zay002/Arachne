from __future__ import annotations

import math
import shutil
import subprocess

import rclpy
from geometry_msgs.msg import TransformStamped
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
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("yaw_axis", 2)
        self.declare_parameter("pitch_axis", 3)
        self.declare_parameter("yaw_rate", 1.8)
        self.declare_parameter("pitch_rate", 0.8)
        self.declare_parameter("min_pitch", -0.35)
        self.declare_parameter("max_pitch", 0.75)
        self.declare_parameter("target_height", 0.48)
        self.declare_parameter("gazebo_gui_camera", False)
        self.declare_parameter("gazebo_follow_target", "arachne")
        self.declare_parameter("gazebo_update_period", 0.2)
        self.declare_parameter("gazebo_distance", 2.0)
        self.declare_parameter("gazebo_min_height", 0.65)
        self.declare_parameter("gazebo_max_height", 2.5)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.view_frame = self.get_parameter("view_frame").value
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
        self.gazebo_follow_target = str(self.get_parameter("gazebo_follow_target").value)
        self.gazebo_update_period = float(self.get_parameter("gazebo_update_period").value)
        self.gazebo_distance = float(self.get_parameter("gazebo_distance").value)
        self.gazebo_min_height = float(self.get_parameter("gazebo_min_height").value)
        self.gazebo_max_height = float(self.get_parameter("gazebo_max_height").value)

        self.yaw = 0.0
        self.pitch = 0.35
        self.last_joy: Joy | None = None
        self.last_joy_time = self.get_clock().now()
        self.last_update = self.get_clock().now()
        self.last_gazebo_update = self.get_clock().now()
        self.gazebo_follow_ready = False
        self.gz_cmd = shutil.which("gz")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.yaw_pub = self.create_publisher(Float64, "/arachne/camera_yaw", 10)
        self.create_subscription(Joy, "joy", self._joy_callback, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)
        self.get_logger().info("Camera follow ready: right stick orbits the demo camera")

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now.nanoseconds - self.last_update.nanoseconds) * 1e-9, 0.0)
        self.last_update = now

        joy = self.last_joy
        joy_age = (now.nanoseconds - self.last_joy_time.nanoseconds) * 1e-9
        if joy is not None and joy_age <= self.joy_timeout and dt > 0.0:
            self.yaw = self._normalize_angle(self.yaw - self._axis(joy, self.yaw_axis) * self.yaw_rate * dt)
            self.pitch = min(
                max(self.pitch + self._axis(joy, self.pitch_axis) * self.pitch_rate * dt, self.min_pitch),
                self.max_pitch,
            )

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.view_frame
        transform.transform.translation.z = self.target_height
        qx, qy, qz, qw = self._quaternion_from_euler(0.0, self.pitch, self.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        yaw_msg = Float64()
        yaw_msg.data = self.yaw
        self.yaw_pub.publish(yaw_msg)

        if self.gazebo_gui_camera:
            self._update_gazebo_camera(now)

    def _update_gazebo_camera(self, now) -> None:
        if self.gz_cmd is None:
            return
        elapsed = (now.nanoseconds - self.last_gazebo_update.nanoseconds) * 1e-9
        if elapsed < self.gazebo_update_period:
            return
        self.last_gazebo_update = now

        if not self.gazebo_follow_ready:
            self.gazebo_follow_ready = self._gz_service(
                "/gui/follow",
                "gz.msgs.StringMsg",
                'data: "%s"' % self.gazebo_follow_target,
            )

        horizontal_distance = max(self.gazebo_distance * math.cos(self.pitch), 0.8)
        offset_x = -horizontal_distance * math.cos(self.yaw)
        offset_y = -horizontal_distance * math.sin(self.yaw)
        offset_z = self._clamp(
            self.target_height + self.gazebo_distance * math.sin(self.pitch),
            self.gazebo_min_height,
            self.gazebo_max_height,
        )
        self._gz_service(
            "/gui/follow/offset",
            "gz.msgs.Vector3d",
            f"x: {offset_x:.4f} y: {offset_y:.4f} z: {offset_z:.4f}",
        )

    def _gz_service(self, service: str, reqtype: str, request: str) -> bool:
        try:
            result = subprocess.run(
                [
                    self.gz_cmd,
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    reqtype,
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "250",
                    "--req",
                    request,
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.35,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

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
