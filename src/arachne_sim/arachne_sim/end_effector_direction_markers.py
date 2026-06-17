from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _point(x: float, y: float, z: float) -> Point:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def _axis_vector(axis: str) -> tuple[float, float, float]:
    axis = axis.strip().lower()
    if axis.startswith("plus_"):
        axis = "+" + axis.removeprefix("plus_")
    elif axis.startswith("minus_"):
        axis = "-" + axis.removeprefix("minus_")
    sign = -1.0 if axis.startswith("-") else 1.0
    key = axis[1:] if axis.startswith(("-", "+")) else axis
    if key == "x":
        return (sign, 0.0, 0.0)
    if key == "y":
        return (0.0, sign, 0.0)
    if key == "z":
        return (0.0, 0.0, sign)
    raise ValueError(f"unsupported axis {axis!r}; use x/y/z with optional sign")


class EndEffectorDirectionMarkers(Node):
    def __init__(self) -> None:
        super().__init__("end_effector_direction_markers")
        self.declare_parameter("marker_topic", "/arachne/model/direction_markers")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("tool_axis", "plus_z")
        self.declare_parameter("camera_axis", "plus_z")
        self.declare_parameter("tool_length_m", 0.22)
        self.declare_parameter("camera_length_m", 0.38)
        self.declare_parameter("publish_rate", 20.0)

        marker_topic = str(self.get_parameter("marker_topic").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.tool_axis = _axis_vector(str(self.get_parameter("tool_axis").value))
        self.camera_axis = _axis_vector(str(self.get_parameter("camera_axis").value))
        self.tool_length = float(self.get_parameter("tool_length_m").value)
        self.camera_length = float(self.get_parameter("camera_length_m").value)

        self.publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            "Direction markers ready: "
            f"{self.tool_frame} tool_axis={self.get_parameter('tool_axis').value}, "
            f"{self.camera_frame} camera_axis={self.get_parameter('camera_axis').value}"
        )

    def _publish(self) -> None:
        markers = MarkerArray()
        markers.markers.append(
            self._direction_marker(
                marker_id=1,
                frame_id=self.tool_frame,
                namespace="tool0_grasp_direction",
                axis=self.tool_axis,
                length=self.tool_length,
                color=_color(0.0, 0.7, 1.0, 0.95),
                shaft_radius=0.012,
                head_radius=0.035,
            )
        )
        markers.markers.append(
            self._direction_marker(
                marker_id=2,
                frame_id=self.camera_frame,
                namespace="camera_center_ray",
                axis=self.camera_axis,
                length=self.camera_length,
                color=_color(1.0, 0.82, 0.0, 0.95),
                shaft_radius=0.008,
                head_radius=0.026,
            )
        )
        self.publisher.publish(markers)

    def _direction_marker(
        self,
        *,
        marker_id: int,
        frame_id: str,
        namespace: str,
        axis: tuple[float, float, float],
        length: float,
        color: ColorRGBA,
        shaft_radius: float,
        head_radius: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [
            _point(0.0, 0.0, 0.0),
            _point(axis[0] * length, axis[1] * length, axis[2] * length),
        ]
        marker.scale.x = shaft_radius
        marker.scale.y = head_radius
        marker.scale.z = head_radius * 0.75
        marker.color = color
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.frame_locked = True
        return marker


def main() -> None:
    rclpy.init()
    node = EndEffectorDirectionMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
