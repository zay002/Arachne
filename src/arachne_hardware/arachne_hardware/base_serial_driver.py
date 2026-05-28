from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class ScoutOfficialStatusBridge(Node):
    """Status helper for the official AgileX Scout ROS2 base driver.

    The real base control node should be `scout_base/scout_base_node` from
    `agilexrobotics/scout_ros2`. That node subscribes to `/cmd_vel`, publishes
    `/odom`, `/scout_status`, and owns the CAN/ugv_sdk connection. This helper
    only republishes a lightweight Arachne status string so higher-level launch
    files have a consistent health topic across devices.
    """

    def __init__(self) -> None:
        super().__init__("scout_official_status_bridge")
        self.declare_parameter("odom_topic", "/odom")
        self.status_pub = self.create_publisher(String, "/arachne/hardware/base_status", 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self.create_timer(1.0, self._publish_status)
        self.last_status = "waiting for official scout_base /odom"
        self.get_logger().info("Scout official status bridge ready")

    def _on_odom(self, msg: Odometry) -> None:
        self.last_status = (
            "scout_base odom "
            f"x={msg.pose.pose.position.x:.3f} y={msg.pose.pose.position.y:.3f} "
            f"vx={msg.twist.twist.linear.x:.3f} wz={msg.twist.twist.angular.z:.3f}"
        )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self.last_status
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScoutOfficialStatusBridge()
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
