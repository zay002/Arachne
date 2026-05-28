from __future__ import annotations

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AuboOfficialStatusProbe(Node):
    """Connectivity probe for AUBO's official ROS2 driver path.

    Arachne should drive the real Aubo i5 through `AuboRobot/aubo_ros2_driver`
    and ros2_control. This node does not command the robot; it only publishes a
    small health string so the combined bringup has one status topic per device.
    """

    def __init__(self) -> None:
        super().__init__("aubo_official_status_probe")
        self.declare_parameter("robot_ip", "192.168.127.128")
        self.declare_parameter("aubo_port", 80)
        self.declare_parameter("timeout_sec", 0.4)
        self.status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        self.create_timer(1.0, self._probe)
        self.last_status = "waiting"
        self.get_logger().info("Aubo official status probe ready")

    def _probe(self) -> None:
        robot_ip = str(self.get_parameter("robot_ip").value)
        port = int(self.get_parameter("aubo_port").value)
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        try:
            with socket.create_connection((robot_ip, port), timeout=timeout_sec):
                self.last_status = f"aubo reachable at {robot_ip}:{port}"
        except OSError as exc:
            self.last_status = f"aubo not reachable at {robot_ip}:{port}: {exc}"
        msg = String()
        msg.data = self.last_status
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = AuboOfficialStatusProbe()
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
