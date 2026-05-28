from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class SafetyCmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("arachne_safety_cmd_vel_gate")
        self.declare_parameter("input_topic", "/arachne/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.enabled = False
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_cmd, 10
        )
        self.create_subscription(Bool, "/arachne/safety/enabled", self._on_enabled, 10)
        self.create_timer(0.5, self._publish_stop_if_disabled)
        self.get_logger().info("Safety cmd_vel gate ready")

    def _on_enabled(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _on_cmd(self, msg: Twist) -> None:
        if self.enabled:
            self.pub.publish(msg)

    def _publish_stop_if_disabled(self) -> None:
        if not self.enabled:
            self.pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = SafetyCmdVelGate()
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
