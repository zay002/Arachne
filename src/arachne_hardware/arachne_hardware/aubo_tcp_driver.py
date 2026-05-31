from __future__ import annotations

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from aubo_msgs.srv import JsonRpc
except ImportError:  # pragma: no cover - optional until the official Aubo driver is installed.
    JsonRpc = None


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


class AuboTeachCommandBridge(Node):
    """Bridge Arachne teach-mode commands to the official AUBO JsonRpc service."""

    def __init__(self) -> None:
        super().__init__("aubo_teach_command_bridge")
        self.declare_parameter("command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("jsonrpc_service", "jsonrpc_service")
        self.declare_parameter("teach_method", "freedrive")
        self.status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
        )
        self.client = None
        if JsonRpc is None:
            self._publish_status("aubo teach bridge unavailable: missing aubo_msgs", warn=True)
        else:
            self.client = self.create_client(
                JsonRpc,
                str(self.get_parameter("jsonrpc_service").value),
            )
            self._publish_status("aubo teach bridge ready")

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ("teach_on", "on", "enter", "enable"):
            self._call_teach(True)
        elif command in ("teach_off", "off", "exit", "disable"):
            self._call_teach(False)
        else:
            self._publish_status(f"aubo teach ignored unknown command: {command}", warn=True)

    def _call_teach(self, enabled: bool) -> None:
        if self.client is None:
            self._publish_status("aubo teach skipped: JsonRpc client unavailable", warn=True)
            return
        if not self.client.service_is_ready() and not self.client.wait_for_service(timeout_sec=0.2):
            self._publish_status("aubo teach skipped: jsonrpc_service unavailable", warn=True)
            return

        method = str(self.get_parameter("teach_method").value).strip() or "freedrive"
        request = JsonRpc.Request()
        request.cls = "RobotManage"
        request.func = method
        request.params = "[true]" if enabled else "[false]"
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda result_future, mode=enabled, func=method: self._on_teach_response(
                result_future, mode, func
            )
        )
        self._publish_status(f"aubo teach {'on' if enabled else 'off'} requested via {method}")

    def _on_teach_response(self, future, enabled: bool, method: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depends on live driver.
            self._publish_status(f"aubo teach {method} call failed: {exc}", warn=True)
            return
        error = getattr(response, "error", "")
        if error:
            self._publish_status(f"aubo teach {method} error: {error}", warn=True)
            return
        self._publish_status(f"aubo teach {'on' if enabled else 'off'} active via {method}")

    def _publish_status(self, text: str, *, warn: bool = False) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def status_probe_main() -> None:
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


def teach_command_bridge_main() -> None:
    rclpy.init()
    node = AuboTeachCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    status_probe_main()


if __name__ == "__main__":
    main()
