from __future__ import annotations

import importlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class MS42DCOfficialBridge(Node):
    """Bridge Arachne gripper commands to the vendor MS42DC ROS2 package.

    The MS42DC documentation ships a ROS2 package named `step_motor`.
    Its `motor_node` owns the serial port and subscribes to `motor_control`
    with `step_motor/msg/Motor`. Arachne should not duplicate that serial
    driver when the official package is available; this node only translates
    the shared Arachne Open/Close command surface into the vendor message.
    """

    def __init__(self) -> None:
        super().__init__("ms42dc_official_bridge")
        self.declare_parameter("motor_control_topic", "motor_control")
        self.declare_parameter("device_id", 1)
        self.declare_parameter("sub_divide", 32)
        self.declare_parameter("mode", 2)
        self.declare_parameter("open_angle_tenths", 19656)
        self.declare_parameter("close_angle_tenths", 19656)
        self.declare_parameter("speed_tenths", 150)

        motor_module = importlib.import_module("step_motor.msg")
        self.motor_msg_type = getattr(motor_module, "Motor")
        self.motor_pub = self.create_publisher(
            self.motor_msg_type,
            str(self.get_parameter("motor_control_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(String, "/arachne/hardware/gripper_status", 10)
        self.create_subscription(String, "/arachne/gripper/command", self._on_command, 10)
        self.create_subscription(Float64, "/arachne/gripper/angle_degrees", self._on_angle, 10)
        self.create_timer(1.0, self._publish_status)
        self.last_status = "ready"
        self.target_opening_tenths = int(self.get_parameter("open_angle_tenths").value)

        self.get_logger().info(
            "MS42DC bridge ready: /arachne/gripper/command -> step_motor/motor_control"
        )

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "close":
            angle = int(self.get_parameter("close_angle_tenths").value)
            self._publish_motor(direction=1, angle_tenths=angle, label="close")
            self.target_opening_tenths = 0
        elif command in ("open", "home"):
            angle = int(self.get_parameter("open_angle_tenths").value)
            self._publish_motor(direction=0, angle_tenths=angle, label="open")
            self.target_opening_tenths = angle
        elif command == "stop":
            self._publish_motor(direction=0, angle_tenths=0, speed_tenths=0, label="stop")
        elif command.isdigit():
            self._publish_opening_target(int(command))
        else:
            self.get_logger().warning(f"Ignoring unknown gripper command: {msg.data!r}")

    def _on_angle(self, msg: Float64) -> None:
        direction = 1 if msg.data >= 0.0 else 0
        angle = int(round(abs(float(msg.data)) * 10.0))
        self._publish_motor(direction=direction, angle_tenths=angle, label="angle")
        self._update_target_opening(direction=direction, angle_tenths=angle)

    def _publish_opening_target(self, target_tenths: int) -> None:
        max_open = max(int(self.get_parameter("open_angle_tenths").value), 1)
        target = max(0, min(int(target_tenths), max_open))
        current = max(0, min(int(self.target_opening_tenths), max_open))
        delta = target - current
        if delta == 0:
            self.last_status = f"opening target unchanged: {target}/{max_open}"
            self.get_logger().info(self.last_status)
            return
        self._publish_motor(
            direction=0 if delta > 0 else 1,
            angle_tenths=abs(delta),
            label=f"opening {target}/{max_open}",
        )
        self.target_opening_tenths = target

    def _update_target_opening(self, *, direction: int, angle_tenths: int) -> None:
        max_open = max(int(self.get_parameter("open_angle_tenths").value), 1)
        delta = -int(angle_tenths) if int(direction) == 1 else int(angle_tenths)
        self.target_opening_tenths = max(
            0, min(int(self.target_opening_tenths) + delta, max_open)
        )

    def _publish_motor(
        self,
        *,
        direction: int,
        angle_tenths: int,
        label: str,
        speed_tenths: int | None = None,
    ) -> None:
        motor = self.motor_msg_type()
        motor.id = int(self.get_parameter("device_id").value)
        motor.speed = int(self.get_parameter("speed_tenths").value if speed_tenths is None else speed_tenths)
        motor.dir = int(direction)
        motor.mode = int(self.get_parameter("mode").value)
        motor.angle = max(0, min(int(angle_tenths), 65535))
        motor.state = 0
        motor.sub_divide = int(self.get_parameter("sub_divide").value)
        self.motor_pub.publish(motor)
        self.last_status = (
            f"{label}: id={motor.id} mode={motor.mode} dir={motor.dir} "
            f"angle={motor.angle} speed={motor.speed} sub_divide={motor.sub_divide}"
        )
        self.get_logger().info(self.last_status)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self.last_status
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    try:
        node = MS42DCOfficialBridge()
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing official MS42DC ROS2 package `step_motor`. "
            "Extract the vendor ROS2.zip into the workspace and rebuild."
        ) from exc
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
