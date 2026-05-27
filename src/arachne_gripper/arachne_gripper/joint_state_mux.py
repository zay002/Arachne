from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_mux")
        self.declare_parameter("default_topic", "/arachne/default_joint_states")
        self.declare_parameter("gui_topic", "/arachne/gui_joint_states")
        self.declare_parameter("base_topic", "/arachne/base/joint_states")
        self.declare_parameter("gripper_topic", "/arachne/gripper/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("publish_rate", 30.0)

        default_topic = self.get_parameter("default_topic").get_parameter_value().string_value
        gui_topic = self.get_parameter("gui_topic").get_parameter_value().string_value
        base_topic = self.get_parameter("base_topic").get_parameter_value().string_value
        gripper_topic = self.get_parameter("gripper_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.default_positions: dict[str, float] = {}
        self.default_order: list[str] = []
        self.gui_positions: dict[str, float] = {}
        self.gui_order: list[str] = []
        self.base_positions: dict[str, float] = {}
        self.base_order: list[str] = []
        self.gripper_positions: dict[str, float] = {}
        self.gripper_order: list[str] = []

        self.pub = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, default_topic, self._default_callback, 10)
        self.create_subscription(JointState, gui_topic, self._gui_callback, 10)
        self.create_subscription(JointState, base_topic, self._base_callback, 10)
        self.create_subscription(JointState, gripper_topic, self._gripper_callback, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._tick)

        self.get_logger().info(
            "Joint state mux ready: "
            f"{default_topic} + {gui_topic} + {base_topic} + {gripper_topic} -> {output_topic}"
        )

    def _update_positions(
        self, msg: JointState, positions: dict[str, float], order: list[str]
    ) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                if name not in positions:
                    order.append(name)
                positions[name] = msg.position[index]

    def _default_callback(self, msg: JointState) -> None:
        self._update_positions(msg, self.default_positions, self.default_order)

    def _gui_callback(self, msg: JointState) -> None:
        self._update_positions(msg, self.gui_positions, self.gui_order)

    def _base_callback(self, msg: JointState) -> None:
        self._update_positions(msg, self.base_positions, self.base_order)

    def _gripper_callback(self, msg: JointState) -> None:
        self._update_positions(msg, self.gripper_positions, self.gripper_order)

    def _tick(self) -> None:
        if (
            not self.default_positions
            and not self.gui_positions
            and not self.base_positions
            and not self.gripper_positions
        ):
            return

        names = list(self.default_order)
        for order in (self.gui_order, self.base_order, self.gripper_order):
            for name in order:
                if name not in names:
                    names.append(name)

        positions_by_name = dict(self.default_positions)
        positions_by_name.update(self.gui_positions)
        positions_by_name.update(self.base_positions)
        positions_by_name.update(self.gripper_positions)

        positions = []
        for name in names:
            positions.append(positions_by_name[name])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JointStateMux()
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
