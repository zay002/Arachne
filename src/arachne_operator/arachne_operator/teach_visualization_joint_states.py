from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


AUBO_JOINT_ALIASES = {
    "shoulder_joint": "aubo_shoulder_joint",
    "upperArm_joint": "aubo_upperArm_joint",
    "foreArm_joint": "aubo_foreArm_joint",
    "wrist1_joint": "aubo_wrist1_joint",
    "wrist2_joint": "aubo_wrist2_joint",
    "wrist3_joint": "aubo_wrist3_joint",
}

DEFAULT_VISUALIZATION_JOINTS = {
    "front_right_wheel": 0.0,
    "front_left_wheel": 0.0,
    "rear_left_wheel": 0.0,
    "rear_right_wheel": 0.0,
    "ms42dc_left_finger_joint": 0.0,
}


class TeachVisualizationJointStates(Node):
    def __init__(self) -> None:
        super().__init__("teach_visualization_joint_states")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/arachne/teach_visualization/joint_states")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 10)
        self.alias_notice_logged = False

        self.get_logger().info(
            f"Teach visualization joint state adapter ready: {input_topic} -> {output_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header

        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        index_by_name: dict[str, int] = {}
        used_alias = False

        has_velocities = len(msg.velocity) >= len(msg.name)
        has_efforts = len(msg.effort) >= len(msg.name)

        for index, raw_name in enumerate(msg.name):
            name = AUBO_JOINT_ALIASES.get(raw_name, raw_name)
            used_alias = used_alias or name != raw_name

            if name in index_by_name:
                out_index = index_by_name[name]
            else:
                index_by_name[name] = len(names)
                names.append(name)
                positions.append(0.0)
                if has_velocities:
                    velocities.append(0.0)
                if has_efforts:
                    efforts.append(0.0)
                out_index = len(names) - 1

            if index < len(msg.position):
                positions[out_index] = float(msg.position[index])
            if has_velocities:
                velocities[out_index] = float(msg.velocity[index])
            if has_efforts:
                efforts[out_index] = float(msg.effort[index])

        for name, default_position in DEFAULT_VISUALIZATION_JOINTS.items():
            if name in index_by_name:
                continue
            index_by_name[name] = len(names)
            names.append(name)
            positions.append(default_position)
            if has_velocities:
                velocities.append(0.0)
            if has_efforts:
                efforts.append(0.0)

        out.name = names
        out.position = positions
        out.velocity = velocities
        out.effort = efforts
        self.publisher.publish(out)

        if used_alias and not self.alias_notice_logged:
            self.get_logger().info(
                "Aubo joint names were adapted for the Arachne visualization URDF."
            )
            self.alias_notice_logged = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeachVisualizationJointStates()
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
