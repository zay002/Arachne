from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


AUBO_JOINT_ALIASES = {
    "shoulder_joint": "aubo_shoulder_joint",
    "shoulder": "aubo_shoulder_joint",
    "upperArm_joint": "aubo_upperArm_joint",
    "upper_arm_joint": "aubo_upperArm_joint",
    "upperarm_joint": "aubo_upperArm_joint",
    "upperArm": "aubo_upperArm_joint",
    "foreArm_joint": "aubo_foreArm_joint",
    "fore_arm_joint": "aubo_foreArm_joint",
    "forearm_joint": "aubo_foreArm_joint",
    "foreArm": "aubo_foreArm_joint",
    "wrist1_joint": "aubo_wrist1_joint",
    "wrist_1_joint": "aubo_wrist1_joint",
    "wrist1": "aubo_wrist1_joint",
    "wrist2_joint": "aubo_wrist2_joint",
    "wrist_2_joint": "aubo_wrist2_joint",
    "wrist2": "aubo_wrist2_joint",
    "wrist3_joint": "aubo_wrist3_joint",
    "wrist_3_joint": "aubo_wrist3_joint",
    "wrist3": "aubo_wrist3_joint",
    "aubo_i5_shoulder_joint": "aubo_shoulder_joint",
    "aubo_i5_upperArm_joint": "aubo_upperArm_joint",
    "aubo_i5_foreArm_joint": "aubo_foreArm_joint",
    "aubo_i5_wrist1_joint": "aubo_wrist1_joint",
    "aubo_i5_wrist2_joint": "aubo_wrist2_joint",
    "aubo_i5_wrist3_joint": "aubo_wrist3_joint",
}

DEFAULT_VISUALIZATION_JOINTS = {
    "front_right_wheel": 0.0,
    "front_left_wheel": 0.0,
    "rear_left_wheel": 0.0,
    "rear_right_wheel": 0.0,
    "aubo_shoulder_joint": -1.5707963267949,
    "aubo_upperArm_joint": 0.201570428261868,
    "aubo_foreArm_joint": 1.65970467002488,
    "aubo_wrist1_joint": 0.485178041391533,
    "aubo_wrist2_joint": 1.67675136677345,
    "aubo_wrist3_joint": 0.76432946885334,
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
        self.real_pose_notice_logged = False
        self.idle_notice_logged = False
        self.last_input_time = 0.0
        self.create_timer(0.2, self._publish_default_if_idle)

        self.get_logger().info(
            f"Teach visualization joint state adapter ready: {input_topic} -> {output_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self.last_input_time = self.get_clock().now().nanoseconds * 1e-9
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
            name = self._normalize_joint_name(raw_name)
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
        if not self.real_pose_notice_logged:
            aubo_seen = sum(
                1 for name in set(AUBO_JOINT_ALIASES.values()) if name in index_by_name
            )
            if aubo_seen >= 6:
                self.get_logger().info("RViz visualization is following real Aubo /joint_states.")
                self.real_pose_notice_logged = True
                self.idle_notice_logged = False

    def _publish_default_if_idle(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_input_time and now - self.last_input_time < 0.5:
            return
        if not self.idle_notice_logged:
            self.get_logger().warn(
                "No fresh /joint_states for RViz visualization; publishing default model pose."
            )
            self.idle_notice_logged = True
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DEFAULT_VISUALIZATION_JOINTS)
        msg.position = [DEFAULT_VISUALIZATION_JOINTS[name] for name in msg.name]
        self.publisher.publish(msg)

    def _normalize_joint_name(self, raw_name: str) -> str:
        if raw_name in AUBO_JOINT_ALIASES:
            return AUBO_JOINT_ALIASES[raw_name]
        if raw_name.startswith("aubo_") and raw_name in DEFAULT_VISUALIZATION_JOINTS:
            return raw_name
        lowered = raw_name.lower()
        for prefix in ("aubo_i5_", "aubo_", "i5_", "rob1_", "robot_"):
            if lowered.startswith(prefix):
                stripped = raw_name[len(prefix) :]
                if stripped in AUBO_JOINT_ALIASES:
                    return AUBO_JOINT_ALIASES[stripped]
        return raw_name


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
