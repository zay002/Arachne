from __future__ import annotations

from dataclasses import dataclass, replace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class GripperProfile:
    joint_names: tuple[str, ...]
    open_position: float
    closed_position: float
    max_velocity: float
    multipliers: tuple[float, ...] | None = None


PROFILES = {
    "ms42dc": GripperProfile(
        joint_names=("ms42dc_left_finger_joint", "ms42dc_right_finger_joint"),
        open_position=0.0,
        closed_position=0.6,
        max_velocity=1.0,
        multipliers=(1.0, -1.0),
    ),
    "ag95": GripperProfile(
        joint_names=(
            "left_outer_knuckle_joint",
            "right_outer_knuckle_joint",
            "left_finger_joint",
            "right_finger_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
        ),
        open_position=0.0,
        closed_position=0.93,
        max_velocity=1.2,
        multipliers=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
    ),
}


class GripperSimController(Node):
    def __init__(self) -> None:
        super().__init__("gripper_sim_controller")
        self.declare_parameter("profile", "")
        self.declare_parameter("gripper_type", "ms42dc")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("open_position", -1.0)
        self.declare_parameter("closed_position", -1.0)
        self.declare_parameter("max_velocity", -1.0)
        self.declare_parameter("joint_state_topic", "/arachne/gripper/joint_states")
        self.declare_parameter("command_topic", "/arachne/gripper/command")
        self.declare_parameter("position_topic", "/arachne/gripper/position")
        self.declare_parameter("open_service", "/arachne/gripper/open")
        self.declare_parameter("close_service", "/arachne/gripper/close")
        self.declare_parameter("stop_service", "/arachne/gripper/stop")

        profile_name = self.get_parameter("profile").get_parameter_value().string_value
        if not profile_name:
            profile_name = self.get_parameter("gripper_type").get_parameter_value().string_value

        if profile_name not in PROFILES:
            known = ", ".join(sorted(PROFILES))
            raise ValueError(f"Unsupported gripper profile '{profile_name}'. Expected one of: {known}")

        self.profile = self._profile_with_overrides(PROFILES[profile_name])
        self.position = self.profile.open_position
        self.target = self.position
        self.stopped = False

        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.period = 1.0 / max(publish_rate, 1.0)
        joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        position_topic = self.get_parameter("position_topic").get_parameter_value().string_value
        open_service = self.get_parameter("open_service").get_parameter_value().string_value
        close_service = self.get_parameter("close_service").get_parameter_value().string_value
        stop_service = self.get_parameter("stop_service").get_parameter_value().string_value

        self.joint_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_subscription(String, command_topic, self._command_callback, 10)
        self.create_subscription(Float64, position_topic, self._position_callback, 10)

        self.create_service(Trigger, open_service, self._open_service)
        self.create_service(Trigger, close_service, self._close_service)
        self.create_service(Trigger, stop_service, self._stop_service)

        self.timer = self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"Gripper sim controller ready for {profile_name}. "
            f"open={self.profile.open_position}, closed={self.profile.closed_position}. "
            f"Use {command_topic} with 'open', 'close', or 'stop'."
        )

    def _profile_with_overrides(self, profile: GripperProfile) -> GripperProfile:
        open_position = self.get_parameter("open_position").get_parameter_value().double_value
        closed_position = self.get_parameter("closed_position").get_parameter_value().double_value
        max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value

        return replace(
            profile,
            open_position=profile.open_position if open_position < 0.0 else open_position,
            closed_position=profile.closed_position if closed_position < 0.0 else closed_position,
            max_velocity=profile.max_velocity if max_velocity <= 0.0 else max_velocity,
        )

    def _open_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_target(self.profile.open_position)
        response.success = True
        response.message = "Opening gripper"
        return response

    def _close_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_target(self.profile.closed_position)
        response.success = True
        response.message = "Closing gripper"
        return response

    def _stop_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.stopped = True
        self.target = self.position
        response.success = True
        response.message = "Stopped gripper"
        return response

    def _command_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "open":
            self._set_target(self.profile.open_position)
        elif command == "close":
            self._set_target(self.profile.closed_position)
        elif command == "stop":
            self.stopped = True
            self.target = self.position
        else:
            self.get_logger().warn(f"Unknown gripper command '{msg.data}'")

    def _position_callback(self, msg: Float64) -> None:
        ratio = min(max(msg.data, 0.0), 1.0)
        span = self.profile.closed_position - self.profile.open_position
        self._set_target(self.profile.open_position + ratio * span)

    def _set_target(self, target: float) -> None:
        lower = min(self.profile.open_position, self.profile.closed_position)
        upper = max(self.profile.open_position, self.profile.closed_position)
        self.target = min(max(target, lower), upper)
        self.stopped = False

    def _tick(self) -> None:
        if not self.stopped:
            delta = self.target - self.position
            max_step = self.profile.max_velocity * self.period
            if abs(delta) <= max_step:
                self.position = self.target
            else:
                self.position += max_step if delta > 0.0 else -max_step

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.profile.joint_names)
        multipliers = self.profile.multipliers or tuple(1.0 for _ in self.profile.joint_names)
        msg.position = [self.position * multiplier for multiplier in multipliers]
        self.joint_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GripperSimController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
