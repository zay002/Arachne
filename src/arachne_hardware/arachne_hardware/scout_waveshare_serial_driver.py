from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


CAN_BITRATE_CODES = {
    1000000: 0x01,
    800000: 0x02,
    500000: 0x03,
    400000: 0x04,
    250000: 0x05,
    200000: 0x06,
    125000: 0x07,
    100000: 0x08,
    50000: 0x09,
    20000: 0x0A,
    10000: 0x0B,
    5000: 0x0C,
}

FRAME_TYPE_CODES = {"standard": 0x01, "extended": 0x02}
MODE_CODES = {"normal": 0x00, "silent": 0x01, "loopback": 0x02, "silent_loopback": 0x03}
PROTOCOL_CONFIG_CODES = {"fixed": 0x02, "variable": 0x12}

SCOUT_CONTROL_MODE_ID = 0x421
SCOUT_MOTION_COMMAND_ID = 0x111
SCOUT_SYSTEM_STATE_ID = 0x211
SCOUT_MOTION_STATE_ID = 0x221


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    frame_type: str


def _checksum(data: bytes | bytearray) -> int:
    return sum(data) & 0xFF


def _lookup(mapping: dict, key, label: str) -> int:
    if key not in mapping:
        valid = ", ".join(str(item) for item in mapping)
        raise ValueError(f"unsupported {label}: {key}; supported: {valid}")
    return mapping[key]


def encode_config(
    can_bitrate: int,
    frame_type: str,
    protocol: str = "variable",
    mode: str = "normal",
) -> bytes:
    payload = bytearray(
        [
            0xAA,
            0x55,
            _lookup(PROTOCOL_CONFIG_CODES, protocol, "protocol"),
            _lookup(CAN_BITRATE_CODES, can_bitrate, "CAN bitrate"),
            _lookup(FRAME_TYPE_CODES, frame_type, "frame type"),
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            _lookup(MODE_CODES, mode, "mode"),
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )
    payload.append(_checksum(payload[2:19]))
    return bytes(payload)


def encode_variable_frame(can_id: int, data: bytes, frame_type: str) -> bytes:
    if len(data) > 8:
        raise ValueError("CAN2.0 data length must be <= 8 bytes")
    frame_type_bit = 0x20 if frame_type == "extended" else 0x00
    id_length = 4 if frame_type == "extended" else 2
    type_byte = 0xC0 | frame_type_bit | len(data)
    return bytes([0xAA, type_byte]) + can_id.to_bytes(id_length, "little") + data + bytes([0x55])


def decode_variable_frames(stream: bytes) -> tuple[list[CanFrame], bytes]:
    frames: list[CanFrame] = []
    index = 0
    while index < len(stream):
        try:
            start = stream.index(0xAA, index)
        except ValueError:
            return frames, b""
        if start + 3 >= len(stream):
            return frames, stream[start:]

        type_byte = stream[start + 1]
        if type_byte & 0xC0 != 0xC0:
            index = start + 1
            continue

        frame_type = "extended" if type_byte & 0x20 else "standard"
        remote = bool(type_byte & 0x10)
        dlc = type_byte & 0x0F
        id_length = 4 if frame_type == "extended" else 2
        data_length = 0 if remote else dlc
        end = start + 2 + id_length + data_length
        if end >= len(stream):
            return frames, stream[start:]
        if stream[end] != 0x55:
            index = start + 1
            continue

        can_id = int.from_bytes(stream[start + 2 : start + 2 + id_length], "little")
        data_start = start + 2 + id_length
        frames.append(CanFrame(can_id, stream[data_start : data_start + data_length], frame_type))
        index = end + 1
    return frames, b""


def int16_to_bytes(value: int) -> bytes:
    value = max(-32768, min(32767, int(value)))
    return value.to_bytes(2, "big", signed=True)


def int16_from_bytes(data: bytes) -> int:
    return int.from_bytes(data, "big", signed=True)


class ScoutWaveshareSerialDriver(Node):
    """Scout 2.0 driver for Waveshare USB-CAN-A serial adapters.

    The AgileX Scout v2 protocol uses standard 11-bit CAN IDs over a 500 kbit/s
    bus. Waveshare USB-CAN-A itself is a USB serial device, so this node writes
    Waveshare's variable-length serial frame format directly and works in WSL2
    without SocketCAN support.
    """

    def __init__(self) -> None:
        super().__init__("scout_waveshare_serial_driver")
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("usb_baudrate", 2_000_000)
        self.declare_parameter("can_bitrate", 500_000)
        self.declare_parameter("frame_type", "standard")
        self.declare_parameter("mode", "normal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 1.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("auto_enable_command_mode", True)

        self.port = str(self.get_parameter("port").value)
        self.frame_type = str(self.get_parameter("frame_type").value).strip().lower()
        self.can_bitrate = int(self.get_parameter("can_bitrate").value)
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.auto_enable_command_mode = bool(
            self.get_parameter("auto_enable_command_mode").value
        )

        self.serial = serial.Serial(
            self.port,
            baudrate=int(self.get_parameter("usb_baudrate").value),
            timeout=0,
            write_timeout=0.2,
        )
        self.serial.write(encode_config(self.can_bitrate, self.frame_type, mode=self.mode))
        time.sleep(0.05)

        self.latest_twist = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.rx_buffer = b""
        self.last_motion_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.latest_linear_velocity = 0.0
        self.latest_angular_velocity = 0.0
        self.latest_status = "waiting for Scout feedback"

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/arachne/hardware/base_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            10,
        )

        send_period = 1.0 / max(float(self.get_parameter("send_rate_hz").value), 1.0)
        self.create_timer(send_period, self._tick)
        self.create_timer(1.0, self._publish_status)
        if self.auto_enable_command_mode:
            self._send_frame(SCOUT_CONTROL_MODE_ID, bytes([0x01]))
        self.get_logger().info(
            "Scout Waveshare driver ready: "
            f"port={self.port}, can_bitrate={self.can_bitrate}, frame_type={self.frame_type}"
        )

    def destroy_node(self) -> bool:
        try:
            self._send_motion(0.0, 0.0)
            self.serial.close()
        except Exception:
            pass
        return super().destroy_node()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.latest_twist = msg
        self.latest_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        self._read_available()
        now = self.get_clock().now()
        age = (now - self.latest_cmd_time).nanoseconds / 1e9
        twist = self.latest_twist if age <= self.command_timeout_sec else Twist()
        linear = max(-self.max_linear_speed, min(self.max_linear_speed, twist.linear.x))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, twist.angular.z))
        self._send_motion(linear, angular)

    def _send_motion(self, linear_mps: float, angular_rps: float) -> None:
        linear = int(round(linear_mps * 1000.0))
        angular = int(round(angular_rps * 1000.0))
        payload = (
            int16_to_bytes(linear)
            + int16_to_bytes(angular)
            + int16_to_bytes(0)
            + int16_to_bytes(0)
        )
        self._send_frame(SCOUT_MOTION_COMMAND_ID, payload)

    def _send_frame(self, can_id: int, payload: bytes) -> None:
        self.serial.write(encode_variable_frame(can_id, payload, self.frame_type))

    def _read_available(self) -> None:
        while True:
            chunk = self.serial.read(4096)
            if not chunk:
                break
            self.rx_buffer += chunk
        frames, remainder = decode_variable_frames(self.rx_buffer)
        self.rx_buffer = remainder[-128:]
        for frame in frames:
            self._handle_frame(frame)

    def _handle_frame(self, frame: CanFrame) -> None:
        if frame.can_id == SCOUT_SYSTEM_STATE_ID and len(frame.data) >= 8:
            battery = int16_from_bytes(frame.data[2:4]) / 10.0
            error = int16_from_bytes(frame.data[4:6])
            self.latest_status = (
                "scout system "
                f"vehicle_state={frame.data[0]} control_mode={frame.data[1]} "
                f"battery={battery:.1f}V error=0x{error:04x}"
            )
        elif frame.can_id == SCOUT_MOTION_STATE_ID and len(frame.data) >= 8:
            linear = int16_from_bytes(frame.data[0:2]) / 1000.0
            angular = int16_from_bytes(frame.data[2:4]) / 1000.0
            self._integrate_and_publish_odom(linear, angular)

    def _integrate_and_publish_odom(self, linear: float, angular: float) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_motion_time).nanoseconds / 1e9
        self.last_motion_time = now
        if dt < 0.0 or dt > 1.0:
            dt = 0.0

        self.latest_linear_velocity = linear
        self.latest_angular_velocity = angular
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + angular * dt), math.cos(self.yaw + angular * dt))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = str(self.get_parameter("odom_frame").value)
        odom.child_frame_id = str(self.get_parameter("base_frame").value)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"{self.latest_status}; "
            f"odom_estimate x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}deg "
            f"vx={self.latest_linear_velocity:.3f} wz={self.latest_angular_velocity:.3f}"
        )
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScoutWaveshareSerialDriver()
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
