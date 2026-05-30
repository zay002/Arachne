from __future__ import annotations

import threading
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

try:
    import serial
except ImportError as exc:  # pragma: no cover - reported clearly at runtime.
    serial = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


class MS42DCDirectSerialDriver(Node):
    """Direct ROS2-to-serial driver for the MS42DC gripper.

    The vendor ROS2 package is useful on native Linux, but its serial backend
    can abort on WSL2 cdc_acm devices. This node keeps Arachne's normal ROS
    command surface and writes the documented MS42DC USB serial frames directly.
    """

    def __init__(self) -> None:
        super().__init__("ms42dc_direct_serial_driver")
        if serial is None:
            raise RuntimeError("python3-serial/pyserial is required") from _SERIAL_IMPORT_ERROR

        self.declare_parameter("port", "/dev/motor_serial")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("command_topic", "/arachne/gripper/command")
        self.declare_parameter("angle_topic", "/arachne/gripper/angle_degrees")
        self.declare_parameter("device_id", 1)
        self.declare_parameter("sub_divide", 32)
        self.declare_parameter("mode", 2)
        self.declare_parameter("open_angle_tenths", 18720)
        self.declare_parameter("close_angle_tenths", 18720)
        self.declare_parameter("speed_tenths", 200)
        self.declare_parameter("retry_on_error", True)

        self._lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self.last_status = "starting"
        self.status_pub = self.create_publisher(String, "/arachne/hardware/gripper_status", 10)
        self.create_subscription(
            String, str(self.get_parameter("command_topic").value), self._on_command, 10
        )
        self.create_subscription(
            Float64, str(self.get_parameter("angle_topic").value), self._on_angle, 10
        )
        self.create_timer(1.0, self._publish_status)
        self._open_serial()

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "close":
            self._send_motion(
                direction=1,
                angle_tenths=int(self.get_parameter("close_angle_tenths").value),
                label="close",
            )
        elif command in ("open", "home"):
            self._send_motion(
                direction=0,
                angle_tenths=int(self.get_parameter("open_angle_tenths").value),
                label="open",
            )
        elif command == "stop":
            self._send_motion(direction=0, angle_tenths=0, speed_tenths=0, label="stop")
        else:
            self.get_logger().warning(f"Ignoring unknown gripper command: {msg.data!r}")

    def _on_angle(self, msg: Float64) -> None:
        direction = 1 if msg.data >= 0.0 else 0
        angle = int(round(abs(float(msg.data)) * 10.0))
        self._send_motion(direction=direction, angle_tenths=angle, label="angle")

    def _send_motion(
        self,
        *,
        direction: int,
        angle_tenths: int,
        label: str,
        speed_tenths: int | None = None,
    ) -> None:
        speed = int(self.get_parameter("speed_tenths").value if speed_tenths is None else speed_tenths)
        frame = self._build_frame(
            device_id=int(self.get_parameter("device_id").value),
            mode=int(self.get_parameter("mode").value),
            direction=direction,
            sub_divide=int(self.get_parameter("sub_divide").value),
            angle_tenths=angle_tenths,
            speed_tenths=speed,
        )
        self._write_frame(frame)
        self.last_status = (
            f"{label}: dir={direction} angle={max(0, min(angle_tenths, 65535))} "
            f"speed={max(0, min(speed, 65535))} sub_divide={self.get_parameter('sub_divide').value}"
        )
        self.get_logger().info(self.last_status)

    def _build_frame(
        self,
        *,
        device_id: int,
        mode: int,
        direction: int,
        sub_divide: int,
        angle_tenths: int,
        speed_tenths: int,
    ) -> bytes:
        angle = max(0, min(int(angle_tenths), 65535))
        speed = max(0, min(int(speed_tenths), 65535))
        payload = [
            0x7B,
            device_id & 0xFF,
            mode & 0xFF,
            direction & 0xFF,
            sub_divide & 0xFF,
            (angle >> 8) & 0xFF,
            angle & 0xFF,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
        ]
        payload.append(self._xor(payload))
        payload.append(0x7D)
        return bytes(payload)

    def _xor(self, values: Iterable[int]) -> int:
        result = 0
        for value in values:
            result ^= value & 0xFF
        return result

    def _open_serial(self) -> None:
        with self._lock:
            if self._serial is not None and self._serial.is_open:
                return
            port = str(self.get_parameter("port").value)
            baudrate = int(self.get_parameter("baudrate").value)
            timeout = float(self.get_parameter("timeout_sec").value)
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
            )
            self.last_status = f"serial open: {port} @ {baudrate}"
            self.get_logger().info(self.last_status)

    def _close_serial(self) -> None:
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def _write_frame(self, frame: bytes) -> None:
        try:
            self._open_serial()
            self._write_frame_once(frame)
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warning(f"MS42DC serial write failed once: {exc}")
            self._close_serial()
            if not bool(self.get_parameter("retry_on_error").value):
                raise
            self._open_serial()
            self._write_frame_once(frame)

    def _write_frame_once(self, frame: bytes) -> None:
        with self._lock:
            if self._serial is None:
                raise RuntimeError("serial port is not open")
            self._serial.write(frame)
            self._serial.flush()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self.last_status
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MS42DCDirectSerialDriver()
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
