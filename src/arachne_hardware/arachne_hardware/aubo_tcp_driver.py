from __future__ import annotations

import math
import socket
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String

from arachne_hardware.aubo_sdk import (
    DEFAULT_AUBO_CONTROL_OWNER_PATH,
    DEFAULT_AUBO_TEACH_FLAG_PATH,
    AuboDirectJsonRpc,
    claim_control_owner as _claim_control_owner,
    clear_teach_gate,
    release_control_owner as _release_control_owner,
    set_teach_gate,
)
from arachne_hardware.aubo_sdk.safety import (
    exit_servo_mode,
    read_robot_state,
    stop_joint,
)
from arachne_hardware.aubo_sdk.teach import (
    read_teach_status,
    send_teach_rpc,
    wait_teach_disabled,
)
from arachne_hardware.aubo_sdk.velocity import speed_joint


class AuboOfficialStatusProbe(Node):
    """Connectivity probe for AUBO's official ROS2 driver path.

    Arachne should drive the real Aubo i5 through `AuboRobot/aubo_ros2_driver`
    and ros2_control. This node does not command the robot; it only publishes a
    small health string so the combined bringup has one status topic per device.
    """

    def __init__(self) -> None:
        super().__init__("aubo_official_status_probe")
        self.declare_parameter("robot_ip", "192.168.127.128")
        self.declare_parameter("aubo_port", 30004)
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
    """Bridge Arachne teach commands to AUBO hand-guiding mode.

    The ros2_control hardware interface keeps sending servoJoint hold commands
    while the robot is Running.  This bridge therefore toggles a local flag file
    first; the patched Arachne Aubo hardware interface sees the flag, stops
    servo mode, and keeps command targets synchronized to actual_q while the
    arm is being hand-guided.
    """

    def __init__(self) -> None:
        super().__init__("aubo_teach_command_bridge")
        self.declare_parameter("command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("robot_ip", "192.168.127.128")
        self.declare_parameter("rpc_port", 30004)
        self.declare_parameter("rpc_timeout_sec", 2.0)
        self.declare_parameter("teach_method", "freedrive")
        self.declare_parameter("teach_flag_path", DEFAULT_AUBO_TEACH_FLAG_PATH)
        self.declare_parameter("control_owner_path", DEFAULT_AUBO_CONTROL_OWNER_PATH)
        self.declare_parameter("control_owner_name", "teach_panel")
        self.declare_parameter("teach_enter_settle_sec", 0.35)
        self.declare_parameter("teach_exit_timeout_sec", 3.0)
        self.declare_parameter("teach_exit_poll_sec", 0.15)
        self.control_owner_active = False
        self.status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
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
        method = str(self.get_parameter("teach_method").value).strip() or "freedrive"
        flag_path = Path(str(self.get_parameter("teach_flag_path").value))
        owner_path = Path(str(self.get_parameter("control_owner_path").value))
        owner_name = str(self.get_parameter("control_owner_name").value).strip() or "teach_panel"
        ip = str(self.get_parameter("robot_ip").value)
        port = int(self.get_parameter("rpc_port").value)
        timeout = float(self.get_parameter("rpc_timeout_sec").value)
        owner_claimed = False
        try:
            owner_ok, owner_message = _claim_control_owner(owner_path, owner_name)
            if not owner_ok:
                self._publish_status(
                    f"aubo teach {method} refused; control {owner_message}", warn=True
                )
                return
            owner_claimed = True
            if enabled:
                set_teach_gate(flag_path, True)
                time.sleep(float(self.get_parameter("teach_enter_settle_sec").value))
                with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                    result = send_teach_rpc(rpc, method, True)
                    status = read_teach_status(rpc, method)
                self.control_owner_active = True
                self._publish_status(
                    f"aubo teach on active via {method}: result={result} status={status}"
                )
                return

            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                result = send_teach_rpc(rpc, method, False)
                status = wait_teach_disabled(
                    rpc,
                    method,
                    max(float(self.get_parameter("teach_exit_timeout_sec").value), 0.0),
                    max(float(self.get_parameter("teach_exit_poll_sec").value), 0.05),
                )
            self._clear_teach_flag(flag_path)
            _release_control_owner(owner_path, owner_name)
            self.control_owner_active = False
            self._publish_status(
                f"aubo teach off complete via {method}: result={result} status={status}"
            )
        except Exception as exc:  # pragma: no cover - depends on live hardware.
            if enabled:
                self._clear_teach_flag(flag_path)
                if owner_claimed and not self.control_owner_active:
                    _release_control_owner(owner_path, owner_name)
            else:
                self._publish_status(
                    f"aubo teach {method} failed; keeping ROS teach gate active: {exc}",
                    warn=True,
                )
                return
            self._publish_status(f"aubo teach {method} failed: {exc}", warn=True)

    def _send_teach_rpc(self, rpc: AuboDirectJsonRpc, method: str, enabled: bool) -> Any:
        return send_teach_rpc(rpc, method, enabled)

    def _read_teach_status(self, rpc: AuboDirectJsonRpc, method: str) -> Any:
        return read_teach_status(rpc, method)

    def _wait_teach_disabled(self, rpc: AuboDirectJsonRpc, method: str) -> Any:
        timeout = max(float(self.get_parameter("teach_exit_timeout_sec").value), 0.0)
        poll = max(float(self.get_parameter("teach_exit_poll_sec").value), 0.05)
        return wait_teach_disabled(rpc, method, timeout, poll)

    def _read_robot_state(self, rpc: AuboDirectJsonRpc) -> tuple[str, str]:
        return read_robot_state(rpc)

    def _clear_teach_flag(self, path: Path) -> None:
        try:
            clear_teach_gate(path)
        except OSError as exc:
            self._publish_status(f"could not clear teach flag {path}: {exc}", warn=True)
            return

    def _publish_status(self, text: str, *, warn: bool = False) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


class AuboSdkVelocityBridge(Node):
    """Bridge Arachne manual jog commands to AUBO SDK speedJoint.

    Manual jog should not depend on a high-frequency external servoJoint loop on
    the Jetson.  This node gates ros2_control's servoJoint writer, then lets the
    AUBO controller execute speed following internally.  Release and watchdog
    paths call stopJoint first, then release the gate so ros2_control can resume
    measured-position hold.
    """

    def __init__(self) -> None:
        super().__init__("aubo_sdk_velocity_bridge")
        self.declare_parameter("command_topic", "/arachne/aubo/joint_velocity_command")
        self.declare_parameter("robot_ip", "192.168.127.128")
        self.declare_parameter("rpc_port", 30004)
        self.declare_parameter("rpc_timeout_sec", 0.6)
        self.declare_parameter("teach_flag_path", DEFAULT_AUBO_TEACH_FLAG_PATH)
        self.declare_parameter("control_owner_path", DEFAULT_AUBO_CONTROL_OWNER_PATH)
        self.declare_parameter("control_owner_name", "teach_panel")
        self.declare_parameter("command_watchdog_sec", 0.75)
        self.declare_parameter("send_period_sec", 0.20)
        self.declare_parameter("gate_settle_sec", 0.08)
        self.declare_parameter("command_start_delay_sec", 0.04)
        self.declare_parameter("max_joint_speed_rad_sec", 0.25)
        self.declare_parameter("velocity_change_epsilon_rad_sec", 0.30)
        self.declare_parameter("speed_joint_accel_rad_sec2", 2.0)
        self.declare_parameter("speed_joint_time_sec", 100.0)
        self.declare_parameter("stop_joint_accel_rad_sec2", 8.0)
        self.declare_parameter("busy_retry_delay_sec", 0.04)
        self.declare_parameter("zero_epsilon_rad_sec", 1.0e-4)

        self.status_pub = self.create_publisher(String, "/arachne/hardware/aubo_status", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("command_topic").value),
            self._on_velocity_command,
            qos,
        )

        self.rpc: AuboDirectJsonRpc | None = None
        self.target_velocity: list[float] | None = None
        self.commanded_velocity: list[float] | None = None
        self.first_nonzero_command_stamp = 0.0
        self.last_command_stamp = 0.0
        self.last_send_stamp = 0.0
        self.active = False
        self.gate_owned = False
        self.control_owner_owned = False
        self.last_status_stamp = 0.0
        self.create_timer(0.01, self._tick)
        self._publish_status(
            "aubo sdk velocity bridge ready: "
            f"topic={self.get_parameter('command_topic').value}"
        )

    def destroy_node(self) -> bool:
        self._stop_velocity("node shutdown")
        self._close_rpc()
        return super().destroy_node()

    def _on_velocity_command(self, msg: Float64MultiArray) -> None:
        velocity = self._validated_velocity(list(msg.data))
        if velocity is None:
            return
        now = time.monotonic()
        previous = self.target_velocity
        self.target_velocity = velocity
        self.last_command_stamp = now
        if self._is_zero_velocity(velocity):
            self.first_nonzero_command_stamp = 0.0
            self._stop_velocity("zero command")
        elif previous is None or self._is_zero_velocity(previous):
            self.first_nonzero_command_stamp = now

    def _tick(self) -> None:
        velocity = self.target_velocity
        if velocity is None:
            return

        now = time.monotonic()
        watchdog = max(float(self.get_parameter("command_watchdog_sec").value), 0.04)
        if now - self.last_command_stamp > watchdog:
            self.target_velocity = None
            self._stop_velocity("watchdog timeout")
            return
        if self._is_zero_velocity(velocity):
            return

        start_delay = max(float(self.get_parameter("command_start_delay_sec").value), 0.0)
        if (
            not self.active
            and start_delay > 0.0
            and self.first_nonzero_command_stamp > 0.0
            and now - self.first_nonzero_command_stamp < start_delay
        ):
            return

        if self.commanded_velocity is not None and not self._velocity_changed_enough(
            velocity, self.commanded_velocity
        ):
            return

        period = max(float(self.get_parameter("send_period_sec").value), 0.01)
        if now - self.last_send_stamp < period:
            return
        if not self._ensure_active():
            return
        if self.commanded_velocity is not None:
            self._call_stop_joint("target update")
            self.commanded_velocity = None
            retry_delay = max(float(self.get_parameter("busy_retry_delay_sec").value), 0.0)
            if retry_delay > 0.0:
                time.sleep(retry_delay)
        if self._send_speed_joint(velocity):
            self.commanded_velocity = list(velocity)
            self.last_send_stamp = time.monotonic()

    def _validated_velocity(self, values: list[float]) -> list[float] | None:
        if len(values) != 6:
            self._publish_status(
                f"aubo sdk velocity ignored invalid length={len(values)}", warn=True
            )
            return None
        max_speed = max(float(self.get_parameter("max_joint_speed_rad_sec").value), 0.01)
        clamped: list[float] = []
        for value in values:
            value = float(value) if math.isfinite(float(value)) else 0.0
            clamped.append(max(-max_speed, min(max_speed, value)))
        return clamped

    def _is_zero_velocity(self, values: list[float]) -> bool:
        epsilon = max(float(self.get_parameter("zero_epsilon_rad_sec").value), 0.0)
        return all(abs(value) <= epsilon for value in values)

    def _velocity_changed_enough(self, target: list[float], commanded: list[float]) -> bool:
        epsilon = max(float(self.get_parameter("velocity_change_epsilon_rad_sec").value), 0.0)
        for target_value, commanded_value in zip(target, commanded):
            if target_value * commanded_value < 0.0:
                return True
            if abs(target_value - commanded_value) >= epsilon:
                return True
        return False

    def _ensure_active(self) -> bool:
        if self.active:
            return True
        if not self._robot_running():
            return False
        if not self._claim_control_owner():
            return False
        if not self._set_gate(True):
            self._release_control_owner()
            return False
        settle = max(float(self.get_parameter("gate_settle_sec").value), 0.0)
        if settle > 0.0:
            time.sleep(settle)
        self._exit_servo_mode()
        self._call_stop_joint("pre-speed cleanup", throttle_status=True)
        retry_delay = max(float(self.get_parameter("busy_retry_delay_sec").value), 0.0)
        if retry_delay > 0.0:
            time.sleep(retry_delay)
        self.active = True
        self._publish_status("aubo sdk velocity active")
        return True

    def _robot_running(self) -> bool:
        try:
            mode = str(self._robot_call("RobotState.getRobotModeType")).strip().lower()
            safety = str(self._robot_call("RobotState.getSafetyModeType")).strip().lower()
        except Exception as exc:
            self._publish_status_throttled(f"aubo sdk velocity state read failed: {exc}", warn=True)
            return False
        ready = mode == "running" and safety in ("normal", "reducedmode")
        if not ready:
            self._publish_status_throttled(
                f"aubo sdk velocity waiting for Running/Normal: mode={mode} safety={safety}",
                warn=True,
            )
        return ready

    def _send_speed_joint(self, velocity: list[float]) -> bool:
        accel = max(float(self.get_parameter("speed_joint_accel_rad_sec2").value), 0.05)
        duration = max(float(self.get_parameter("speed_joint_time_sec").value), 0.005)
        try:
            result = speed_joint(
                self._rpc(),
                velocity,
                accel,
                duration,
                stop_accel=max(float(self.get_parameter("stop_joint_accel_rad_sec2").value), 0.05),
                busy_retry_delay=max(float(self.get_parameter("busy_retry_delay_sec").value), 0.0),
                status=lambda text, warn: self._publish_status_throttled(text, warn=warn),
            )
        except Exception as exc:
            self._publish_status(f"aubo sdk speedJoint failed: {exc}", warn=True)
            self._close_rpc()
            self._stop_velocity("speedJoint failure", close_rpc=False)
            return False
        return result

    def _call_stop_joint(self, reason: str, *, throttle_status: bool = False) -> None:
        accel = max(float(self.get_parameter("stop_joint_accel_rad_sec2").value), 0.05)
        status = (
            (lambda text, warn: self._publish_status_throttled(text, warn=warn))
            if throttle_status
            else (lambda text, warn: self._publish_status(text, warn=warn))
        )
        try:
            stop_joint(self._rpc(), accel, reason, warn_only=True, status=status)
        except Exception as exc:
            status(f"aubo sdk stopJoint failed during {reason}: {exc}", True)

    def _stop_velocity(self, reason: str, *, close_rpc: bool = True) -> None:
        if not self.active:
            if self.gate_owned:
                self._set_gate(False)
            if self.control_owner_owned:
                self._release_control_owner()
            if close_rpc:
                self._close_rpc()
            self.commanded_velocity = None
            self.first_nonzero_command_stamp = 0.0
            return
        try:
            self._call_stop_joint(reason)
        finally:
            self.active = False
            self.commanded_velocity = None
            self.first_nonzero_command_stamp = 0.0
            self.last_send_stamp = 0.0
            if self.gate_owned:
                self._set_gate(False)
            if self.control_owner_owned:
                self._release_control_owner()
            if close_rpc:
                self._close_rpc()
            self._publish_status(f"aubo sdk velocity stopped: {reason}")

    def _exit_servo_mode(self) -> None:
        try:
            exit_servo_mode(
                self._rpc(),
                status=lambda text, warn: self._publish_status_throttled(text, warn=warn),
            )
        except Exception as fallback_exc:
            self._publish_status_throttled(
                f"aubo setServoMode(false) failed: {fallback_exc}", warn=True
            )

    def _robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        rpc = self._rpc()
        return rpc.robot_call(suffix, params)

    def _rpc(self) -> AuboDirectJsonRpc:
        if self.rpc is not None and self.rpc.sock is not None:
            return self.rpc
        ip = str(self.get_parameter("robot_ip").value)
        port = int(self.get_parameter("rpc_port").value)
        timeout = float(self.get_parameter("rpc_timeout_sec").value)
        self.rpc = AuboDirectJsonRpc(ip, port, timeout)
        self.rpc.connect()
        return self.rpc

    def _close_rpc(self) -> None:
        if self.rpc is not None:
            self.rpc.close()
            self.rpc = None

    def _claim_control_owner(self) -> bool:
        if self.control_owner_owned:
            return True
        path = Path(str(self.get_parameter("control_owner_path").value))
        owner = str(self.get_parameter("control_owner_name").value).strip() or "teach_panel"
        ok, message = _claim_control_owner(path, owner)
        if not ok:
            self._publish_status_throttled(
                f"aubo sdk velocity refused; control {message}", warn=True
            )
            return False
        self.control_owner_owned = True
        return True

    def _release_control_owner(self) -> None:
        path = Path(str(self.get_parameter("control_owner_path").value))
        owner = str(self.get_parameter("control_owner_name").value).strip() or "teach_panel"
        _release_control_owner(path, owner)
        self.control_owner_owned = False

    def _set_gate(self, enabled: bool) -> bool:
        path = Path(str(self.get_parameter("teach_flag_path").value))
        try:
            set_teach_gate(path, enabled)
        except OSError as exc:
            self._publish_status(f"aubo sdk velocity gate update failed: {exc}", warn=True)
            return False
        self.gate_owned = enabled
        return True

    def _publish_status_throttled(self, text: str, *, warn: bool = False) -> None:
        now = time.monotonic()
        if now - self.last_status_stamp < 1.0:
            return
        self.last_status_stamp = now
        self._publish_status(text, warn=warn)

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


def sdk_velocity_bridge_main() -> None:
    rclpy.init()
    node = AuboSdkVelocityBridge()
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
