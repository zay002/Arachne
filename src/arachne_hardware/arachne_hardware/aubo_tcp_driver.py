from __future__ import annotations

import json
import math
import socket
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String


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


class AuboDirectJsonRpc:
    def __init__(self, ip: str, port: int, timeout: float) -> None:
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def __enter__(self) -> "AuboDirectJsonRpc":
        self.connect()
        return self

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if names:
            self.robot_name = str(names[0])

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def call(self, method: str, params: list[Any] | None = None) -> Any:
        if self.sock is None:
            raise RuntimeError("not connected")
        self.request_id += 1
        request = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params or [],
            "id": self.request_id,
        }
        self.sock.sendall(json.dumps(request, separators=(",", ":")).encode("utf-8"))
        response = json.loads(self.sock.recv(8192).decode("utf-8", errors="replace"))
        if response.get("error") not in (None, "", "None", "null"):
            raise RuntimeError(f"{method} failed: {response['error']}")
        return response.get("result")

    def robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        return self.call(f"{self.robot_name}.{suffix}", params)


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
        self.declare_parameter("teach_flag_path", "/tmp/arachne_aubo_teach_mode")
        self.declare_parameter("teach_enter_settle_sec", 0.35)
        self.declare_parameter("teach_exit_timeout_sec", 3.0)
        self.declare_parameter("teach_exit_poll_sec", 0.15)
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
        ip = str(self.get_parameter("robot_ip").value)
        port = int(self.get_parameter("rpc_port").value)
        timeout = float(self.get_parameter("rpc_timeout_sec").value)
        try:
            if enabled:
                flag_path.write_text("1\n", encoding="utf-8")
                time.sleep(float(self.get_parameter("teach_enter_settle_sec").value))
                with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                    result = self._send_teach_rpc(rpc, method, True)
                    status = self._read_teach_status(rpc, method)
                self._publish_status(
                    f"aubo teach on active via {method}: result={result} status={status}"
                )
                return

            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                result = self._send_teach_rpc(rpc, method, False)
                status = self._wait_teach_disabled(rpc, method)
            self._clear_teach_flag(flag_path)
            self._publish_status(
                f"aubo teach off complete via {method}: result={result} status={status}"
            )
        except Exception as exc:  # pragma: no cover - depends on live hardware.
            if enabled:
                self._clear_teach_flag(flag_path)
            else:
                self._publish_status(
                    f"aubo teach {method} failed; keeping ROS teach gate active: {exc}",
                    warn=True,
                )
                return
            self._publish_status(f"aubo teach {method} failed: {exc}", warn=True)

    def _send_teach_rpc(self, rpc: AuboDirectJsonRpc, method: str, enabled: bool) -> Any:
        if method == "freedrive":
            return rpc.robot_call("RobotManage.freedrive", [enabled])
        if method == "backdrive":
            return rpc.robot_call("RobotManage.backdrive", [enabled])
        if method == "handguide":
            if enabled:
                return rpc.robot_call("RobotManage.handguideMode", [[], []])
            return rpc.robot_call("RobotManage.exitHandguideMode")
        raise RuntimeError(f"unsupported Aubo teach_method: {method}")

    def _read_teach_status(self, rpc: AuboDirectJsonRpc, method: str) -> Any:
        try:
            if method == "freedrive":
                return rpc.robot_call("RobotManage.isFreedriveEnabled")
            if method == "backdrive":
                return rpc.robot_call("RobotManage.isBackdriveEnabled")
            if method == "handguide":
                return rpc.robot_call("RobotManage.getHandguideStatus")
        except Exception as exc:
            return f"status unavailable: {exc}"
        return "unknown"

    def _wait_teach_disabled(self, rpc: AuboDirectJsonRpc, method: str) -> Any:
        timeout = max(float(self.get_parameter("teach_exit_timeout_sec").value), 0.0)
        poll = max(float(self.get_parameter("teach_exit_poll_sec").value), 0.05)
        deadline = time.monotonic() + timeout
        status: Any = "unknown"
        while time.monotonic() <= deadline:
            status = self._read_teach_status(rpc, method)
            mode, safety = self._read_robot_state(rpc)
            teach_disabled = status is False or str(status).strip().lower() in (
                "false",
                "0",
                "disabled",
                "off",
            )
            state_ready = mode == "running" and safety in ("normal", "reducedmode")
            if teach_disabled and state_ready:
                return f"teach={status} mode={mode} safety={safety}"
            if isinstance(status, str) and status.startswith("status unavailable"):
                time.sleep(min(timeout, poll))
                status = f"{status}; mode={mode} safety={safety}"
            try:
                self._send_teach_rpc(rpc, method, False)
            except Exception as exc:
                status = f"disable retry failed: {exc}"
            time.sleep(poll)
        mode, safety = self._read_robot_state(rpc)
        raise TimeoutError(
            "teach mode did not return to Running/Normal before timeout; "
            f"last teach={status} mode={mode} safety={safety}"
        )

    def _read_robot_state(self, rpc: AuboDirectJsonRpc) -> tuple[str, str]:
        try:
            mode = str(rpc.robot_call("RobotState.getRobotModeType")).strip().lower()
            safety = str(rpc.robot_call("RobotState.getSafetyModeType")).strip().lower()
            return mode, safety
        except Exception as exc:
            return "unknown", f"unknown:{exc}"

    def _clear_teach_flag(self, path: Path) -> None:
        try:
            path.unlink(missing_ok=True)
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
        self.declare_parameter("teach_flag_path", "/tmp/arachne_aubo_teach_mode")
        self.declare_parameter("command_watchdog_sec", 0.16)
        self.declare_parameter("send_period_sec", 0.04)
        self.declare_parameter("gate_settle_sec", 0.08)
        self.declare_parameter("max_joint_speed_rad_sec", 0.25)
        self.declare_parameter("speed_joint_accel_rad_sec2", 2.0)
        self.declare_parameter("speed_joint_time_sec", 0.04)
        self.declare_parameter("stop_joint_accel_rad_sec2", 8.0)
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
        self.last_command_stamp = 0.0
        self.last_send_stamp = 0.0
        self.active = False
        self.gate_owned = False
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
        self.target_velocity = velocity
        self.last_command_stamp = now
        if self._is_zero_velocity(velocity):
            self._stop_velocity("zero command")

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

        period = max(float(self.get_parameter("send_period_sec").value), 0.01)
        if now - self.last_send_stamp < period:
            return
        if not self._ensure_active():
            return
        self._send_speed_joint(velocity)
        self.last_send_stamp = now

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

    def _ensure_active(self) -> bool:
        if self.active:
            return True
        if not self._robot_running():
            return False
        self._set_gate(True)
        settle = max(float(self.get_parameter("gate_settle_sec").value), 0.0)
        if settle > 0.0:
            time.sleep(settle)
        self._exit_servo_mode()
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

    def _send_speed_joint(self, velocity: list[float]) -> None:
        accel = max(float(self.get_parameter("speed_joint_accel_rad_sec2").value), 0.05)
        duration = max(float(self.get_parameter("speed_joint_time_sec").value), 0.005)
        try:
            result = self._robot_call("MotionControl.speedJoint", [velocity, accel, duration])
        except Exception as exc:
            self._publish_status(f"aubo sdk speedJoint failed: {exc}", warn=True)
            self._close_rpc()
            self._stop_velocity("speedJoint failure", close_rpc=False)
            return
        if result not in (0, None):
            self._publish_status_throttled(f"aubo sdk speedJoint result={result}", warn=True)

    def _stop_velocity(self, reason: str, *, close_rpc: bool = True) -> None:
        if not self.active:
            if self.gate_owned:
                self._set_gate(False)
            if close_rpc:
                self._close_rpc()
            return
        try:
            accel = max(float(self.get_parameter("stop_joint_accel_rad_sec2").value), 0.05)
            result = self._robot_call("MotionControl.stopJoint", [accel])
            if result not in (0, None):
                self._publish_status(f"aubo sdk stopJoint result={result}", warn=True)
        except Exception as exc:
            self._publish_status(f"aubo sdk stopJoint failed during {reason}: {exc}", warn=True)
        finally:
            self.active = False
            self.last_send_stamp = 0.0
            if self.gate_owned:
                self._set_gate(False)
            if close_rpc:
                self._close_rpc()
            self._publish_status(f"aubo sdk velocity stopped: {reason}")

    def _exit_servo_mode(self) -> None:
        try:
            result = self._robot_call("MotionControl.setServoModeSelect", [0])
            if result not in (0, None):
                self._publish_status_throttled(
                    f"aubo setServoModeSelect(0) result={result}", warn=True
                )
        except Exception as exc:
            self._publish_status_throttled(
                f"aubo setServoModeSelect unavailable, trying deprecated setServoMode(false): {exc}",
                warn=True,
            )
            try:
                self._robot_call("MotionControl.setServoMode", [False])
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

    def _set_gate(self, enabled: bool) -> None:
        path = Path(str(self.get_parameter("teach_flag_path").value))
        try:
            if enabled:
                path.write_text("1\n", encoding="utf-8")
            else:
                path.unlink(missing_ok=True)
        except OSError as exc:
            self._publish_status(f"aubo sdk velocity gate update failed: {exc}", warn=True)
            return
        self.gate_owned = enabled

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
