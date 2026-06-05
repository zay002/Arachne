#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_IP = "192.168.127.128"
JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
SAFE_SAFETY_MODES = {"Normal", "ReducedMode"}
REMOTE_START_TRANSITION_MODES = {"PowerOff", "Booting"}
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")


def safety_allows_prestart(mode: str, safety: str) -> bool:
    return mode in REMOTE_START_TRANSITION_MODES and safety == "Undefined"


class AuboJsonRpc:
    def __init__(self, ip: str, timeout: float) -> None:
        self.ip = ip
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def __enter__(self) -> "AuboJsonRpc":
        self.sock = socket.create_connection((self.ip, 30004), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if names:
            self.robot_name = str(names[0])
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self.sock is not None:
            self.sock.close()

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
        if "error" in response:
            raise RuntimeError(f"{method} failed: {response['error']}")
        return response.get("result")

    def robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        return self.call(f"{self.robot_name}.{suffix}", params)

    def mode(self) -> str:
        return str(self.robot_call("RobotState.getRobotModeType"))

    def safety(self) -> str:
        return str(self.robot_call("RobotState.getSafetyModeType"))


class AuboVelocityStartupMonitor(Node):
    def __init__(self, joint_state_topic: str, velocity_topic: str) -> None:
        super().__init__("arachne_aubo_remote_start")
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, velocity_topic, 10)

    def wait_for_positions(self, timeout: float) -> list[float]:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.positions for name in JOINTS):
                return [self.positions[name] for name in JOINTS]
            rclpy.spin_once(self, timeout_sec=0.05)
        missing = [name for name in JOINTS if name not in self.positions]
        raise TimeoutError(f"missing Aubo joint states: {missing}")

    def publish_zero_velocity(self, label: str, count: int = 5, period: float = 0.04) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0 for _ in JOINTS]
        print(f"publish zero velocity ({label})")
        for _ in range(max(count, 1)):
            self.velocity_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(max(period, 0.0))

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS:
                self.positions[name] = float(position)


def run_checked(command: list[str]) -> str:
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)} failed:\n{result.stdout}")
    return result.stdout


def strip_ansi(text: str) -> str:
    return ANSI_ESCAPE_RE.sub("", text)


def ensure_controllers_active(timeout: float, poll: float) -> None:
    required_names = ("joint_state_broadcaster", "forward_command_controller_velocity")
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        try:
            output = strip_ansi(run_checked(["ros2", "control", "list_controllers"]))
            last_output = output.strip()
        except RuntimeError as exc:
            last_output = str(exc)
            print("waiting for controller manager")
            time.sleep(poll)
            continue
        required = {name: False for name in required_names}
        for line in output.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[0] in required and parts[-1] == "active":
                required[parts[0]] = True
        missing = [name for name, active in required.items() if not active]
        if not missing:
            print(last_output)
            print("controllers active: blocking check complete")
            return
        print(f"waiting for active controllers: {missing}")
        time.sleep(poll)
    raise TimeoutError(
        "controllers did not become active before timeout. Last controller state:\n"
        f"{last_output}"
    )


def wait_for_mode(rpc: AuboJsonRpc, expected: set[str], timeout: float, poll: float) -> str:
    deadline = time.monotonic() + timeout
    last_mode = ""
    while time.monotonic() < deadline:
        mode = rpc.mode()
        safety = rpc.safety()
        print(f"state: mode={mode} safety={safety}")
        if safety not in SAFE_SAFETY_MODES and not safety_allows_prestart(mode, safety):
            raise RuntimeError(f"unsafe Aubo safety mode: {safety}")
        if mode in expected:
            return mode
        last_mode = mode
        time.sleep(poll)
    raise TimeoutError(f"timed out waiting for {sorted(expected)}; last mode={last_mode}")


def wait_until_joints_steady(
    node: AuboVelocityStartupMonitor,
    *,
    timeout: float,
    poll: float,
    velocity_tolerance: float = 0.02,
) -> None:
    deadline = time.monotonic() + timeout
    last_delta = float("inf")
    previous = node.wait_for_positions(timeout=timeout)
    while rclpy.ok() and time.monotonic() < deadline:
        time.sleep(poll)
        current = node.wait_for_positions(timeout=0.2)
        last_delta = max(abs(a - b) for a, b in zip(current, previous))
        if last_delta <= velocity_tolerance:
            print(f"joints steady after startup: max_delta={last_delta:.4f}")
            return
        previous = current
    raise TimeoutError(f"joints did not settle after startup: last_delta={last_delta:.4f}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Safely transition a real Aubo arm to ROS zero-speed hold using the "
            "Aubo lifecycle startup API."
        )
    )
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--state-timeout", type=float, default=12.0)
    parser.add_argument("--power-timeout", type=float, default=45.0)
    parser.add_argument("--startup-timeout", type=float, default=45.0)
    parser.add_argument("--steady-timeout", type=float, default=8.0)
    parser.add_argument("--controller-timeout", type=float, default=30.0)
    parser.add_argument("--poll", type=float, default=0.5)
    parser.add_argument(
        "--action-name",
        default="/joint_trajectory_controller/follow_joint_trajectory",
        help="Deprecated; velocity startup no longer uses FollowJointTrajectory.",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--velocity-topic",
        default="/forward_command_controller_velocity/commands",
    )
    args = parser.parse_args()

    print(
        "safety note: this flow uses RobotManage.startup and never calls "
        "releaseRobotBrake directly."
    )
    print("blocking step 1/8: wait for active ROS controllers")
    ensure_controllers_active(args.controller_timeout, args.poll)

    rclpy.init()
    node = AuboVelocityStartupMonitor(args.joint_state_topic, args.velocity_topic)
    try:
        print("blocking step 2/8: read measured joint state")
        node.wait_for_positions(args.state_timeout)
        print("blocking step 3/8: publish zero velocity before power transition")
        node.publish_zero_velocity("before-power")

        with AuboJsonRpc(args.ip, args.rpc_timeout) as rpc:
            mode = rpc.mode()
            safety = rpc.safety()
            print(f"initial rpc state: mode={mode} safety={safety}")
            if safety not in SAFE_SAFETY_MODES and not safety_allows_prestart(mode, safety):
                raise RuntimeError(f"unsafe Aubo safety mode: {safety}")

            if mode != "Running":
                if mode != "Idle":
                    print("blocking step 4/8: power on and wait for Idle")
                    print("poweron")
                    print(f"poweron result: {rpc.robot_call('RobotManage.poweron')}")
                    wait_for_mode(rpc, {"Idle", "Running"}, args.power_timeout, args.poll)
                else:
                    print("blocking step 4/8: robot already Idle")

                print("blocking step 5/8: refresh measured joint state before startup")
                node.wait_for_positions(args.state_timeout)
                node.publish_zero_velocity("before-startup")

                print("blocking step 6/8: call RobotManage.startup and wait for Running")
                print("startup")
                print(f"startup result: {rpc.robot_call('RobotManage.startup')}")
                wait_for_mode(rpc, {"Running"}, args.startup_timeout, args.poll)

                print("blocking step 7/8: wait for joint state to settle after startup")
                wait_until_joints_steady(
                    node,
                    timeout=args.steady_timeout,
                    poll=args.poll,
                )
            else:
                print("blocking steps 4-7/8: Aubo is already Running; keeping zero velocity hold.")

        print("blocking step 8/8: verify velocity hold feedback after Running")
        node.publish_zero_velocity("after-running")
        wait_until_joints_steady(
            node,
            timeout=args.steady_timeout,
            poll=args.poll,
        )
        print("Aubo remote startup complete: velocity controller active, zero-speed hold verified.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
