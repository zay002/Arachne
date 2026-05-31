#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from typing import Any

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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


class HoldActionClient(Node):
    def __init__(self, action_name: str, joint_state_topic: str) -> None:
        super().__init__("arachne_aubo_remote_start")
        self.action = ActionClient(self, FollowJointTrajectory, action_name)
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)

    def wait_for_positions(self, timeout: float) -> list[float]:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.positions for name in JOINTS):
                return [self.positions[name] for name in JOINTS]
            rclpy.spin_once(self, timeout_sec=0.05)
        missing = [name for name in JOINTS if name not in self.positions]
        raise TimeoutError(f"missing Aubo joint states: {missing}")

    def send_hold(self, positions: list[float], *, duration: float, timeout: float, label: str) -> None:
        if not self.action.wait_for_server(timeout_sec=timeout):
            raise TimeoutError("joint trajectory action server is not available")

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = int(timeout)

        print(f"send hold action ({label}): {[round(value, 6) for value in positions]}")
        goal_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout)
        goal_handle = goal_future.result()
        if goal_handle is None:
            raise TimeoutError(f"hold action response timed out: {label}")
        if not goal_handle.accepted:
            raise RuntimeError(f"hold action rejected: {label}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + timeout)
        result_response = result_future.result()
        if result_response is None:
            raise TimeoutError(f"hold action result timed out: {label}")
        result = result_response.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"hold action failed ({label}): code={result.error_code} {result.error_string}"
            )
        self.wait_until_near(positions, timeout=timeout, label=label)

    def wait_until_near(
        self,
        target: list[float],
        *,
        timeout: float,
        tolerance: float = 0.03,
        label: str,
    ) -> None:
        deadline = time.monotonic() + timeout
        best_error = float("inf")
        while rclpy.ok() and time.monotonic() < deadline:
            current = self.wait_for_positions(timeout=0.2)
            error = max(abs(a - b) for a, b in zip(current, target))
            best_error = min(best_error, error)
            if error <= tolerance:
                print(f"hold verified ({label}): max_error={error:.4f}")
                return
        raise TimeoutError(f"hold verification failed ({label}): best_error={best_error:.4f}")

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


def ensure_controllers_active(timeout: float, poll: float) -> None:
    required_names = ("joint_state_broadcaster", "joint_trajectory_controller")
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        try:
            output = run_checked(["ros2", "control", "list_controllers"])
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
        if safety not in SAFE_SAFETY_MODES:
            raise RuntimeError(f"unsafe Aubo safety mode: {safety}")
        if mode in expected:
            return mode
        last_mode = mode
        time.sleep(poll)
    raise TimeoutError(f"timed out waiting for {sorted(expected)}; last mode={last_mode}")


def enable_servo_mode(rpc: AuboJsonRpc, timeout: float, poll: float) -> None:
    print("enable servo mode")
    result = rpc.robot_call("MotionControl.setServoMode", [True])
    print(f"setServoMode(true) result: {result}")
    deadline = time.monotonic() + timeout
    last_value: Any = None
    while time.monotonic() < deadline:
        last_value = rpc.robot_call("MotionControl.isServoModeEnabled")
        print(f"servo_mode_enabled={last_value}")
        if bool(last_value):
            return
        time.sleep(poll)
    raise TimeoutError(f"servo mode did not become enabled; last={last_value}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Safely transition a real Aubo arm to remote hold control."
    )
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--state-timeout", type=float, default=12.0)
    parser.add_argument("--power-timeout", type=float, default=45.0)
    parser.add_argument("--release-timeout", type=float, default=30.0)
    parser.add_argument("--servo-timeout", type=float, default=5.0)
    parser.add_argument("--controller-timeout", type=float, default=30.0)
    parser.add_argument("--poll", type=float, default=0.5)
    parser.add_argument("--hold-duration", type=float, default=1.0)
    parser.add_argument("--action-timeout", type=float, default=8.0)
    parser.add_argument(
        "--action-name",
        default="/joint_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    args = parser.parse_args()

    print("blocking step 1/8: wait for active ROS controllers")
    ensure_controllers_active(args.controller_timeout, args.poll)

    rclpy.init()
    node = HoldActionClient(args.action_name, args.joint_state_topic)
    try:
        print("blocking step 2/8: read measured joint state")
        current = node.wait_for_positions(args.state_timeout)
        print("blocking step 3/8: send pre-power hold command")
        node.send_hold(
            current,
            duration=args.hold_duration,
            timeout=args.action_timeout,
            label="before-power",
        )

        with AuboJsonRpc(args.ip, args.rpc_timeout) as rpc:
            mode = rpc.mode()
            safety = rpc.safety()
            print(f"initial rpc state: mode={mode} safety={safety}")
            if safety not in SAFE_SAFETY_MODES:
                raise RuntimeError(f"unsafe Aubo safety mode: {safety}")

            if mode != "Running":
                if mode != "Idle":
                    print("blocking step 4/8: power on and wait for Idle")
                    print("poweron")
                    print(f"poweron result: {rpc.robot_call('RobotManage.poweron')}")
                    wait_for_mode(rpc, {"Idle", "Running"}, args.power_timeout, args.poll)
                else:
                    print("blocking step 4/8: robot already Idle")

                print("blocking step 5/8: refresh measured joint state and hold before brake release")
                current = node.wait_for_positions(args.state_timeout)
                node.send_hold(
                    current,
                    duration=args.hold_duration,
                    timeout=args.action_timeout,
                    label="before-brake-release",
                )
                print("blocking step 6/8: enable servo mode and wait for confirmation")
                enable_servo_mode(rpc, args.servo_timeout, args.poll)

                print("blocking step 7/8: release brake and wait for Running")
                print("releaseRobotBrake")
                print(f"releaseRobotBrake result: {rpc.robot_call('RobotManage.releaseRobotBrake')}")
                wait_for_mode(rpc, {"Running"}, args.release_timeout, args.poll)
            else:
                print("blocking steps 4-7/8: Aubo is already Running; keeping current hold command.")

        print("blocking step 8/8: verify hold feedback after Running")
        current = node.wait_for_positions(args.state_timeout)
        node.send_hold(
            current,
            duration=args.hold_duration,
            timeout=args.action_timeout,
            label="after-running",
        )
        print("Aubo remote startup complete: controller active, servo hold verified.")
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
