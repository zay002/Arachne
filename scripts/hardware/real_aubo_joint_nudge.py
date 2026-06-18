#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any


DEFAULT_IP = "192.168.127.128"
SAFE_SAFETY = {"normal", "reducedmode"}


class Rpc:
    def __init__(self, ip: str, port: int, timeout: float) -> None:
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def connect(self) -> dict[str, Any]:
        try:
            self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
            self.sock.settimeout(self.timeout)
            response = self.raw_call("getRobotNames")
            if response.get("result"):
                self.robot_name = str(response["result"][0])
            return response
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def raw_call(self, method: str, params: list[Any] | None = None) -> dict[str, Any]:
        if self.sock is None:
            return {"error": "not connected"}
        self.request_id += 1
        request = {"jsonrpc": "2.0", "method": method, "params": params or [], "id": self.request_id}
        try:
            self.sock.sendall(json.dumps(request, separators=(",", ":")).encode())
            return json.loads(self.sock.recv(8192).decode("utf-8", errors="replace"))
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}

    def call(self, suffix: str, params: list[Any] | None = None) -> dict[str, Any]:
        return self.raw_call(f"{self.robot_name}.{suffix}", params)


def result(response: dict[str, Any]) -> Any:
    return response.get("result") if response.get("error") in (None, "", "None") else response.get("error")


def print_call(label: str, response: dict[str, Any]) -> None:
    print(f"{label}: result={response.get('result')} error={response.get('error')}")


def joints_from(response: dict[str, Any]) -> list[float]:
    value = response.get("result")
    if not isinstance(value, list) or len(value) != 6:
        raise RuntimeError(f"bad joint response: {response}")
    return [float(item) for item in value]


def wait_arrival(rpc: Rpc, target: list[float], timeout: float, tolerance: float) -> None:
    deadline = time.monotonic() + timeout
    last_error = float("inf")
    while time.monotonic() < deadline:
        current = joints_from(rpc.call("RobotState.getJointPositions"))
        last_error = max(abs(math.atan2(math.sin(t - c), math.cos(t - c))) for t, c in zip(target, current))
        if last_error <= tolerance:
            print(f"arrival: ok max_error={last_error:.4f}rad")
            return
        time.sleep(0.05)
    raise TimeoutError(f"arrival timeout: max_error={last_error:.4f}rad tolerance={tolerance:.4f}rad")


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal real Aubo moveJoint nudge and return test.")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=30004)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--joint", type=int, default=6, choices=range(1, 7))
    parser.add_argument("--delta-deg", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.08)
    parser.add_argument("--accel", type=float, default=0.20)
    parser.add_argument("--arrival-timeout", type=float, default=10.0)
    parser.add_argument("--tolerance", type=float, default=0.02)
    parser.add_argument("--skip-enable", action="store_true")
    args = parser.parse_args()

    rpc = Rpc(args.ip, args.port, args.timeout)
    try:
        print_call("connect/getRobotNames", rpc.connect())
        if rpc.sock is None:
            return 1

        mode_response = rpc.call("RobotState.getRobotModeType")
        safety_response = rpc.call("RobotState.getSafetyModeType")
        print_call("mode", mode_response)
        print_call("safety", safety_response)

        mode = str(result(mode_response)).strip()
        safety = str(result(safety_response)).strip()
        if not args.skip_enable and mode != "Running":
            print_call("enable/poweron", rpc.call("RobotManage.poweron"))
            print_call("enable/startup", rpc.call("RobotManage.startup"))
            mode_response = rpc.call("RobotState.getRobotModeType")
            safety_response = rpc.call("RobotState.getSafetyModeType")
            print_call("mode_after_enable", mode_response)
            print_call("safety_after_enable", safety_response)
            mode = str(result(mode_response)).strip()
            safety = str(result(safety_response)).strip()
        else:
            print("enable: skipped" if args.skip_enable else "enable: already Running")

        if mode.lower() != "running" or safety.lower() not in SAFE_SAFETY:
            print(f"refusing moveJoint: mode={mode} safety={safety}", file=sys.stderr)
            return 2

        print_call("setServoModeSelect(0)", rpc.call("MotionControl.setServoModeSelect", [0]))
        print_call("stopJoint/pre", rpc.call("MotionControl.stopJoint", [args.accel]))
        home = joints_from(rpc.call("RobotState.getJointPositions"))
        target = list(home)
        target[args.joint - 1] += math.radians(args.delta_deg)

        move = rpc.call("MotionControl.moveJoint", [target, args.accel, args.speed, 0.0, 0.0])
        print_call("moveJoint/out", move)
        if result(move) not in (0, None):
            return 3
        wait_arrival(rpc, target, args.arrival_timeout, args.tolerance)

        back = rpc.call("MotionControl.moveJoint", [home, args.accel, args.speed, 0.0, 0.0])
        print_call("moveJoint/back", back)
        if result(back) not in (0, None):
            return 4
        wait_arrival(rpc, home, args.arrival_timeout, args.tolerance)
        print_call("stopJoint/post", rpc.call("MotionControl.stopJoint", [args.accel]))
        print("ok: Aubo SDK moveJoint can move the arm and return")
        return 0
    finally:
        rpc.close()


if __name__ == "__main__":
    raise SystemExit(main())
