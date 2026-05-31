#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any


DEFAULT_IP = "192.168.127.128"
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only Aubo startup readiness check. This script never powers on, "
            "releases brakes, or changes servo mode."
        )
    )
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--allow-not-running",
        action="store_true",
        help="return success when safety is acceptable even if RobotMode is not Running",
    )
    args = parser.parse_args()

    with AuboJsonRpc(args.ip, args.timeout) as rpc:
        mode = str(rpc.robot_call("RobotState.getRobotModeType"))
        safety = str(rpc.robot_call("RobotState.getSafetyModeType"))
        operational = rpc.robot_call("RobotManage.getOperationalMode")
        control = rpc.robot_call("RobotManage.getRobotControlMode")
        sim = rpc.robot_call("RobotManage.isSimulationEnabled")
        joints = rpc.robot_call("RobotState.getJointPositions")

        print(f"connected: {args.ip} robot={rpc.robot_name}")
        print(f"mode: {mode}")
        print(f"safety: {safety}")
        print(f"operational_mode: {operational}")
        print(f"control_mode: {control}")
        print(f"simulation: {sim}")
        print(f"joints: {joints}")

    if safety not in SAFE_SAFETY_MODES:
        print(
            "ERROR: Aubo safety state is not safe for remote control. "
            "Resolve it on the teach pendant/control cabinet before retrying.",
            file=sys.stderr,
        )
        return 1

    if mode != "Running":
        print(
            "ERROR: Aubo is not in Running mode. Use the teach pendant/control "
            "cabinet to perform connect -> power on -> start, then rerun this check.",
            file=sys.stderr,
        )
        return 0 if args.allow_not_running else 3

    print("ready: Aubo is Running with acceptable safety state.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
