#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
from typing import Any


DEFAULT_IP = "192.168.127.128"
DEFAULT_MASS_KG = 2.5
DEFAULT_COG = "0,0,0"
DEFAULT_AOM = "0,0,0"
DEFAULT_INERTIA = "0,0,0,0,0,0"


class AuboJsonRpc:
    def __init__(self, ip: str, port: int, timeout: float) -> None:
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.request_id = 0
        self.robot_name = "rob1"
        self.sock: socket.socket | None = None

    def __enter__(self) -> "AuboJsonRpc":
        self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
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


def parse_vector(text: str, *, expected: int, name: str) -> list[float]:
    values = [part.strip() for part in text.split(",")]
    if len(values) != expected:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected} comma-separated numbers; got {text!r}"
        )
    try:
        parsed = [float(value) for value in values]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} contains a non-number: {text!r}") from exc
    if not all(math.isfinite(value) for value in parsed):
        raise argparse.ArgumentTypeError(f"{name} values must be finite: {text!r}")
    return parsed


def payload_near(
    payload: list[Any],
    mass: float,
    cog: list[float],
    aom: list[float],
    inertia: list[float],
    *,
    tolerance: float = 1e-6,
) -> bool:
    if len(payload) != 4:
        return False
    current_mass = float(payload[0])
    current_cog = [float(value) for value in payload[1]]
    current_aom = [float(value) for value in payload[2]]
    current_inertia = [float(value) for value in payload[3]]
    vectors = (
        (current_cog, cog),
        (current_aom, aom),
        (current_inertia, inertia),
    )
    return (
        abs(current_mass - mass) <= tolerance
        and all(
            len(left) == len(right)
            and all(abs(a - b) <= tolerance for a, b in zip(left, right))
            for left, right in vectors
        )
    )


def format_payload(payload: list[Any]) -> str:
    return (
        f"mass={payload[0]}kg "
        f"cog={payload[1]} "
        f"aom={payload[2]} "
        f"inertia={payload[3]}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read or set the real Aubo controller payload through JSON-RPC."
    )
    parser.add_argument("--ip", default=os.environ.get("AUBO_ROBOT_IP", DEFAULT_IP))
    parser.add_argument("--port", type=int, default=30004)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--mass",
        type=float,
        default=float(os.environ.get("ARACHNE_AUBO_PAYLOAD_MASS", DEFAULT_MASS_KG)),
        help="Payload mass in kg.",
    )
    parser.add_argument(
        "--cog",
        default=os.environ.get("ARACHNE_AUBO_PAYLOAD_COG", DEFAULT_COG),
        help="Center of gravity as x,y,z in meters, relative to the tool flange.",
    )
    parser.add_argument(
        "--aom",
        default=os.environ.get("ARACHNE_AUBO_PAYLOAD_AOM", DEFAULT_AOM),
        help="Axes of moment as rx,ry,rz. Defaults to controller-aligned axes.",
    )
    parser.add_argument(
        "--inertia",
        default=os.environ.get("ARACHNE_AUBO_PAYLOAD_INERTIA", DEFAULT_INERTIA),
        help="Inertia matrix components as ixx,iyy,izz,ixy,ixz,iyz.",
    )
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Confirm writing payload settings.")
    args = parser.parse_args()

    if not math.isfinite(args.mass) or args.mass < 0:
        raise SystemExit("payload mass must be a non-negative finite number")

    cog = parse_vector(args.cog, expected=3, name="--cog")
    aom = parse_vector(args.aom, expected=3, name="--aom")
    inertia = parse_vector(args.inertia, expected=6, name="--inertia")

    with AuboJsonRpc(args.ip, args.port, args.timeout) as rpc:
        current = rpc.robot_call("RobotConfig.getPayload")
        print(f"current Aubo payload: {format_payload(current)}")

        if args.check_only:
            return 0

        confirmed = args.yes or os.environ.get("ARACHNE_CONFIRM_AUBO_PAYLOAD") == "YES"
        if not confirmed:
            print(
                "Refusing to write Aubo payload without confirmation. "
                "Rerun with --yes or ARACHNE_CONFIRM_AUBO_PAYLOAD=YES.",
                file=sys.stderr,
            )
            return 2

        if payload_near(current, args.mass, cog, aom, inertia):
            print("Aubo payload already matches requested values.")
            return 0

        print(
            "setting Aubo payload: "
            f"mass={args.mass}kg cog={cog} aom={aom} inertia={inertia}"
        )
        result = rpc.robot_call("RobotConfig.setPayload", [args.mass, cog, aom, inertia])
        print(f"setPayload result: {result}")
        updated = rpc.robot_call("RobotConfig.getPayload")
        print(f"updated Aubo payload: {format_payload(updated)}")

    if not payload_near(updated, args.mass, cog, aom, inertia):
        raise SystemExit("Aubo payload did not match requested values after setPayload")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
