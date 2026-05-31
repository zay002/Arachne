#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
from typing import Any


DEFAULT_IP = "192.168.127.128"
DEFAULT_PORTS = (80, 30002, 30003, 30004, 9012, 9013)
READ_ONLY_RPC_CALLS = (
    ("getRobotNames", []),
    ("rob1.RobotState.getJointPositions", []),
    ("rob1.RobotState.getTcpPose", []),
    ("rob1.RobotState.getRobotModeType", []),
    ("rob1.RobotState.getSafetyModeType", []),
)


def ping(ip: str) -> bool:
    result = subprocess.run(
        ["ping", "-c", "2", "-W", "1", ip],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    print(result.stdout.strip())
    return result.returncode == 0


def check_ports(ip: str, ports: tuple[int, ...], timeout: float) -> dict[int, str]:
    results: dict[int, str] = {}
    for port in ports:
        sock = socket.socket()
        sock.settimeout(timeout)
        try:
            sock.connect((ip, port))
            results[port] = "open"
        except Exception as exc:
            results[port] = type(exc).__name__
        finally:
            sock.close()
    return results


def rpc_call(sock: socket.socket, method: str, params: list[Any], request_id: int) -> dict[str, Any]:
    request = {
        "jsonrpc": "2.0",
        "method": method,
        "params": params,
        "id": request_id,
    }
    sock.sendall(json.dumps(request, separators=(",", ":")).encode("utf-8"))
    data = sock.recv(8192).decode("utf-8", errors="replace")
    return json.loads(data)


def read_rpc_state(ip: str, timeout: float) -> dict[str, Any]:
    results: dict[str, Any] = {}
    with socket.create_connection((ip, 30004), timeout=timeout) as sock:
        sock.settimeout(timeout)
        robot_name = "rob1"
        for index, (method, params) in enumerate(READ_ONLY_RPC_CALLS, start=1):
            active_method = method
            if method.startswith("rob1.") and robot_name != "rob1":
                active_method = method.replace("rob1.", f"{robot_name}.", 1)
            response = rpc_call(sock, active_method, params, index)
            if method == "getRobotNames" and response.get("result"):
                robot_name = str(response["result"][0])
            results[active_method] = response.get("result", response.get("error"))
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only Aubo controller connectivity probe.")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--ports",
        default=",".join(str(port) for port in DEFAULT_PORTS),
        help="comma-separated TCP ports to probe",
    )
    args = parser.parse_args()

    ports = tuple(int(item.strip()) for item in args.ports.split(",") if item.strip())

    print(f"== Aubo ping: {args.ip} ==")
    ping_ok = ping(args.ip)

    print("\n== TCP ports ==")
    ports_result = check_ports(args.ip, ports, args.timeout)
    for port, status in ports_result.items():
        print(f"{args.ip}:{port} {status}")

    print("\n== Read-only JSON-RPC state ==")
    try:
        rpc_state = read_rpc_state(args.ip, args.timeout)
    except Exception as exc:
        print(f"JSON-RPC failed: {type(exc).__name__}: {exc}")
        return 1
    for key, value in rpc_state.items():
        print(f"{key}: {value}")

    if not ping_ok:
        return 1
    if ports_result.get(30004) != "open":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
