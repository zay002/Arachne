from __future__ import annotations

import json
import socket
from typing import Any


class AuboDirectJsonRpc:
    """Small JSON-RPC client for the Aubo controller.

    This class intentionally has no ROS dependency so it can be shared by nodes,
    scripts, and future service wrappers.
    """

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

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if names:
            self.robot_name = str(names[0])

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
