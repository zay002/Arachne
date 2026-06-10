#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


JOINT_LIMITS_RAD = (
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
)


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict[str, Any]) -> None:
    body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _read_json(handler: BaseHTTPRequestHandler, max_bytes: int = 8 * 1024 * 1024) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0"))
    if length <= 0:
        return {}
    if length > max_bytes:
        raise ValueError(f"request too large: {length} bytes")
    raw = handler.rfile.read(length)
    return json.loads(raw.decode("utf-8"))


def _gpu_summary() -> list[dict[str, Any]]:
    try:
        output = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=index,name,memory.total,memory.used",
                "--format=csv,noheader,nounits",
            ],
            text=True,
            timeout=2.0,
        )
    except Exception:
        return []
    gpus = []
    for line in output.splitlines():
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 4:
            continue
        gpus.append(
            {
                "index": int(parts[0]),
                "name": parts[1],
                "memory_total_mib": int(float(parts[2])),
                "memory_used_mib": int(float(parts[3])),
            }
        )
    return gpus


def _validate_joints(joints: Any) -> list[str]:
    issues = []
    if not isinstance(joints, list) or len(joints) != 6:
        return ["current_joints_rad must be a 6-element list"]
    for index, value in enumerate(joints):
        try:
            q = float(value)
        except Exception:
            issues.append(f"joint {index + 1} is not numeric")
            continue
        low, high = JOINT_LIMITS_RAD[index]
        if q < low or q > high:
            issues.append(f"joint {index + 1}={q:.3f} outside coarse limit [{low:.3f},{high:.3f}]")
    return issues


def _validate_targets(targets: Any, constraints: dict[str, Any]) -> list[str]:
    issues = []
    if not isinstance(targets, list) or not targets:
        return ["targets must be a non-empty list"]
    ground_z = float(constraints.get("ground_min_z_base", -0.22))
    tool_clearance = max(float(constraints.get("tool_ground_clearance", 0.015)), 0.0)
    hard_floor = ground_z + tool_clearance
    for index, target in enumerate(targets):
        if not isinstance(target, dict):
            issues.append(f"target {index} is not an object")
            continue
        xyz = target.get("xyz_base")
        if not isinstance(xyz, list) or len(xyz) != 3:
            issues.append(f"target {index} missing xyz_base[3]")
            continue
        try:
            x, y, z = [float(v) for v in xyz]
        except Exception:
            issues.append(f"target {index} xyz_base contains non-numeric values")
            continue
        if not (-1.0 <= x <= 1.6 and -1.2 <= y <= 1.2 and -0.35 <= z <= 1.4):
            issues.append(f"target {index} xyz=({x:.3f},{y:.3f},{z:.3f}) outside coarse workspace")
        if z < hard_floor:
            issues.append(
                f"target {index} z={z:.3f} below ground/tool floor {hard_floor:.3f}"
            )
    return issues


class RemotePlannerService:
    def __init__(self, log_dir: Path) -> None:
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.started_at = time.time()

    def health(self) -> dict[str, Any]:
        return {
            "ok": True,
            "service": "arachne_remote_planner",
            "uptime_sec": round(time.time() - self.started_at, 3),
            "gpu": _gpu_summary(),
            "capabilities": {
                "constraint_audit": True,
                "remote_moveit": False,
                "gpu_inference": False,
                "trajectory_optimization": False,
            },
        }

    def plan(self, request: dict[str, Any]) -> dict[str, Any]:
        request_id = str(request.get("request_id") or uuid.uuid4().hex)
        constraints = request.get("constraints") if isinstance(request.get("constraints"), dict) else {}
        issues = []
        issues.extend(_validate_joints(request.get("current_joints_rad")))
        issues.extend(_validate_targets(request.get("targets"), constraints))
        self._write_request_log(request_id, request, issues)
        return {
            "request_id": request_id,
            "ok": False,
            "status": "remote_planner_backend_not_configured",
            "message": (
                "Remote service received the request and audited safety constraints. "
                "Install/enable a server-side planner plugin before returning executable joint paths."
            ),
            "constraint_issues": issues,
            "recommended_server_pipeline": [
                "load Arachne URDF/SRDF and current joint state",
                "build full collision scene: vehicle, basket, ground, target cloud, gripper volume",
                "sample 6D grasp poses with top-down and finger-axis constraints",
                "run OMPL/TrajOpt/STOMP candidates in parallel",
                "time-parameterize with joint speed/acceleration/jerk limits",
                "return only validated joint waypoints plus semantic gripper events",
            ],
            "trajectory": None,
        }

    def _write_request_log(self, request_id: str, request: dict[str, Any], issues: list[str]) -> None:
        payload = {
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "request_id": request_id,
            "issues": issues,
            "request": request,
        }
        path = self.log_dir / f"{request_id}.json"
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def make_handler(service: RemotePlannerService):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ArachneRemotePlanner/0.1"

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"{self.address_string()} - {fmt % args}", flush=True)

        def do_GET(self) -> None:
            if self.path.rstrip("/") == "/health":
                _json_response(self, 200, service.health())
            else:
                _json_response(self, 404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:
            try:
                payload = _read_json(self)
            except Exception as exc:
                _json_response(self, 400, {"ok": False, "error": str(exc)})
                return
            if self.path.rstrip("/") == "/plan":
                _json_response(self, 200, service.plan(payload))
            else:
                _json_response(self, 404, {"ok": False, "error": "not found"})

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Arachne remote planning service skeleton.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()

    service = RemotePlannerService(Path(args.log_dir).expanduser().resolve())
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(f"Arachne remote planner listening on http://{args.host}:{args.port}", flush=True)
    print(f"request logs: {service.log_dir}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
