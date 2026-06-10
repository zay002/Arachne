#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any


def _request_json(url: str, payload: dict[str, Any] | None = None, timeout: float = 5.0) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def _load_payload(path: str | None) -> dict[str, Any]:
    if not path:
        return {
            "request_id": uuid.uuid4().hex,
            "current_joints_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "targets": [
                {
                    "name": "example_grasp",
                    "xyz_base": [1.0, 0.0, -0.10],
                    "rpy_base": [0.0, 0.0, 0.0],
                    "phase": "grasp",
                }
            ],
            "constraints": {
                "ground_min_z_base": -0.22,
                "tool_ground_clearance": 0.015,
                "prefer_topdown": True,
                "max_joint_speed_rad_s": 0.6,
                "max_joint_accel_rad_s2": 3.0,
            },
        }
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Arachne remote planner client probe.")
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--payload")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--health", action="store_true")
    args = parser.parse_args()

    endpoint = args.url.rstrip("/")
    try:
        if args.health:
            result = _request_json(f"{endpoint}/health", timeout=args.timeout)
        else:
            result = _request_json(f"{endpoint}/plan", _load_payload(args.payload), args.timeout)
    except urllib.error.URLError as exc:
        raise SystemExit(f"remote planner request failed: {exc}") from exc
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
