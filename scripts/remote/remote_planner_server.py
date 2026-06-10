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


JOINT_NAMES = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)

JOINT_LIMITS_RAD = (
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
)

DEFAULT_MAX_SPEED_RAD_S = 0.6
DEFAULT_MAX_ACCEL_RAD_S2 = 3.0
DEFAULT_MIN_SEGMENT_SEC = 0.18
DEFAULT_MAX_SEGMENT_DELTA_RAD = 1.25


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


def _as_joint_vector(value: Any, label: str) -> tuple[list[float] | None, list[str]]:
    issues = []
    if not isinstance(value, list) or len(value) != 6:
        return None, [f"{label} must be a 6-element list"]
    joints = []
    for index, raw in enumerate(value):
        try:
            q = float(raw)
        except Exception:
            issues.append(f"{label}[{index}] is not numeric")
            continue
        if not math.isfinite(q):
            issues.append(f"{label}[{index}] is not finite")
            continue
        low, high = JOINT_LIMITS_RAD[index]
        if q < low or q > high:
            issues.append(f"{label}[{index}]={q:.3f} outside coarse limit [{low:.3f},{high:.3f}]")
        joints.append(q)
    if issues:
        return None, issues
    return joints, []


def _joint_delta(to_joints: list[float], from_joints: list[float]) -> list[float]:
    deltas = []
    for q_to, q_from in zip(to_joints, from_joints):
        delta = q_to - q_from
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        deltas.append(delta)
    return deltas


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


def _extract_waypoint_candidates(request: dict[str, Any]) -> tuple[list[dict[str, Any]], list[str]]:
    issues: list[str] = []
    current, current_issues = _as_joint_vector(request.get("current_joints_rad"), "current_joints_rad")
    issues.extend(current_issues)
    if current is None:
        return [], issues

    raw_candidates = request.get("candidates")
    if isinstance(raw_candidates, list) and raw_candidates:
        candidates = []
        for index, candidate in enumerate(raw_candidates):
            if not isinstance(candidate, dict):
                issues.append(f"candidate {index} is not an object")
                continue
            raw_waypoints = candidate.get("waypoints_rad")
            if not isinstance(raw_waypoints, list) or not raw_waypoints:
                issues.append(f"candidate {index} missing waypoints_rad")
                continue
            waypoints = [current]
            for wp_index, raw_wp in enumerate(raw_waypoints):
                joints, wp_issues = _as_joint_vector(raw_wp, f"candidate {index} waypoint {wp_index}")
                issues.extend(wp_issues)
                if joints is not None:
                    waypoints.append(joints)
            candidates.append(
                {
                    "label": str(candidate.get("label") or f"candidate_{index}"),
                    "score": float(candidate.get("score", 0.0) or 0.0),
                    "waypoints": waypoints,
                    "events": candidate.get("events") if isinstance(candidate.get("events"), list) else [],
                }
            )
        return candidates, issues

    raw_waypoints = request.get("joint_waypoints_rad")
    if isinstance(raw_waypoints, list) and raw_waypoints:
        waypoints = [current]
        for index, raw_wp in enumerate(raw_waypoints):
            joints, wp_issues = _as_joint_vector(raw_wp, f"joint_waypoints_rad[{index}]")
            issues.extend(wp_issues)
            if joints is not None:
                waypoints.append(joints)
        return [{"label": "joint_waypoints_rad", "score": 0.0, "waypoints": waypoints, "events": []}], issues

    targets = request.get("targets")
    if isinstance(targets, list):
        waypoints = [current]
        events: list[dict[str, Any]] = []
        for index, target in enumerate(targets):
            if not isinstance(target, dict):
                continue
            raw_goal = target.get("joint_goal_rad") or target.get("q_goal_rad")
            if raw_goal is None:
                continue
            joints, wp_issues = _as_joint_vector(raw_goal, f"target {index} joint_goal_rad")
            issues.extend(wp_issues)
            if joints is not None:
                waypoints.append(joints)
                events.append(
                    {
                        "waypoint_index": len(waypoints) - 1,
                        "name": str(target.get("name") or f"target_{index}"),
                        "phase": str(target.get("phase") or ""),
                    }
                )
        if len(waypoints) > 1:
            return [{"label": "target_joint_goals", "score": 0.0, "waypoints": waypoints, "events": events}], issues

    issues.append("request has no joint candidates: provide candidates[].waypoints_rad, joint_waypoints_rad, or target joint_goal_rad")
    return [], issues


def _dedupe_waypoints(waypoints: list[list[float]]) -> list[list[float]]:
    compact: list[list[float]] = []
    for waypoint in waypoints:
        if not compact:
            compact.append(waypoint)
            continue
        if max(abs(delta) for delta in _joint_delta(waypoint, compact[-1])) < 1e-5:
            continue
        compact.append(waypoint)
    return compact


def _time_parameterize(
    candidate: dict[str, Any], constraints: dict[str, Any]
) -> tuple[dict[str, Any] | None, list[str]]:
    issues: list[str] = []
    waypoints = _dedupe_waypoints(candidate["waypoints"])
    if len(waypoints) < 2:
        return None, ["candidate has fewer than two distinct waypoints"]

    max_speed = max(float(constraints.get("max_joint_speed_rad_s", DEFAULT_MAX_SPEED_RAD_S)), 0.05)
    max_accel = max(float(constraints.get("max_joint_accel_rad_s2", DEFAULT_MAX_ACCEL_RAD_S2)), 0.05)
    min_segment = max(float(constraints.get("min_segment_duration_sec", DEFAULT_MIN_SEGMENT_SEC)), 0.02)
    max_segment_delta = max(
        float(constraints.get("max_segment_joint_delta_rad", DEFAULT_MAX_SEGMENT_DELTA_RAD)), 0.05
    )

    frames: list[dict[str, Any]] = [
        {
            "time_from_start": 0.0,
            "positions": [round(v, 8) for v in waypoints[0]],
            "velocities": [0.0] * 6,
            "accelerations": [0.0] * 6,
        }
    ]
    segments = []
    total_time = 0.0
    max_seen_speed = 0.0
    for seg_index, (q0, q1) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        delta = _joint_delta(q1, q0)
        max_delta = max(abs(v) for v in delta)
        if max_delta > max_segment_delta:
            issues.append(
                f"segment {seg_index} max joint delta {max_delta:.3f}rad exceeds {max_segment_delta:.3f}rad"
            )
        duration = max(min_segment, max_delta / max_speed, math.sqrt(max_delta / max_accel) * 2.0)
        velocity = [v / duration for v in delta]
        max_seen_speed = max(max_seen_speed, max(abs(v) for v in velocity))
        total_time += duration
        segments.append(
            {
                "index": seg_index,
                "duration": round(duration, 4),
                "max_delta_rad": round(max_delta, 5),
                "max_speed_rad_s": round(max(abs(v) for v in velocity), 5),
            }
        )
        frames.append(
            {
                "time_from_start": round(total_time, 4),
                "positions": [round(v, 8) for v in q1],
                "velocities": [round(v, 8) for v in velocity],
                "accelerations": [0.0] * 6,
            }
        )
    frames[-1]["velocities"] = [0.0] * 6

    if issues:
        return None, issues
    return (
        {
            "joint_names": list(JOINT_NAMES),
            "frames": frames,
            "events": candidate.get("events", []),
            "segments": segments,
            "duration_sec": round(total_time, 4),
            "limits": {
                "max_joint_speed_rad_s": max_speed,
                "max_joint_accel_rad_s2": max_accel,
                "max_segment_joint_delta_rad": max_segment_delta,
            },
            "audit": {
                "raw_waypoints": len(candidate["waypoints"]),
                "distinct_waypoints": len(waypoints),
                "max_speed_rad_s": round(max_seen_speed, 5),
                "max_accel_rad_s2": 0.0,
            },
        },
        [],
    )


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
                "trajectory_optimization": True,
                "joint_waypoint_time_parameterization": True,
            },
        }

    def plan(self, request: dict[str, Any]) -> dict[str, Any]:
        request_id = str(request.get("request_id") or uuid.uuid4().hex)
        constraints = request.get("constraints") if isinstance(request.get("constraints"), dict) else {}
        issues = []
        issues.extend(_validate_joints(request.get("current_joints_rad")))
        issues.extend(_validate_targets(request.get("targets"), constraints))
        candidates, candidate_issues = _extract_waypoint_candidates(request)

        plans = []
        rejected = [{"label": "request", "issues": candidate_issues}] if candidate_issues else []
        for candidate in candidates:
            trajectory, plan_issues = _time_parameterize(candidate, constraints)
            if trajectory is None:
                rejected.append({"label": candidate["label"], "issues": plan_issues})
                continue
            travel = sum(segment["max_delta_rad"] for segment in trajectory["segments"])
            rank = travel - float(candidate.get("score", 0.0) or 0.0)
            plans.append({"rank": rank, "label": candidate["label"], "trajectory": trajectory})

        plans.sort(key=lambda item: item["rank"])
        if not candidates:
            issues.extend(candidate_issues)
        ok = bool(plans) and not issues
        selected = plans[0] if plans else None
        self._write_request_log(request_id, request, issues, selected["trajectory"] if selected else None, rejected)
        if ok and selected is not None:
            return {
                "request_id": request_id,
                "ok": True,
                "status": "joint_waypoint_plan",
                "message": "Remote service selected and time-parameterized a joint waypoint candidate.",
                "selected_candidate": selected["label"],
                "constraint_issues": [],
                "rejected_candidates": rejected,
                "trajectory": selected["trajectory"],
            }
        return {
            "request_id": request_id,
            "ok": False,
            "status": "missing_or_rejected_joint_candidates",
            "message": (
                "Remote service audited the request but could not return a safe joint trajectory. "
                "Provide joint candidates now; enable remote MoveIt/TrajOpt later for pose-only requests."
            ),
            "constraint_issues": issues,
            "rejected_candidates": rejected,
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

    def _write_request_log(
        self,
        request_id: str,
        request: dict[str, Any],
        issues: list[str],
        trajectory: dict[str, Any] | None = None,
        rejected: list[dict[str, Any]] | None = None,
    ) -> None:
        payload = {
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "request_id": request_id,
            "issues": issues,
            "rejected_candidates": rejected or [],
            "selected_trajectory": trajectory,
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
