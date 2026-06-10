#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


AUBO_JOINT_NAMES = (
    "aubo_shoulder_joint",
    "aubo_upperArm_joint",
    "aubo_foreArm_joint",
    "aubo_wrist1_joint",
    "aubo_wrist2_joint",
    "aubo_wrist3_joint",
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
    return json.loads(handler.rfile.read(length).decode("utf-8"))


def _duration_sec(duration: Any) -> float:
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) * 1e-9


class RemoteMoveItPlanner(Node):
    def __init__(self, plan_service: str) -> None:
        super().__init__("arachne_remote_moveit_planner")
        self.plan_client = self.create_client(GetMotionPlan, plan_service)
        self.plan_service = plan_service
        self.started_at = time.time()

    def health(self) -> dict[str, Any]:
        return {
            "ok": True,
            "service": "arachne_remote_moveit_planner",
            "uptime_sec": round(time.time() - self.started_at, 3),
            "moveit_plan_service": self.plan_service,
            "moveit_ready": self.plan_client.service_is_ready(),
            "capabilities": {
                "remote_moveit": True,
                "ompl": True,
                "joint_waypoint_time_parameterization": True,
            },
        }

    def plan(self, request: dict[str, Any]) -> dict[str, Any]:
        request_id = str(request.get("request_id") or uuid.uuid4().hex)
        targets = request.get("moveit_targets")
        if not isinstance(targets, list) or not targets:
            return {
                "request_id": request_id,
                "ok": False,
                "status": "missing_moveit_targets",
                "message": "Provide moveit_targets with tool0_xyz_aubo and tool0_quat_aubo.",
                "trajectory": None,
            }
        current = self._joint_vector(request.get("current_joints_rad"), "current_joints_rad")
        moveit_options = request.get("moveit") if isinstance(request.get("moveit"), dict) else {}
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            return {
                "request_id": request_id,
                "ok": False,
                "status": "moveit_service_unavailable",
                "message": f"Waiting for {self.plan_service}",
                "trajectory": None,
            }

        frames: list[dict[str, Any]] = [
            {
                "time_from_start": 0.0,
                "positions": current,
                "velocities": [0.0] * 6,
                "accelerations": [0.0] * 6,
            }
        ]
        total_time = 0.0
        q_start = list(current)
        segment_reports = []
        planner_id = str(moveit_options.get("planner_id") or "RRTConnectkConfigDefault")

        for index, target in enumerate(targets):
            if not isinstance(target, dict):
                return self._failure(request_id, f"target {index} is not an object", frames)
            result = self._call_moveit(q_start, target, moveit_options, planner_id)
            if not result["ok"]:
                result.update({"request_id": request_id, "trajectory": self._trajectory(frames, segment_reports)})
                return result
            points = result["points"]
            if not points:
                return self._failure(request_id, f"target {index} returned no points", frames)
            for point_index, point in enumerate(points[1:], start=1):
                total_time += max(point["dt"], 0.02)
                frames.append(
                    {
                        "time_from_start": round(total_time, 4),
                        "positions": point["positions"],
                        "velocities": point["velocities"],
                        "accelerations": point["accelerations"],
                    }
                )
            q_start = frames[-1]["positions"]
            segment_reports.append(
                {
                    "index": index,
                    "name": str(target.get("name") or f"target_{index}"),
                    "planner": planner_id,
                    "points": len(points),
                    "duration": round(sum(point["dt"] for point in points[1:]), 4),
                }
            )
        if frames:
            frames[-1]["velocities"] = [0.0] * 6
        return {
            "request_id": request_id,
            "ok": True,
            "status": "remote_moveit_ompl_plan",
            "message": "Remote MoveIt OMPL planning succeeded.",
            "selected_candidate": "remote_moveit",
            "constraint_issues": [],
            "rejected_candidates": [],
            "trajectory": self._trajectory(frames, segment_reports),
        }

    def _call_moveit(
        self,
        q_start: list[float],
        target: dict[str, Any],
        options: dict[str, Any],
        planner_id: str,
    ) -> dict[str, Any]:
        xyz = self._float_vector(target.get("tool0_xyz_aubo"), 3, "tool0_xyz_aubo")
        quat = self._float_vector(target.get("tool0_quat_aubo"), 4, "tool0_quat_aubo")
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name = "aubo_arm"
        motion.pipeline_id = "ompl"
        motion.planner_id = planner_id
        motion.num_planning_attempts = max(int(options.get("planning_attempts", 1)), 1)
        motion.allowed_planning_time = max(float(options.get("planning_time", 1.0)), 0.2)
        motion.max_velocity_scaling_factor = min(max(float(options.get("velocity_scale", 0.5)), 0.01), 1.0)
        motion.max_acceleration_scaling_factor = min(max(float(options.get("accel_scale", 0.8)), 0.01), 1.0)
        motion.start_state.joint_state.name = list(AUBO_JOINT_NAMES)
        motion.start_state.joint_state.position = [float(v) for v in q_start]
        motion.start_state.is_diff = False
        motion.goal_constraints = [self._pose_constraint(target, xyz, quat)]

        future = self.plan_client.call_async(req)
        timeout = motion.allowed_planning_time + max(float(options.get("service_timeout_padding", 0.5)), 0.0)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {"ok": False, "status": "moveit_timeout", "message": f"{target.get('name', '')} timeout"}
        response = future.result()
        if response is None:
            return {"ok": False, "status": "moveit_empty_response", "message": "empty MoveIt response"}
        motion_response = getattr(response, "motion_plan_response", response)
        error_code = getattr(motion_response, "error_code", None)
        error_value = int(getattr(error_code, "val", 99999))
        if error_value != 1:
            return {
                "ok": False,
                "status": "moveit_failed",
                "message": f"{target.get('name', '')} error_code={error_value}",
            }
        trajectory = motion_response.trajectory.joint_trajectory
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}
        if not all(name in name_to_index for name in AUBO_JOINT_NAMES):
            return {"ok": False, "status": "moveit_bad_trajectory", "message": "missing Aubo joints"}
        points = []
        previous_t = 0.0
        for point in trajectory.points:
            t = _duration_sec(point.time_from_start)
            points.append(
                {
                    "dt": max(t - previous_t, 0.0),
                    "positions": [float(point.positions[name_to_index[name]]) for name in AUBO_JOINT_NAMES],
                    "velocities": self._point_values(point.velocities, name_to_index),
                    "accelerations": self._point_values(point.accelerations, name_to_index),
                }
            )
            previous_t = t
        return {"ok": True, "points": points}

    def _pose_constraint(self, target: dict[str, Any], xyz: list[float], quat: list[float]) -> Constraints:
        tolerance_pos = max(float(target.get("position_tolerance", 0.015)), 0.002)
        tolerance_ori = max(float(target.get("orientation_tolerance", 0.35)), 0.01)
        constraints = Constraints()
        constraints.name = str(target.get("name") or "remote_pose_goal")

        pose = PoseStamped()
        pose.header.frame_id = "aubo_base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [tolerance_pos]
        pos = PositionConstraint()
        pos.header.frame_id = "aubo_base_link"
        pos.link_name = "tool0"
        pos.constraint_region.primitives = [primitive]
        pos.constraint_region.primitive_poses = [pose.pose]
        pos.weight = 1.0
        constraints.position_constraints = [pos]

        ori = OrientationConstraint()
        ori.header.frame_id = "aubo_base_link"
        ori.link_name = "tool0"
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = tolerance_ori
        ori.absolute_y_axis_tolerance = tolerance_ori
        ori.absolute_z_axis_tolerance = tolerance_ori
        ori.weight = 1.0
        constraints.orientation_constraints = [ori]
        return constraints

    def _trajectory(self, frames: list[dict[str, Any]], segments: list[dict[str, Any]]) -> dict[str, Any]:
        duration = float(frames[-1]["time_from_start"]) if frames else 0.0
        return {
            "joint_names": [
                "shoulder_joint",
                "upperArm_joint",
                "foreArm_joint",
                "wrist1_joint",
                "wrist2_joint",
                "wrist3_joint",
            ],
            "frames": frames,
            "segments": segments,
            "duration_sec": round(duration, 4),
            "audit": {"raw_waypoints": len(frames), "remote_moveit": True},
        }

    def _failure(self, request_id: str, message: str, frames: list[dict[str, Any]]) -> dict[str, Any]:
        return {
            "request_id": request_id,
            "ok": False,
            "status": "remote_moveit_failed",
            "message": message,
            "trajectory": self._trajectory(frames, []),
        }

    def _point_values(self, values: Any, name_to_index: dict[str, int]) -> list[float]:
        if len(values) < len(name_to_index):
            return [0.0] * 6
        return [float(values[name_to_index[name]]) for name in AUBO_JOINT_NAMES]

    def _joint_vector(self, value: Any, label: str) -> list[float]:
        return self._float_vector(value, 6, label)

    def _float_vector(self, value: Any, length: int, label: str) -> list[float]:
        if not isinstance(value, list) or len(value) != length:
            raise ValueError(f"{label} must be a {length}-element list")
        return [float(v) for v in value]


def make_handler(planner: RemoteMoveItPlanner):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ArachneRemoteMoveItPlanner/0.1"

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"{self.address_string()} - {fmt % args}", flush=True)

        def do_GET(self) -> None:
            if self.path.rstrip("/") == "/health":
                _json_response(self, 200, planner.health())
            else:
                _json_response(self, 404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:
            try:
                payload = _read_json(self)
                result = planner.plan(payload)
            except Exception as exc:
                _json_response(self, 400, {"ok": False, "error": str(exc)})
                return
            _json_response(self, 200, result)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Arachne remote MoveIt HTTP planner.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--moveit-plan-service", default="/plan_kinematic_path")
    args = parser.parse_args()

    rclpy.init()
    planner = RemoteMoveItPlanner(args.moveit_plan_service)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(planner)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), make_handler(planner))
        print(f"Arachne remote MoveIt planner listening on http://{args.host}:{args.port}", flush=True)
        httpd.serve_forever()
    finally:
        executor.shutdown()
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
