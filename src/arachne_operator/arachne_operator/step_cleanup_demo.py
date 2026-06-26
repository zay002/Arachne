from __future__ import annotations

import argparse
import json
import os
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from arachne_operator.aubo_move_joint_client import AuboMoveJointClient
from arachne_operator.repo_paths import root_dir


TERMINAL_STATES = {"succeeded", "failed", "canceled"}
STARTABLE_STATES = ("idle", *TERMINAL_STATES)
DEFAULT_SEARCH_JOINTS = "-1.611779,-0.457910,1.071527,-0.044520,1.575231,0.771459"


@dataclass
class Candidate:
    class_name: str
    confidence: float
    received_at: float
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class Snapshot:
    state: str
    message: str
    started_at: str
    finished_at: str
    attempts: int
    grasps: int
    progress_m: float
    latest_candidate: dict[str, Any]


class StepCleanupDemo(Node):
    """Stop-look-step-look cleanup demo.

    It deliberately reuses grasp_task_server for perception and grasp execution.
    This node only decides when to nudge the base forward and when to trigger a
    grasp.
    """

    def __init__(self) -> None:
        super().__init__("arachne_step_cleanup_demo")
        self.declare_parameter("detection_topic", "/arachne/perception/taco_instances")
        self.declare_parameter("restart_search_topic", "/arachne/grasp_preview/restart_search")
        self.declare_parameter(
            "real_search_scan_control_topic", "/arachne/grasp_preview/real_search_scan"
        )
        self.declare_parameter("base_command_topic", "/arachne/grasp_task/base_command")
        self.declare_parameter("base_state_topic", "/arachne/grasp_task/base_state")
        self.declare_parameter("base_status_service", "/arachne/grasp_task/base_status")
        self.declare_parameter("base_stop_service", "/arachne/grasp_task/base_stop")
        self.declare_parameter("grasp_start_service", "/arachne/grasp_task/start")
        self.declare_parameter("grasp_stop_service", "/arachne/grasp_task/stop")
        self.declare_parameter("grasp_preflight_service", "/arachne/grasp_task/preflight")
        self.declare_parameter("confidence", 0.08)
        self.declare_parameter("observe_timeout_sec", 8.0)
        self.declare_parameter("candidate_fresh_sec", 2.0)
        self.declare_parameter("grasp_min_base_x_m", 0.30)
        self.declare_parameter("grasp_max_base_x_m", 0.90)
        self.declare_parameter("target_base_x_m", 0.72)
        self.declare_parameter("approach_step_m", 0.12)
        self.declare_parameter("max_approach_steps", 5)
        self.declare_parameter("max_grasps", 1)
        self.declare_parameter("grasp_timeout_sec", 180.0)
        self.declare_parameter("base_step_timeout_sec", 20.0)
        self.declare_parameter("return_home_on_finish", True)
        self.declare_parameter("move_to_search_pose_before_start", True)
        self.declare_parameter("required_search_joints", DEFAULT_SEARCH_JOINTS)
        self.declare_parameter("search_pose_move_speed_rad_sec", 0.25)
        self.declare_parameter("search_pose_move_accel_rad_sec2", 0.45)
        self.declare_parameter("search_pose_move_timeout_sec", 20.0)
        self.declare_parameter("search_pose_goal_tolerance_rad", 0.08)
        self.declare_parameter("aubo_move_joint_action_name", "/arachne/aubo/move_joint")
        self.declare_parameter("recover_lost_target", True)
        self.declare_parameter("log_root", "log/step_cleanup_demo")

        self.workspace_root = root_dir()
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.state = "idle"
        self.message = "ready"
        self.started_at = ""
        self.finished_at = ""
        self.attempts = 0
        self.grasps = 0
        self.progress_m = 0.0
        self.last_approach_step_m = 0.0
        self.latest_candidate: Candidate | None = None
        self.base_state: dict[str, Any] = {}
        self.grasp_state: dict[str, Any] = {}
        self.run_dir: Path | None = None
        self.task_id = ""

        self.state_pub = self.create_publisher(String, "/arachne/step_cleanup/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/step_cleanup/event", 10)
        self.restart_search_pub = self.create_publisher(
            Empty, str(self.get_parameter("restart_search_topic").value), 10
        )
        self.scan_pub = self.create_publisher(
            Bool, str(self.get_parameter("real_search_scan_control_topic").value), 10
        )
        self.base_command_pub = self.create_publisher(
            String, str(self.get_parameter("base_command_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detection_topic").value),
            self._detection_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_state_topic").value),
            self._base_state_cb,
            10,
        )
        self.create_subscription(String, "/arachne/grasp_task/state", self._grasp_state_cb, 10)

        self.grasp_start_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_start_service").value)
        )
        self.grasp_stop_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_stop_service").value)
        )
        self.grasp_preflight_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_preflight_service").value)
        )
        self.base_status_client = self.create_client(
            Trigger, str(self.get_parameter("base_status_service").value)
        )
        self.base_stop_client = self.create_client(
            Trigger, str(self.get_parameter("base_stop_service").value)
        )
        self.aubo_move_joint = AuboMoveJointClient(
            self, str(self.get_parameter("aubo_move_joint_action_name").value)
        )

        self.create_service(Trigger, "/arachne/step_cleanup/start", self._start_cb)
        self.create_service(Trigger, "/arachne/step_cleanup/stop", self._stop_cb)
        self.create_service(Trigger, "/arachne/step_cleanup/status", self._status_cb)
        self.create_service(Trigger, "/arachne/step_cleanup/preflight", self._preflight_cb)
        self.create_timer(0.5, self._publish_state)

    def _start_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy or self.state not in STARTABLE_STATES:
                response.success = False
                response.message = self._snapshot_json()
                return response
            self._prepare_run_log_locked()
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _stop_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.cancel_event.set()
        self._set_scan(False)
        self._call_trigger(self.base_stop_client, 1.0)
        self._call_trigger(self.grasp_stop_client, 1.0)
        self._finish("canceled", "step cleanup stopping")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._call_trigger(self.grasp_preflight_client, 30.0)
        response.success = ok
        response.message = message
        return response

    def _run(self) -> None:
        with self.lock:
            self.started_at = datetime.now().isoformat(timespec="seconds")
            self.finished_at = ""
            self.attempts = 0
            self.grasps = 0
            self.progress_m = 0.0
            self.last_approach_step_m = 0.0
            self.latest_candidate = None
        max_steps = max(int(self.get_parameter("max_approach_steps").value), 0)
        max_grasps = max(int(self.get_parameter("max_grasps").value), 1)
        ok, message = self._call_trigger(self.grasp_preflight_client, 30.0)
        if not ok:
            self._finish("failed", f"preflight failed: {message}")
            return
        ok, message = self._move_to_search_pose_if_needed()
        if not ok:
            self._finish("failed", message)
            return

        while rclpy.ok() and not self.cancel_event.is_set():
            if self.grasps >= max_grasps:
                self._finish("succeeded", f"step cleanup complete: grasps={self.grasps}")
                return
            candidate = self._observe_once()
            if candidate is None:
                if self._recover_lost_target_once():
                    candidate = self._observe_once()
            if candidate is None:
                self._finish("failed", "no trash candidate")
                return
            x = self._candidate_base_x(candidate)
            if x is None:
                self._finish("failed", "trash candidate has no base_x")
                return
            min_x = float(self.get_parameter("grasp_min_base_x_m").value)
            max_x = float(self.get_parameter("grasp_max_base_x_m").value)
            if min_x <= x <= max_x:
                self._event(
                    "candidate_decision",
                    {"base_x_m": x, "min_x_m": min_x, "max_x_m": max_x, "decision": "grasp"},
                )
                if not self._run_grasp(candidate):
                    return
                continue
            if x < min_x:
                self._event(
                    "candidate_decision",
                    {"base_x_m": x, "min_x_m": min_x, "max_x_m": max_x, "decision": "too_close"},
                )
                self._finish("failed", f"trash too close: base_x={x:.2f}m")
                return
            if self.attempts >= max_steps:
                self._event(
                    "candidate_decision",
                    {
                        "base_x_m": x,
                        "min_x_m": min_x,
                        "max_x_m": max_x,
                        "decision": "too_far_exhausted",
                        "attempts": self.attempts,
                    },
                )
                self._finish("failed", f"trash still too far after {self.attempts} steps")
                return
            target = float(self.get_parameter("target_base_x_m").value)
            step = min(
                float(self.get_parameter("approach_step_m").value),
                max(0.02, x - target),
            )
            self._event(
                "candidate_decision",
                {
                    "base_x_m": x,
                    "min_x_m": min_x,
                    "max_x_m": max_x,
                    "target_x_m": target,
                    "step_m": step,
                    "decision": "approach",
                },
            )
            if not self._drive_forward(step):
                return
            self.last_approach_step_m = step

        if self.cancel_event.is_set():
            self._finish("canceled", "step cleanup canceled")

    def _observe_once(self) -> Candidate | None:
        with self.lock:
            self.latest_candidate = None
        self._set_state("observing", "detect trash from still camera")
        self._set_scan(True)
        self.restart_search_pub.publish(Empty())
        deadline = time.monotonic() + max(float(self.get_parameter("observe_timeout_sec").value), 0.1)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() < deadline:
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._set_scan(False)
                self._event("candidate", asdict(candidate))
                return candidate
            time.sleep(0.05)
        self._set_scan(False)
        return None

    def _drive_forward(
        self, distance: float, *, finish_on_error: bool = True, count_attempt: bool = True
    ) -> bool:
        if count_attempt and distance > 0.0:
            self.attempts += 1
        direction = "forward" if distance >= 0.0 else "back"
        self._set_state("approaching", f"move base {direction} {abs(distance):.2f}m")
        self._set_scan(False)
        payload = {
            "command": "drive_relative",
            "distance_m": float(distance),
            "request_id": f"step-cleanup-{uuid.uuid4().hex[:8]}",
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.base_command_pub.publish(msg)
        started = time.monotonic()
        timeout = max(float(self.get_parameter("base_step_timeout_sec").value), 1.0)
        command_seen = False
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() - started < timeout:
            snapshot = self._query_base_status()
            active = snapshot.get("active_command", {})
            if isinstance(active, dict) and active.get("request_id") == payload["request_id"]:
                command_seen = True
            if command_seen and self._base_done(snapshot, payload):
                state = str(snapshot.get("state", "")).lower()
                if state == "failed":
                    if finish_on_error:
                        self._finish("failed", str(snapshot.get("message", "base failed")))
                    else:
                        self._event("base_step_failed", {"message": str(snapshot.get("message", "base failed"))})
                    return False
                self.progress_m += distance
                self._event("base_step_done", {"distance_m": distance})
                return True
            time.sleep(0.05)
        self._call_trigger(self.base_stop_client, 1.0)
        if self.cancel_event.is_set():
            return False
        if finish_on_error:
            self._finish("failed", "base approach timeout")
        else:
            self._event("base_step_failed", {"message": "base approach timeout"})
        return False

    def _recover_lost_target_once(self) -> bool:
        if not bool(self.get_parameter("recover_lost_target").value):
            return False
        with self.lock:
            back = min(abs(self.last_approach_step_m), abs(self.progress_m))
        if back <= 1e-3:
            return False
        self._event("lost_target_recovery", {"backtrack_m": back})
        ok = self._drive_forward(-back, finish_on_error=False, count_attempt=False)
        if ok:
            with self.lock:
                self.last_approach_step_m = 0.0
        return ok

    def _move_to_search_pose_if_needed(self) -> tuple[bool, str]:
        if not bool(self.get_parameter("move_to_search_pose_before_start").value):
            return True, "search pose move disabled"
        target = self._parse_float_list(str(self.get_parameter("required_search_joints").value), 6)
        if target is None:
            return False, "required_search_joints must contain 6 floats"
        self._set_state("preparing", "move arm to step cleanup search pose")
        ok, message, final_error = self.aubo_move_joint.move_joint(
            target,
            label="step_cleanup_search_pose",
            speed_rad_sec=float(self.get_parameter("search_pose_move_speed_rad_sec").value),
            accel_rad_sec2=float(self.get_parameter("search_pose_move_accel_rad_sec2").value),
            goal_tolerance_rad=float(self.get_parameter("search_pose_goal_tolerance_rad").value),
            timeout_sec=float(self.get_parameter("search_pose_move_timeout_sec").value),
        )
        self._event(
            "search_pose_move",
            {"ok": ok, "message": message, "final_error_rad": final_error, "target_joints": target},
        )
        return ok, message

    def _run_grasp(self, candidate: Candidate) -> bool:
        self._set_scan(False)
        self._set_state("grasp", f"run grasp for {candidate.class_name}")
        ok, message = self._call_trigger(self.grasp_start_client, 5.0)
        if not ok:
            self._finish("failed", f"grasp start failed: {message}")
            return False
        deadline = time.monotonic() + max(float(self.get_parameter("grasp_timeout_sec").value), 5.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() < deadline:
            with self.lock:
                state = str(self.grasp_state.get("state", "")).lower()
                text = str(self.grasp_state.get("message", ""))
            if state == "succeeded":
                self.grasps += 1
                self._event("grasp_complete", {"grasps": self.grasps})
                return True
            if state in ("failed", "canceled"):
                self._finish("failed", f"grasp {state}: {text}")
                return False
            time.sleep(0.1)
        self._call_trigger(self.grasp_stop_client, 1.0)
        self._finish("failed", "grasp timeout")
        return False

    def _detection_cb(self, msg: String) -> None:
        for candidate in self._parse_candidates(msg.data):
            if candidate.confidence < float(self.get_parameter("confidence").value):
                continue
            if self._candidate_base_x(candidate) is None:
                continue
            with self.lock:
                current = self.latest_candidate
                if current is None or candidate.confidence >= current.confidence:
                    self.latest_candidate = candidate

    def _base_state_cb(self, msg: String) -> None:
        with self.lock:
            self.base_state = self._parse_json_object(msg.data)

    def _grasp_state_cb(self, msg: String) -> None:
        with self.lock:
            self.grasp_state = self._parse_json_object(msg.data)

    def _fresh_candidate(self) -> Candidate | None:
        with self.lock:
            candidate = self.latest_candidate
        if candidate is None:
            return None
        max_age = max(float(self.get_parameter("candidate_fresh_sec").value), 0.1)
        return candidate if time.monotonic() - candidate.received_at <= max_age else None

    def _parse_candidates(self, text: str) -> list[Candidate]:
        payload = self._parse_json_object(text)
        if not payload:
            return []
        items = payload.get("instances") or payload.get("detections") or [payload]
        if not isinstance(items, list):
            return []
        candidates = []
        for raw in items:
            if not isinstance(raw, dict):
                continue
            class_name = str(
                raw.get("taco_class")
                or raw.get("class_name")
                or raw.get("class")
                or raw.get("label")
                or "trash"
            )
            confidence = float(raw.get("confidence", raw.get("score", 0.0)))
            candidates.append(Candidate(class_name, confidence, time.monotonic(), dict(raw)))
        return candidates

    def _candidate_base_x(self, candidate: Candidate) -> float | None:
        raw = candidate.raw
        xyz = raw.get("base_grasp_xyz")
        if not isinstance(xyz, list) or len(xyz) < 1:
            xyz = self._planning_waypoint_xyz(raw, "grasp")
        try:
            return float(xyz[0]) if isinstance(xyz, list) and xyz else None
        except (TypeError, ValueError):
            return None

    def _planning_waypoint_xyz(self, raw: dict[str, Any], name: str) -> list[float]:
        for key in ("planning_key_waypoints", "waypoints_base"):
            items = raw.get(key)
            if not isinstance(items, list):
                continue
            for item in items:
                if (
                    isinstance(item, dict)
                    and str(item.get("name", "")).strip() == name
                    and isinstance(item.get("xyz"), list)
                ):
                    return item["xyz"]
        return []

    def _query_base_status(self) -> dict[str, Any]:
        ok, message = self._call_trigger(self.base_status_client, 0.2)
        if ok:
            snapshot = self._parse_json_object(message)
            if snapshot:
                with self.lock:
                    self.base_state = snapshot
                return snapshot
        with self.lock:
            return dict(self.base_state)

    def _base_done(self, snapshot: dict[str, Any], payload: dict[str, Any]) -> bool:
        active = snapshot.get("active_command", {})
        if isinstance(active, dict) and payload.get("request_id") != active.get("request_id"):
            return False
        state = str(snapshot.get("state", "")).lower()
        busy = bool(snapshot.get("worker_busy", False))
        return state in ("succeeded", "failed", "canceled", "idle") and not busy

    def _call_trigger(self, client: Any, timeout: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"service unavailable: {getattr(client, 'srv_name', 'trigger')}"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, f"service timeout: {getattr(client, 'srv_name', 'trigger')}"
        response = future.result()
        if response is None:
            return False, "empty response"
        return bool(response.success), str(response.message)

    def _set_scan(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.scan_pub.publish(msg)

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _finish(self, state: str, message: str) -> None:
        self._set_scan(False)
        self._return_home_if_needed()
        with self.lock:
            self.state = state
            self.message = message
            self.finished_at = datetime.now().isoformat(timespec="seconds")
            run_dir = self.run_dir
        self._event("state", {"state": state, "message": message})
        self._publish_state()
        if run_dir is not None:
            with (run_dir / "summary.json").open("w", encoding="utf-8") as handle:
                json.dump(asdict(self._snapshot()), handle, ensure_ascii=False, indent=2)

    def _return_home_if_needed(self) -> None:
        if not bool(self.get_parameter("return_home_on_finish").value):
            return
        with self.lock:
            progress = float(self.progress_m)
        if abs(progress) <= 1e-3 or self.cancel_event.is_set():
            return
        self._set_state("returning", f"return to start {-progress:.2f}m")
        if self._drive_forward(-progress, finish_on_error=False):
            with self.lock:
                self.progress_m = 0.0
            self._event("return_home_done", {"distance_m": -progress})

    def _event(self, kind: str, data: dict[str, Any]) -> None:
        event = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "task_id": self.task_id,
            **data,
        }
        text = json.dumps(event, ensure_ascii=False, sort_keys=True)
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        if self.run_dir is not None:
            try:
                with (self.run_dir / "events.jsonl").open("a", encoding="utf-8") as handle:
                    handle.write(text + "\n")
            except OSError as exc:
                self.get_logger().warning(f"step cleanup log write failed: {exc}")
        if kind == "state":
            self.get_logger().info(f"{data.get('state')}: {data.get('message')}")

    def _prepare_run_log_locked(self) -> None:
        self.task_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        root = Path(str(self.get_parameter("log_root").value)).expanduser()
        if not root.is_absolute():
            root = self.workspace_root / root
        self.run_dir = root / self.task_id
        self.run_dir.mkdir(parents=True, exist_ok=True)
        latest = root / "latest"
        try:
            if latest.exists() or latest.is_symlink():
                latest.unlink()
            os.symlink(self.run_dir, latest)
        except OSError:
            pass

    def _snapshot(self) -> Snapshot:
        with self.lock:
            latest = asdict(self.latest_candidate) if self.latest_candidate else {}
            return Snapshot(
                state=self.state,
                message=self.message,
                started_at=self.started_at,
                finished_at=self.finished_at,
                attempts=self.attempts,
                grasps=self.grasps,
                progress_m=self.progress_m,
                latest_candidate=latest,
            )

    def _snapshot_json(self) -> str:
        return json.dumps(asdict(self._snapshot()), ensure_ascii=False, sort_keys=True)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._snapshot_json()
        self.state_pub.publish(msg)

    def _parse_json_object(self, text: str) -> dict[str, Any]:
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return {}
        return payload if isinstance(payload, dict) else {}

    def _parse_float_list(self, text: str, expected: int) -> list[float] | None:
        try:
            values = [
                float(item.strip())
                for item in text.replace(";", ",").split(",")
                if item.strip()
            ]
        except ValueError:
            return None
        return values if len(values) == expected else None


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Arachne step cleanup demo")
    parser.add_argument("--dry-run-check", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.dry_run_check:
        print("step_cleanup_demo dry-run check ok")
        return

    rclpy.init(args=ros_args)
    node = StepCleanupDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
