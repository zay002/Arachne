from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


TERMINAL_STATES = {"succeeded", "failed", "canceled"}
STARTABLE_STATES = ("idle", "paused", *TERMINAL_STATES)


@dataclass
class Candidate:
    class_name: str
    confidence: float
    message: str
    received_at: str
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class CleanupSnapshot:
    state: str
    message: str
    started_at: str
    finished_at: str
    direction: int
    progress_m: float
    cycle_count: int
    latest_candidate: dict[str, Any]
    active_candidate: dict[str, Any]
    recovery_attempts: int
    last_grasp_failure: str


class RoadCleanupTaskServer(Node):
    """Real-machine patrol wrapper for road-side trash cleanup.

    The node is intentionally thin: it owns the task-level patrol policy while
    the existing grasp_server owns YOLO-SEG, point-cloud extraction, planning,
    and real execution. The YOLO-SEG pipeline publishes lightweight detection
    events on /arachne/perception/taco_instances; when TACO-trained weights are
    installed, the same topic carries the new class labels without changing the
    task interface.
    """

    def __init__(self) -> None:
        super().__init__("arachne_road_cleanup_task_server")
        self.declare_parameter("detection_topic", "/arachne/perception/taco_instances")
        self.declare_parameter("base_command_topic", "/arachne/grasp_task/base_command")
        self.declare_parameter("base_state_topic", "/arachne/grasp_task/base_state")
        self.declare_parameter("grasp_start_service", "/arachne/grasp_task/start")
        self.declare_parameter("grasp_stop_service", "/arachne/grasp_task/stop")
        self.declare_parameter("grasp_status_service", "/arachne/grasp_task/status")
        self.declare_parameter("grasp_preflight_service", "/arachne/grasp_task/preflight")
        self.declare_parameter("base_stop_service", "/arachne/grasp_task/base_stop")
        self.declare_parameter("base_status_service", "/arachne/grasp_task/base_status")
        self.declare_parameter("restart_search_topic", "/arachne/grasp_preview/restart_search")
        self.declare_parameter(
            "real_search_scan_control_topic", "/arachne/grasp_preview/real_search_scan"
        )
        self.declare_parameter("patrol_pattern", "line")
        self.declare_parameter("patrol_distance_m", 1.5)
        self.declare_parameter("patrol_step_m", 1.5)
        self.declare_parameter("patrol_box_width_m", 1.0)
        self.declare_parameter("patrol_box_height_m", 1.2)
        self.declare_parameter("patrol_entry_m", 0.3)
        self.declare_parameter("patrol_base_speed_mps", 0.06)
        self.declare_parameter("max_round_trips", 2)
        self.declare_parameter("detection_confidence", 0.08)
        self.declare_parameter("detection_timeout_sec", 3.0)
        self.declare_parameter("initial_detection_wait_sec", 0.0)
        self.declare_parameter("require_3d_candidate", True)
        self.declare_parameter("candidate_min_base_x_m", 0.25)
        self.declare_parameter("candidate_max_base_x_m", 1.03)
        self.declare_parameter("candidate_max_abs_base_y_m", 0.60)
        self.declare_parameter("candidate_min_base_z_m", -0.18)
        self.declare_parameter("candidate_max_reach_m", 1.03)
        self.declare_parameter("candidate_max_depth_m", 0.85)
        self.declare_parameter("patrol_turn_scale", 1.0)
        self.declare_parameter("base_step_timeout_sec", 8.0)
        self.declare_parameter("base_stop_wait_sec", 3.0)
        self.declare_parameter("grasp_timeout_sec", 90.0)
        self.declare_parameter("reach_recovery_enabled", True)
        self.declare_parameter("reach_recovery_max_attempts", 3)
        self.declare_parameter("reach_recovery_step_m", 0.10)
        self.declare_parameter("reach_recovery_wait_detection_sec", 3.0)
        self.declare_parameter("reach_recovery_continue_on_exhausted", True)
        self.declare_parameter("continue_on_grasp_failure", True)
        self.declare_parameter("auto_return_home_on_empty_route", True)
        self.declare_parameter("loop", True)

        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.state = "idle"
        self.message = "ready"
        self.started_at = ""
        self.finished_at = ""
        self.direction = 1
        self.progress_m = 0.0
        self.cycle_count = 0
        self.latest_candidate: Candidate | None = None
        self.active_candidate: Candidate | None = None
        self.recovery_attempts = 0
        self.last_grasp_failure = ""
        self.base_state: dict[str, Any] = {}
        self.grasp_state: dict[str, Any] = {}
        self.patrol_waypoints: list[tuple[float, float]] = []
        self.patrol_current_index = 0
        self.patrol_target_index = 1
        self.patrol_loop_start_index = 0
        self.patrol_heading_rad = 0.0
        self.completed_base_segments: list[dict[str, Any]] = []
        self.successful_grasps = 0
        self.base_segment_interrupted_by_grasp = False
        self.real_search_scan_enabled = False

        self.state_pub = self.create_publisher(String, "/arachne/road_cleanup/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/road_cleanup/event", 10)
        self.base_command_pub = self.create_publisher(
            String, str(self.get_parameter("base_command_topic").value), 10
        )
        self.restart_search_pub = self.create_publisher(
            Empty, str(self.get_parameter("restart_search_topic").value), 10
        )
        self.real_search_scan_pub = self.create_publisher(
            Bool, str(self.get_parameter("real_search_scan_control_topic").value), 10
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

        self.start_srv = self.create_service(Trigger, "/arachne/road_cleanup/start", self._start_cb)
        self.pause_srv = self.create_service(Trigger, "/arachne/road_cleanup/pause", self._pause_cb)
        self.return_home_srv = self.create_service(
            Trigger, "/arachne/road_cleanup/return_home", self._return_home_cb
        )
        self.stop_srv = self.create_service(Trigger, "/arachne/road_cleanup/stop", self._stop_cb)
        self.status_srv = self.create_service(Trigger, "/arachne/road_cleanup/status", self._status_cb)
        self.preflight_srv = self.create_service(
            Trigger, "/arachne/road_cleanup/preflight", self._preflight_cb
        )

        self.grasp_start_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_start_service").value)
        )
        self.grasp_stop_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_stop_service").value)
        )
        self.grasp_status_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_status_service").value)
        )
        self.grasp_preflight_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_preflight_service").value)
        )
        self.base_stop_client = self.create_client(
            Trigger, str(self.get_parameter("base_stop_service").value)
        )
        self.base_status_client = self.create_client(
            Trigger, str(self.get_parameter("base_status_service").value)
        )

        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.5, self._publish_real_search_scan)
        self._event(
            "server_ready",
            {
                "detection_topic": str(self.get_parameter("detection_topic").value),
                "restart_search_topic": str(self.get_parameter("restart_search_topic").value),
                "perception_source": "grasp_server_yolo_seg",
            },
        )

    def _start_cb(self, _request, response):
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy or self.state not in STARTABLE_STATES:
                response.success = False
                response.message = self._snapshot_json()
                return response
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run_task, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _pause_cb(self, _request, response):
        self.cancel_event.set()
        self._set_real_search_scan(False)
        self._call_trigger_background(self.base_stop_client, 1.0, "base_stop")
        self._call_trigger_background(self.grasp_stop_client, 1.0, "grasp_stop")
        self._set_state("paused", "road cleanup paused")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _return_home_cb(self, _request, response):
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy:
                self.cancel_event.set()
                self._set_real_search_scan(False)
                self._call_trigger_background(self.base_stop_client, 1.0, "base_stop")
                self._call_trigger_background(self.grasp_stop_client, 1.0, "grasp_stop")
                response.success = False
                response.message = "road cleanup is busy; pause first, then return home"
                return response
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run_return_home, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _stop_cb(self, _request, response):
        self.cancel_event.set()
        self._set_real_search_scan(False)
        self._call_trigger_background(self.base_stop_client, 1.0, "base_stop")
        self._call_trigger_background(self.grasp_stop_client, 1.0, "grasp_stop")
        self._set_state("canceled", "road cleanup stopping")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request, response):
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request, response):
        ok, message = self._call_trigger(self.grasp_preflight_client, 30.0)
        response.success = ok
        response.message = message
        return response

    def _detection_cb(self, msg: String) -> None:
        threshold = float(self.get_parameter("detection_confidence").value)
        candidates = [item for item in self._parse_candidates(msg.data) if item.confidence >= threshold]
        if not candidates:
            return
        reachable: list[Candidate] = []
        for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
            ok, reason = self._candidate_reachable(candidate)
            if ok:
                reachable.append(candidate)
            else:
                self._event("candidate_ignored", {**asdict(candidate), "reason": reason})
        if not reachable:
            return
        candidate = reachable[0]
        with self.lock:
            self.latest_candidate = candidate
        self._event("candidate", asdict(candidate))

    def _base_state_cb(self, msg: String) -> None:
        with self.lock:
            self.base_state = self._parse_json_object(msg.data)

    def _grasp_state_cb(self, msg: String) -> None:
        with self.lock:
            self.grasp_state = self._parse_json_object(msg.data)

    def _run_task(self) -> None:
        with self.lock:
            self.started_at = datetime.now().isoformat(timespec="seconds")
            self.finished_at = ""
            self.direction = 1
            self.progress_m = 0.0
            self.cycle_count = 0
            self.completed_base_segments = []
            self.patrol_waypoints = self._make_patrol_waypoints()
            self.patrol_current_index = 0
            self.patrol_target_index = 1 if len(self.patrol_waypoints) > 1 else 0
            self.patrol_heading_rad = 0.0
            self.active_candidate = None
            self.recovery_attempts = 0
            self.last_grasp_failure = ""
            self.successful_grasps = 0
        self._set_state("preflight", "checking camera, base and grasp primitive")
        ok, message = self._call_trigger(self.grasp_preflight_client, 30.0)
        if not ok:
            self._finish("failed", f"preflight failed: {message}")
            return

        self._set_real_search_scan(True)
        self._restart_visual_search("road cleanup start")
        self._set_state("searching", "initial visual search before patrol")
        deadline = time.monotonic() + max(
            float(self.get_parameter("initial_detection_wait_sec").value), 0.0
        )
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() < deadline:
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._handle_candidate(candidate)
                if self._is_terminal() or self.cancel_event.is_set():
                    break
                self._set_state("patrol", "resume patrol after grasp")
                self._restart_visual_search("resume patrol after initial grasp")
                break
            time.sleep(0.05)
        if self._is_terminal() or self.cancel_event.is_set():
            return
        self._set_state("patrol", "patrol route while grasp_server YOLO-SEG watches")
        while rclpy.ok() and not self.cancel_event.is_set():
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._handle_candidate(candidate)
                if self._is_terminal():
                    break
                if self.cancel_event.is_set():
                    break
                self._set_state("patrol", "resume patrol after grasp")
                self._restart_visual_search("resume patrol after grasp")
                continue

            if not self._patrol_step():
                break

        if self.cancel_event.is_set() and self.state != "paused":
            self._finish("canceled", "road cleanup canceled")
        elif self.state not in TERMINAL_STATES:
            self._run_return_home()
            if self.state in TERMINAL_STATES:
                return
            self._finish("succeeded", "road cleanup complete")

    def _run_return_home(self) -> None:
        self._set_real_search_scan(False)
        self._set_state("returning", "returning to road cleanup start")
        with self.lock:
            completed = [dict(item) for item in self.completed_base_segments]
        inverse_segments = self._inverse_base_segments(completed)
        if not inverse_segments:
            self._finish("succeeded", "return home complete: no completed base legs")
            return
        payload = {
            "command": "replay_segments",
            "label": "road_cleanup_return_home",
            "segments": inverse_segments,
            "request_id": self._new_base_request_id("return"),
        }
        ok = self._execute_base_payload(
            payload,
            0.0,
            "return_home_done",
            monitor_candidates=False,
            record_completed=False,
        )
        if ok and not self.cancel_event.is_set():
            with self.lock:
                self.completed_base_segments = []
                self.progress_m = 0.0
                self.patrol_current_index = 0
                self.patrol_target_index = 1
                self.patrol_heading_rad = 0.0
            self._finish("succeeded", "return home complete")

    def _patrol_step(self) -> bool:
        pattern = str(self.get_parameter("patrol_pattern").value).strip().lower()
        if pattern in ("box_entry", "rectangle_entry", "real_box", "sim_box"):
            return self._waypoint_patrol_step()
        return self._line_patrol_step()

    def _line_patrol_step(self) -> bool:
        distance_limit = abs(float(self.get_parameter("patrol_distance_m").value))
        step = abs(float(self.get_parameter("patrol_step_m").value))
        remaining = distance_limit - abs(self.progress_m)
        if remaining <= 1e-3:
            with self.lock:
                self.cycle_count += 1
            return False

        distance = min(step, remaining)
        self._set_state(
            "patrol",
            f"scan while moving forward step={distance:.2f}m",
        )
        payload = {
            "command": "drive_relative",
            "distance_m": distance,
            "speed_m_s": max(float(self.get_parameter("patrol_base_speed_mps").value), 0.0),
            "request_id": self._new_base_request_id("line"),
        }
        self._restart_visual_search("line patrol step start")
        return self._execute_base_payload(payload, distance, "base_step_done")

    def _waypoint_patrol_step(self) -> bool:
        if len(self.patrol_waypoints) < 2:
            self._finish("failed", "patrol route has fewer than 2 waypoints")
            return False
        if self.patrol_target_index >= len(self.patrol_waypoints):
            return False
        if self.patrol_target_index <= 0:
            self.patrol_target_index = 1

        self.patrol_current_index = min(
            max(self.patrol_current_index, 0), len(self.patrol_waypoints) - 1
        )
        start = self.patrol_waypoints[self.patrol_current_index]
        target = self.patrol_waypoints[self.patrol_target_index]
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance <= 1e-3:
            self._advance_patrol_index()
            return True

        desired_heading = math.atan2(dy, dx)
        turn = self._angle_diff(desired_heading, self.patrol_heading_rad)
        commanded_turn = turn * max(float(self.get_parameter("patrol_turn_scale").value), 0.1)
        speed = max(float(self.get_parameter("patrol_base_speed_mps").value), 0.0)
        segments: list[dict[str, Any]] = []
        if abs(turn) > math.radians(2.0):
            segments.append(
                {
                    "type": "angular",
                    "action": "left" if commanded_turn >= 0.0 else "right",
                    "angle_rad": abs(commanded_turn),
                    "signed_angle_rad": commanded_turn,
                }
            )
        segments.append(
            {
                "type": "linear",
                "action": "forward",
                "distance_m": distance,
                "signed_distance_m": distance,
                "linear_x": speed,
            }
        )
        self._set_state(
            "patrol",
            (
                f"sim box patrol leg {self.patrol_current_index + 1}->{self.patrol_target_index + 1} "
                f"distance={distance:.2f}m turn={math.degrees(commanded_turn):.1f}deg"
            ),
        )
        payload = {
            "command": "replay_segments",
            "label": "road_cleanup_sim_box",
            "segments": segments,
            "request_id": self._new_base_request_id("box"),
        }
        self._restart_visual_search("sim box patrol leg start")
        ok = self._execute_base_payload(payload, distance, "base_leg_done")
        interrupted = self.base_segment_interrupted_by_grasp
        if ok and not interrupted:
            with self.lock:
                self.patrol_heading_rad = desired_heading
            self._advance_patrol_index()
        return ok

    def _execute_base_payload(
        self,
        payload: dict[str, Any],
        progress_delta_m: float,
        event_kind: str,
        *,
        monitor_candidates: bool = True,
        record_completed: bool = True,
    ) -> bool:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.base_segment_interrupted_by_grasp = False
        self.base_command_pub.publish(msg)
        command_started = False
        started = time.monotonic()
        timeout = max(float(self.get_parameter("base_step_timeout_sec").value), 1.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() - started < timeout:
            if monitor_candidates:
                candidate = self._fresh_candidate()
                if candidate is not None:
                    self.base_segment_interrupted_by_grasp = True
                    self._call_trigger(self.base_stop_client, 1.0)
                    self._record_base_progress(payload, progress_delta_m)
                    self._handle_candidate(candidate)
                    if self._is_terminal():
                        return False
                    return True
            base_snapshot = self._query_base_status()
            if self._base_matches_command(base_snapshot, payload):
                state = str(base_snapshot.get("state", "")).lower()
                worker_busy = bool(base_snapshot.get("worker_busy", False))
                command_started = command_started or worker_busy or state == "running"
                if command_started and self._base_terminal(base_snapshot):
                    result = base_snapshot.get("latest_result", {})
                    progress = float(result.get("progress_m", progress_delta_m))
                    target = float(result.get("target_m", progress_delta_m))
                    if state == "failed":
                        self._finish("failed", str(base_snapshot.get("message", "base failed")))
                        return False
                    if state == "canceled":
                        return False
                    signed_progress = math.copysign(abs(progress), progress_delta_m)
                    with self.lock:
                        self.progress_m += signed_progress
                        if record_completed:
                            self.completed_base_segments.extend(
                                self._segments_from_base_payload(payload, signed_progress)
                            )
                    self._event(
                        event_kind,
                        {
                            "distance_m": progress_delta_m,
                            "state": state,
                            "progress_m": progress,
                            "target_m": target,
                        },
                    )
                    return True
            elif not command_started:
                msg = String()
                msg.data = json.dumps(payload, sort_keys=True)
                self.base_command_pub.publish(msg)
            elif self._base_terminal(base_snapshot):
                self._finish("failed", "base command ended without matching active command")
                return False
            if self._base_terminal() and command_started:
                with self.lock:
                    self.progress_m += progress_delta_m
                    if record_completed:
                        self.completed_base_segments.extend(
                            self._segments_from_base_payload(payload, progress_delta_m)
                        )
                return True
            time.sleep(0.05)
        self._call_trigger(self.base_stop_client, 1.0)
        if self.cancel_event.is_set():
            return False
        self._finish("failed", "base patrol step timeout")
        return False

    def _record_base_progress(self, payload: dict[str, Any], fallback_delta_m: float) -> None:
        result = self._query_base_status().get("latest_result", {})
        if not isinstance(result, dict):
            return
        try:
            progress = float(result.get("progress_m", 0.0))
        except (TypeError, ValueError):
            return
        if abs(progress) <= 1e-3:
            return
        signed_progress = math.copysign(abs(progress), fallback_delta_m)
        with self.lock:
            self.progress_m += signed_progress
            self.completed_base_segments.extend(
                self._segments_from_base_payload(payload, signed_progress)
            )

    def _make_patrol_waypoints(self) -> list[tuple[float, float]]:
        pattern = str(self.get_parameter("patrol_pattern").value).strip().lower()
        if pattern not in ("box_entry", "rectangle_entry", "real_box", "sim_box"):
            self.patrol_loop_start_index = 0
            return []
        width = max(float(self.get_parameter("patrol_box_width_m").value), 0.2)
        height = max(float(self.get_parameter("patrol_box_height_m").value), 0.2)
        entry = max(float(self.get_parameter("patrol_entry_m").value), 0.0)
        half_width = width * 0.5
        bottom_x = entry
        top_x = entry + height
        self.patrol_loop_start_index = 2
        route = [
            (0.0, 0.0),
            (bottom_x, 0.0),
            (bottom_x, -half_width),
            (top_x, -half_width),
            (top_x, half_width),
            (bottom_x, half_width),
        ]
        self._event(
            "patrol_route",
            {
                "pattern": "box_entry",
                "entry_m": entry,
                "box_width_m": width,
                "box_height_m": height,
                "waypoints": route,
                "loop_start_index": self.patrol_loop_start_index,
            },
        )
        return route

    def _advance_patrol_index(self) -> None:
        with self.lock:
            self.patrol_current_index = self.patrol_target_index
            if self.patrol_target_index >= len(self.patrol_waypoints) - 1:
                self.patrol_target_index = min(
                    max(self.patrol_loop_start_index, 0), len(self.patrol_waypoints) - 1
                )
                self.cycle_count += 1
            else:
                self.patrol_target_index += 1
            max_round_trips = int(self.get_parameter("max_round_trips").value)
            if max_round_trips > 0 and self.cycle_count >= max_round_trips:
                self.patrol_target_index = len(self.patrol_waypoints)

    def _new_base_request_id(self, label: str) -> str:
        return f"{label}-{time.monotonic_ns()}"

    def _angle_diff(self, target: float, source: float) -> float:
        return math.atan2(math.sin(target - source), math.cos(target - source))

    def _segments_from_base_payload(
        self, payload: dict[str, Any], progress_delta_m: float | None = None
    ) -> list[dict[str, Any]]:
        command = str(payload.get("command", "")).strip().lower()
        if command in ("replay_segments", "replay"):
            segments = payload.get("segments", [])
            if isinstance(segments, list):
                return [dict(item) for item in segments if isinstance(item, dict)]
            return []
        if command == "drive_relative":
            distance = (
                float(progress_delta_m)
                if progress_delta_m is not None
                else float(payload.get("distance_m", payload.get("distance", 0.0)))
            )
            return [
                {
                    "type": "linear",
                    "action": "forward" if distance >= 0.0 else "back",
                    "distance_m": abs(distance),
                    "signed_distance_m": distance,
                    "linear_x": float(payload.get("speed_m_s", payload.get("speed_mps", 0.0))),
                }
            ]
        if command == "turn_relative":
            angle = float(payload.get("angle_rad", 0.0))
            return [
                {
                    "type": "angular",
                    "action": "left" if angle >= 0.0 else "right",
                    "angle_rad": abs(angle),
                    "signed_angle_rad": angle,
                }
            ]
        return []

    def _inverse_base_segments(self, segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
        inverse: list[dict[str, Any]] = []
        for segment in reversed(segments):
            item = dict(segment)
            motion_type = str(item.get("type", "")).strip().lower()
            if motion_type == "linear":
                signed = float(item.get("signed_distance_m", item.get("distance_m", 0.0)))
                item["signed_distance_m"] = -signed
                item["distance_m"] = abs(float(item["signed_distance_m"]))
                item["action"] = "forward" if float(item["signed_distance_m"]) >= 0.0 else "back"
            elif motion_type == "angular":
                signed = float(item.get("signed_angle_rad", item.get("angle_rad", 0.0)))
                item["signed_angle_rad"] = -signed
                item["angle_rad"] = abs(float(item["signed_angle_rad"]))
                item["action"] = "left" if float(item["signed_angle_rad"]) >= 0.0 else "right"
            else:
                linear = float(item.get("linear_x", 0.0))
                angular = float(item.get("angular_z", 0.0))
                item["linear_x"] = -linear
                item["angular_z"] = -angular
            inverse.append(item)
        return inverse

    def _handle_candidate(self, candidate: Candidate) -> None:
        max_recovery = (
            max(int(self.get_parameter("reach_recovery_max_attempts").value), 0)
            if bool(self.get_parameter("reach_recovery_enabled").value)
            else 0
        )
        attempt = 0
        while rclpy.ok() and not self.cancel_event.is_set():
            with self.lock:
                self.active_candidate = candidate
                self.latest_candidate = None
                self.grasp_state = {}
            suffix = f" recovery_try={attempt}/{max_recovery}" if attempt else ""
            self._set_state(
                "grasp",
                f"detected {candidate.class_name}; stop base and run grasp{suffix}",
            )
            self._call_trigger(self.base_stop_client, 1.0)
            if not self._wait_for_base_idle(float(self.get_parameter("base_stop_wait_sec").value)):
                self._event("base_stop_wait_timeout", {"base": self._query_base_status()})
            ok, message = self._call_trigger(self.grasp_start_client, 3.0)
            if not ok and self._grasp_start_retryable(message):
                time.sleep(0.3)
                ok, message = self._call_trigger(self.grasp_start_client, 3.0)
            if not ok:
                self._finish("failed", f"grasp start failed: {message}")
                return

            state, text = self._wait_for_grasp_terminal()
            if self.cancel_event.is_set():
                self._call_trigger(self.grasp_stop_client, 1.0)
                return
            if state == "succeeded":
                self._event(
                    "grasp_complete",
                    {"candidate": asdict(candidate), "recovery_attempts": attempt},
                )
                with self.lock:
                    self.active_candidate = None
                    self.successful_grasps += 1
                return

            failure = f"grasp {state or 'failed'}: {text}".strip()
            with self.lock:
                self.last_grasp_failure = failure
            if not self._should_recover_after_grasp_failure(state, text):
                if bool(self.get_parameter("continue_on_grasp_failure").value):
                    self._event(
                        "grasp_failed_continue",
                        {"candidate": asdict(candidate), "failure": failure},
                    )
                    with self.lock:
                        self.active_candidate = None
                    self._set_state("patrol", f"skip failed {candidate.class_name}; continue patrol")
                    return
                self._finish("failed", failure)
                return
            if attempt >= max_recovery:
                self._event(
                    "reach_recovery_exhausted",
                    {
                        "candidate": asdict(candidate),
                        "attempts": attempt,
                        "failure": failure,
                    },
                )
                with self.lock:
                    self.active_candidate = None
                if bool(self.get_parameter("reach_recovery_continue_on_exhausted").value):
                    self._set_state(
                        "patrol",
                        f"skip unreachable {candidate.class_name}; continue patrol",
                    )
                    return
                self._finish("failed", failure)
                return

            attempt += 1
            with self.lock:
                self.recovery_attempts += 1
            if not self._run_reach_recovery_step(attempt, failure):
                return
            updated = self._wait_for_recovery_candidate()
            if updated is None:
                self._event(
                    "reach_recovery_no_redetect",
                    {
                        "candidate": asdict(candidate),
                        "attempt": attempt,
                        "failure": failure,
                    },
                )
                with self.lock:
                    self.active_candidate = None
                self._set_state("patrol", "no fresh detection after base recovery; resume patrol")
                return
            candidate = updated

    def _wait_for_grasp_terminal(self) -> tuple[str, str]:
        deadline = time.monotonic() + max(float(self.get_parameter("grasp_timeout_sec").value), 5.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() < deadline:
            with self.lock:
                state = str(self.grasp_state.get("state", "")).lower()
                text = str(self.grasp_state.get("message", ""))
            if state in TERMINAL_STATES:
                return state, text
            time.sleep(0.1)
        return "canceled" if self.cancel_event.is_set() else "failed", "grasp timeout"

    def _should_recover_after_grasp_failure(self, state: str, text: str) -> bool:
        if state == "canceled" or self.cancel_event.is_set():
            return False
        lower = text.lower()
        if "gripper" in lower or "capture" in lower:
            return False
        recover_tokens = (
            "planning",
            "planner",
            "plan",
            "ik",
            "unreachable",
            "trajectory unavailable",
            "no_ik_solution",
            "return code",
            "timeout",
        )
        return state == "failed" and any(token in lower for token in recover_tokens)

    def _run_reach_recovery_step(self, attempt: int, failure: str) -> bool:
        distance = abs(float(self.get_parameter("reach_recovery_step_m").value)) * float(
            self.direction
        )
        self._set_state(
            "recover",
            f"grasp unreachable/planning failed; move base {distance:.2f}m and recalc",
        )
        self._event(
            "reach_recovery_start",
            {"attempt": attempt, "distance_m": distance, "failure": failure},
        )
        with self.lock:
            self.latest_candidate = None
        self._restart_visual_search("reach recovery before base move")
        payload = {"command": "drive_relative", "distance_m": distance}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.base_command_pub.publish(msg)

        command_started = False
        started = time.monotonic()
        timeout = max(float(self.get_parameter("base_step_timeout_sec").value), 1.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() - started < timeout:
            base_snapshot = self._query_base_status()
            if self._base_matches_command(base_snapshot, payload):
                state = str(base_snapshot.get("state", "")).lower()
                worker_busy = bool(base_snapshot.get("worker_busy", False))
                command_started = command_started or worker_busy or state == "running"
                if command_started and self._base_terminal(base_snapshot):
                    if state == "failed":
                        self._finish("failed", str(base_snapshot.get("message", "base failed")))
                        return False
                    if state == "canceled":
                        return False
                    result = base_snapshot.get("latest_result", {})
                    target = float(result.get("target_m", distance))
                    with self.lock:
                        self.progress_m += target if abs(target) > 1e-6 else distance
                    self._restart_visual_search("reach recovery after base move")
                    self._event("reach_recovery_done", {"attempt": attempt, "distance_m": distance})
                    return True
            elif not command_started:
                msg = String()
                msg.data = json.dumps(payload, sort_keys=True)
                self.base_command_pub.publish(msg)
            elif self._base_terminal(base_snapshot):
                self._finish("failed", "base recovery command ended without matching active command")
                return False
            if self._base_terminal() and command_started:
                with self.lock:
                    self.progress_m += distance
                self._restart_visual_search("reach recovery after base move")
                self._event("reach_recovery_done", {"attempt": attempt, "distance_m": distance})
                return True
            time.sleep(0.05)
        self._call_trigger(self.base_stop_client, 1.0)
        if self.cancel_event.is_set():
            return False
        self._finish("failed", "base reach-recovery step timeout")
        return False

    def _wait_for_recovery_candidate(self) -> Candidate | None:
        deadline = time.monotonic() + max(
            float(self.get_parameter("reach_recovery_wait_detection_sec").value), 0.1
        )
        self._set_state("tracking", "visual tracking/re-detect after base recovery")
        next_restart = 0.0
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_restart:
                self._restart_visual_search("tracking target after reach recovery")
                next_restart = now + 0.8
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._event("reach_recovery_redetect", asdict(candidate))
                return candidate
            time.sleep(0.05)
        return None

    def _restart_visual_search(self, reason: str) -> None:
        self.restart_search_pub.publish(Empty())
        self._event("visual_search_restart", {"reason": reason})

    def _set_real_search_scan(self, enabled: bool) -> None:
        self.real_search_scan_enabled = bool(enabled)
        self._publish_real_search_scan()
        self._event("real_search_scan", {"enabled": bool(enabled)})

    def _publish_real_search_scan(self) -> None:
        msg = Bool()
        msg.data = bool(self.real_search_scan_enabled)
        self.real_search_scan_pub.publish(msg)

    def _fresh_candidate(self) -> Candidate | None:
        timeout = max(float(self.get_parameter("detection_timeout_sec").value), 0.1)
        with self.lock:
            candidate = self.latest_candidate
        if candidate is None:
            return None
        try:
            stamp = datetime.fromisoformat(candidate.received_at)
            age = (datetime.now() - stamp).total_seconds()
        except ValueError:
            age = 0.0
        return candidate if age <= timeout else None

    def _base_terminal(self, snapshot: dict[str, Any] | None = None) -> bool:
        if snapshot is None:
            with self.lock:
                snapshot = dict(self.base_state)
        state = str(snapshot.get("state", "")).lower()
        worker_busy = bool(snapshot.get("worker_busy", False))
        return state in ("succeeded", "failed", "canceled", "idle") and not worker_busy

    def _wait_for_base_idle(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while rclpy.ok() and time.monotonic() <= deadline:
            if self._base_terminal(self._query_base_status()):
                return True
            time.sleep(0.05)
        return self._base_terminal(self._query_base_status())

    def _grasp_start_retryable(self, message: str) -> bool:
        snapshot = self._parse_json_object(message)
        if not snapshot:
            return False
        state = str(snapshot.get("state", "")).lower()
        base = snapshot.get("base", {})
        worker_busy = bool(snapshot.get("worker_busy", False))
        base_busy = isinstance(base, dict) and bool(base.get("worker_busy", False))
        return state in ("idle", "preflight", "running") or worker_busy or base_busy

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

    def _base_matches_command(self, snapshot: dict[str, Any], payload: dict[str, Any]) -> bool:
        active = snapshot.get("active_command", {})
        if not isinstance(active, dict):
            return False
        if "request_id" in payload:
            return str(active.get("request_id", "")) == str(payload.get("request_id", ""))
        if str(active.get("command", "")).strip().lower() != str(payload.get("command", "")).strip().lower():
            return False
        for key in ("distance_m", "speed_m_s"):
            if key in payload:
                try:
                    if abs(float(active.get(key, 0.0)) - float(payload[key])) > 1e-4:
                        return False
                except (TypeError, ValueError):
                    return False
        return True

    def _is_terminal(self) -> bool:
        with self.lock:
            return self.state in TERMINAL_STATES

    def _parse_candidates(self, text: str) -> list[Candidate]:
        payload = self._parse_json_object(text)
        if not payload:
            return []
        if isinstance(payload.get("instances"), list) and payload["instances"]:
            items = [item for item in payload["instances"] if isinstance(item, dict)]
        elif isinstance(payload.get("detections"), list) and payload["detections"]:
            items = [item for item in payload["detections"] if isinstance(item, dict)]
        else:
            items = [payload]
        return [self._candidate_from_raw(item) for item in items]

    def _parse_candidate(self, text: str) -> Candidate | None:
        candidates = self._parse_candidates(text)
        if not candidates:
            return None
        return max(candidates, key=lambda item: item.confidence)

    def _candidate_from_raw(self, raw: dict[str, Any]) -> Candidate:
        class_name = str(
            raw.get("taco_class")
            or raw.get("class_name")
            or raw.get("class")
            or raw.get("label")
            or "trash"
        )
        confidence = float(raw.get("confidence", raw.get("score", 0.0)))
        return Candidate(
            class_name=class_name,
            confidence=confidence,
            message=f"{class_name} {confidence:.2f}",
            received_at=datetime.now().isoformat(timespec="milliseconds"),
            raw=dict(raw),
        )

    def _candidate_reachable(self, candidate: Candidate) -> tuple[bool, str]:
        raw = candidate.raw
        has_3d = bool(raw.get("has_3d", False))
        if bool(self.get_parameter("require_3d_candidate").value) and not has_3d:
            return False, "waiting for 3D candidate"
        depth = self._optional_float(raw.get("depth_m"))
        max_depth = float(self.get_parameter("candidate_max_depth_m").value)
        if depth is not None and depth > max_depth:
            return False, f"depth {depth:.2f}m > {max_depth:.2f}m"
        base_xyz = raw.get("base_grasp_xyz")
        if not isinstance(base_xyz, list) or len(base_xyz) < 2:
            base_xyz = self._planning_waypoint_xyz(raw, "grasp")
        if isinstance(base_xyz, list) and len(base_xyz) >= 2:
            x = self._optional_float(base_xyz[0])
            y = self._optional_float(base_xyz[1])
            z = self._optional_float(base_xyz[2]) if len(base_xyz) >= 3 else None
            if x is not None:
                min_x = float(self.get_parameter("candidate_min_base_x_m").value)
                max_x = float(self.get_parameter("candidate_max_base_x_m").value)
                if x < min_x:
                    return False, f"base_x {x:.2f}m < {min_x:.2f}m"
                if x > max_x:
                    return False, f"base_x {x:.2f}m > {max_x:.2f}m"
                reach = math.hypot(x, y or 0.0)
                max_reach = float(self.get_parameter("candidate_max_reach_m").value)
                if reach > max_reach:
                    return False, f"reach {reach:.2f}m > {max_reach:.2f}m"
            if y is not None:
                max_abs_y = float(self.get_parameter("candidate_max_abs_base_y_m").value)
                if abs(y) > max_abs_y:
                    return False, f"base_y {y:.2f}m outside ±{max_abs_y:.2f}m"
            if z is not None:
                min_z = float(self.get_parameter("candidate_min_base_z_m").value)
                if z < min_z:
                    return False, f"base_z {z:.2f}m < {min_z:.2f}m"
        elif bool(self.get_parameter("require_3d_candidate").value):
            return False, "3D candidate missing base grasp"
        return True, "reachable"

    def _planning_waypoint_xyz(self, raw: dict[str, Any], name: str) -> list[float]:
        for key in ("planning_key_waypoints", "waypoints_base"):
            items = raw.get(key)
            if not isinstance(items, list):
                continue
            for item in items:
                if not isinstance(item, dict):
                    continue
                if str(item.get("name", "")).strip() == name and isinstance(item.get("xyz"), list):
                    return item["xyz"]
        return []

    def _optional_float(self, value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _parse_json_object(self, text: str) -> dict[str, Any]:
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return {}
        return payload if isinstance(payload, dict) else {}

    def _call_trigger(self, client, timeout: float) -> tuple[bool, str]:
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
            return False, f"empty response: {getattr(client, 'srv_name', 'trigger')}"
        return bool(response.success), str(response.message)

    def _call_trigger_background(self, client, timeout: float, label: str) -> None:
        def worker() -> None:
            ok, message = self._call_trigger(client, timeout)
            if not ok:
                self._event("async_service_failed", {"service": label, "message": message})

        threading.Thread(target=worker, daemon=True).start()

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _finish(self, state: str, message: str) -> None:
        if state in TERMINAL_STATES:
            self._set_real_search_scan(False)
        with self.lock:
            self.finished_at = datetime.now().isoformat(timespec="seconds")
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _event(self, kind: str, data: dict[str, Any]) -> None:
        event = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            **data,
        }
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False, sort_keys=True)
        self.event_pub.publish(msg)
        if kind == "state":
            self.get_logger().info(f"{data.get('state')}: {data.get('message')}")

    def _snapshot(self) -> CleanupSnapshot:
        with self.lock:
            latest = asdict(self.latest_candidate) if self.latest_candidate is not None else {}
            active = asdict(self.active_candidate) if self.active_candidate is not None else {}
            return CleanupSnapshot(
                state=self.state,
                message=self.message,
                started_at=self.started_at,
                finished_at=self.finished_at,
                direction=self.direction,
                progress_m=self.progress_m,
                cycle_count=self.cycle_count,
                latest_candidate=latest,
                active_candidate=active,
                recovery_attempts=self.recovery_attempts,
                last_grasp_failure=self.last_grasp_failure,
            )

    def _snapshot_json(self) -> str:
        return json.dumps(asdict(self._snapshot()), ensure_ascii=False, sort_keys=True)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._snapshot_json()
        self.state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoadCleanupTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.remove_node(node)
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
