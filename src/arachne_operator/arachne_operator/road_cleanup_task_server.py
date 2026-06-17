from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


TERMINAL_STATES = {"succeeded", "failed", "canceled"}


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
        self.declare_parameter("restart_search_topic", "/arachne/grasp_preview/restart_search")
        self.declare_parameter("patrol_distance_m", 1.2)
        self.declare_parameter("patrol_step_m", 1.2)
        self.declare_parameter("max_round_trips", 2)
        self.declare_parameter("detection_confidence", 0.35)
        self.declare_parameter("detection_timeout_sec", 1.2)
        self.declare_parameter("base_step_timeout_sec", 8.0)
        self.declare_parameter("grasp_timeout_sec", 90.0)
        self.declare_parameter("reach_recovery_enabled", True)
        self.declare_parameter("reach_recovery_max_attempts", 3)
        self.declare_parameter("reach_recovery_step_m", 0.10)
        self.declare_parameter("reach_recovery_wait_detection_sec", 3.0)
        self.declare_parameter("reach_recovery_continue_on_exhausted", True)
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

        self.state_pub = self.create_publisher(String, "/arachne/road_cleanup/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/road_cleanup/event", 10)
        self.base_command_pub = self.create_publisher(
            String, str(self.get_parameter("base_command_topic").value), 10
        )
        self.restart_search_pub = self.create_publisher(
            Empty, str(self.get_parameter("restart_search_topic").value), 10
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

        self.create_timer(0.5, self._publish_state)
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
            if busy or self.state not in ("idle", *TERMINAL_STATES):
                response.success = False
                response.message = self._snapshot_json()
                return response
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run_task, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _stop_cb(self, _request, response):
        self.cancel_event.set()
        self._call_trigger(self.base_stop_client, 1.0)
        self._call_trigger(self.grasp_stop_client, 1.0)
        self._set_state("canceled", "road cleanup stopping")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request, response):
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request, response):
        ok, message = self._call_trigger(self.grasp_preflight_client, 4.0)
        response.success = ok
        response.message = message
        return response

    def _detection_cb(self, msg: String) -> None:
        candidate = self._parse_candidate(msg.data)
        if candidate is None:
            return
        threshold = float(self.get_parameter("detection_confidence").value)
        if candidate.confidence < threshold:
            return
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
            self.active_candidate = None
            self.recovery_attempts = 0
            self.last_grasp_failure = ""
        self._set_state("preflight", "checking camera, base and grasp primitive")
        ok, message = self._call_trigger(self.grasp_preflight_client, 5.0)
        if not ok:
            self._finish("failed", f"preflight failed: {message}")
            return

        self._set_state("patrol", "patrol forward/back while grasp_server YOLO-SEG watches")
        while rclpy.ok() and not self.cancel_event.is_set():
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._handle_candidate(candidate)
                if self._is_terminal():
                    break
                if self.cancel_event.is_set():
                    break
                self._set_state("patrol", "resume patrol after grasp")
                continue

            if not self._patrol_step():
                break

        if self.cancel_event.is_set():
            self._finish("canceled", "road cleanup canceled")
        elif self.state not in TERMINAL_STATES:
            self._finish("succeeded", "road cleanup complete")

    def _patrol_step(self) -> bool:
        distance_limit = abs(float(self.get_parameter("patrol_distance_m").value))
        step = abs(float(self.get_parameter("patrol_step_m").value))
        remaining = distance_limit - abs(self.progress_m)
        if remaining <= 1e-3:
            with self.lock:
                self.direction *= -1
                self.progress_m = 0.0
                self.cycle_count += 1
            if not bool(self.get_parameter("loop").value) and self.cycle_count >= 2:
                return False
            max_round_trips = int(self.get_parameter("max_round_trips").value)
            if max_round_trips > 0 and self.cycle_count >= max_round_trips * 2:
                return False
            return True

        distance = min(step, remaining) * float(self.direction)
        self._set_state(
            "patrol",
            f"scan while moving {'forward' if self.direction > 0 else 'back'} step={distance:.2f}m",
        )
        payload = {"command": "drive_relative", "distance_m": distance}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.base_command_pub.publish(msg)
        started = time.monotonic()
        timeout = max(float(self.get_parameter("base_step_timeout_sec").value), 1.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() - started < timeout:
            candidate = self._fresh_candidate()
            if candidate is not None:
                self._call_trigger(self.base_stop_client, 1.0)
                self._handle_candidate(candidate)
                if self._is_terminal():
                    return False
                return True
            if self._base_terminal():
                with self.lock:
                    self.progress_m += distance
                return True
            time.sleep(0.05)
        self._call_trigger(self.base_stop_client, 1.0)
        if self.cancel_event.is_set():
            return False
        self._finish("failed", "base patrol step timeout")
        return False

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
                return

            failure = f"grasp {state or 'failed'}: {text}".strip()
            with self.lock:
                self.last_grasp_failure = failure
            if not self._should_recover_after_grasp_failure(state, text):
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

        started = time.monotonic()
        timeout = max(float(self.get_parameter("base_step_timeout_sec").value), 1.0)
        while rclpy.ok() and not self.cancel_event.is_set() and time.monotonic() - started < timeout:
            if self._base_terminal():
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

    def _base_terminal(self) -> bool:
        with self.lock:
            state = str(self.base_state.get("state", "")).lower()
            worker_busy = bool(self.base_state.get("worker_busy", False))
        return state in ("succeeded", "failed", "canceled", "idle") and not worker_busy

    def _is_terminal(self) -> bool:
        with self.lock:
            return self.state in TERMINAL_STATES

    def _parse_candidate(self, text: str) -> Candidate | None:
        payload = self._parse_json_object(text)
        if not payload:
            return None
        if isinstance(payload.get("instances"), list) and payload["instances"]:
            items = [item for item in payload["instances"] if isinstance(item, dict)]
        elif isinstance(payload.get("detections"), list) and payload["detections"]:
            items = [item for item in payload["detections"] if isinstance(item, dict)]
        else:
            items = [payload]
        if not items:
            return None
        best = max(items, key=lambda item: float(item.get("confidence", item.get("score", 0.0))))
        class_name = str(
            best.get("taco_class")
            or best.get("class_name")
            or best.get("class")
            or best.get("label")
            or "trash"
        )
        confidence = float(best.get("confidence", best.get("score", 0.0)))
        return Candidate(
            class_name=class_name,
            confidence=confidence,
            message=f"{class_name} {confidence:.2f}",
            received_at=datetime.now().isoformat(timespec="milliseconds"),
            raw=dict(best),
        )

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

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _finish(self, state: str, message: str) -> None:
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
