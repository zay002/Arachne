#!/usr/bin/env python3
"""Mock smoke test for road_cleanup_task_server.

This runs the real RoadCleanupTaskServer in-process with mock grasp/base
services. It verifies the current road demo route: scan while driving a simple
forward line, then replay the completed base segments back home without adding
scan/grasp work to the return leg.
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from arachne_operator.road_cleanup_task_server import RoadCleanupTaskServer


class MockWorld(Node):
    def __init__(self) -> None:
        super().__init__("road_cleanup_mock_world")
        self.grasp_starts = 0
        self.base_commands: list[dict] = []
        self.base_state = "idle"
        self.base_message = "mock base idle"
        self.base_active_command: dict = {}
        self.base_latest_result: dict = {}
        self.restart_count = 0
        self.events: list[dict] = []
        self._mock_timers: list[threading.Timer] = []
        self.grasp_state_pub = self.create_publisher(String, "/arachne/grasp_task/state", 10)
        self.base_state_pub = self.create_publisher(String, "/arachne/grasp_task/base_state", 10)
        self.detect_pub = self.create_publisher(String, "/arachne/perception/taco_instances", 10)
        self.create_subscription(String, "/arachne/grasp_task/base_command", self._base_command_cb, 10)
        self.create_subscription(Empty, "/arachne/grasp_preview/restart_search", self._restart_cb, 10)
        self.create_subscription(String, "/arachne/road_cleanup/event", self._event_cb, 10)
        self.create_service(Trigger, "/arachne/grasp_task/preflight", self._ok)
        self.create_service(Trigger, "/arachne/grasp_task/stop", self._ok)
        self.create_service(Trigger, "/arachne/grasp_task/base_stop", self._base_stop)
        self.create_service(Trigger, "/arachne/grasp_task/base_status", self._base_status)
        self.create_service(Trigger, "/arachne/grasp_task/status", self._status)
        self.create_service(Trigger, "/arachne/grasp_task/start", self._grasp_start)

    def _ok(self, _request, response):
        response.success = True
        response.message = "mock ok"
        return response

    def _status(self, _request, response):
        response.success = True
        response.message = "{}"
        return response

    def _base_stop(self, _request, response):
        self._publish_base("idle", "mock base stop")
        response.success = True
        response.message = "mock base stop"
        return response

    def _base_status(self, _request, response):
        response.success = True
        response.message = json.dumps(
            {
                "state": self.base_state,
                "message": self.base_message,
                "worker_busy": self.base_state == "running",
                "active_command": self.base_active_command,
                "latest_result": self.base_latest_result,
            },
            sort_keys=True,
        )
        return response

    def _grasp_start(self, _request, response):
        self.grasp_starts += 1
        attempt = self.grasp_starts
        self._publish_grasp("running", f"mock grasp attempt {attempt}")
        self._schedule(0.25, self._finish_grasp, attempt)
        response.success = True
        response.message = "mock grasp started"
        return response

    def _finish_grasp(self, attempt: int) -> None:
        self._publish_grasp("succeeded", f"mock grasp attempt {attempt} complete")

    def _base_command_cb(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self.base_commands.append(payload)
        self.base_active_command = dict(payload)
        distance = float(payload.get("distance_m", 0.0))
        if str(payload.get("command", "")).strip().lower() in ("replay_segments", "replay"):
            distance = sum(
                abs(float(segment.get("signed_distance_m", segment.get("distance_m", 0.0))))
                for segment in payload.get("segments", [])
                if isinstance(segment, dict)
            )
        self.base_latest_result = {
            "target_m": distance,
            "progress_m": distance,
        }
        self._publish_base("running", "mock base moving")
        self._schedule(0.12, self._publish_base, "succeeded", "mock base done")

    def _schedule(self, delay: float, callback, *args) -> None:
        timer = threading.Timer(delay, callback, args=args)
        self._mock_timers.append(timer)
        timer.start()

    def cancel_timers(self) -> None:
        for timer in self._mock_timers:
            timer.cancel()
        for timer in self._mock_timers:
            timer.join(timeout=0.2)

    def _restart_cb(self, _msg: Empty) -> None:
        self.restart_count += 1

    def _event_cb(self, msg: String) -> None:
        try:
            self.events.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _publish_grasp(self, state: str, message: str) -> None:
        msg = String()
        msg.data = json.dumps({"state": state, "message": message}, sort_keys=True)
        self.grasp_state_pub.publish(msg)

    def _publish_base(self, state: str, message: str) -> None:
        self.base_state = state
        self.base_message = message
        msg = String()
        msg.data = json.dumps(
            {"state": state, "message": message, "worker_busy": state == "running"},
            sort_keys=True,
        )
        self.base_state_pub.publish(msg)

    def publish_detection(self, label: str = "bottle") -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "instances": [
                    {
                        "class_name": label,
                        "confidence": 0.91,
                        "bbox_xyxy": [10, 20, 80, 120],
                        "has_mask": True,
                    }
                ]
            },
            sort_keys=True,
        )
        self.detect_pub.publish(msg)


def call_trigger(node: Node, name: str, timeout: float = 4.0):
    client = node.create_client(Trigger, name)
    deadline = time.monotonic() + timeout
    while not client.wait_for_service(timeout_sec=0.1):
        if time.monotonic() > deadline:
            raise RuntimeError(f"service unavailable: {name}")
    future = client.call_async(Trigger.Request())
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        raise RuntimeError(f"service timeout: {name}")
    return future.result()


def main() -> None:
    rclpy.init()
    server = RoadCleanupTaskServer()
    server.set_parameters(
        [
            Parameter("patrol_pattern", Parameter.Type.STRING, "line"),
            Parameter("patrol_distance_m", Parameter.Type.DOUBLE, 0.4),
            Parameter("patrol_step_m", Parameter.Type.DOUBLE, 0.1),
            Parameter("base_step_timeout_sec", Parameter.Type.DOUBLE, 2.0),
            Parameter("grasp_timeout_sec", Parameter.Type.DOUBLE, 3.0),
            Parameter("initial_detection_wait_sec", Parameter.Type.DOUBLE, 0.0),
            Parameter("loop", Parameter.Type.BOOL, False),
        ]
    )
    mock = MockWorld()
    client_node = Node("road_cleanup_mock_client")
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (server, mock, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(0.5)
        response = call_trigger(client_node, "/arachne/road_cleanup/start")
        assert response.success, response.message
        final = None
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            response = call_trigger(client_node, "/arachne/road_cleanup/status")
            snapshot = json.loads(response.message)
            if snapshot.get("state") in ("succeeded", "failed", "canceled"):
                final = snapshot
                break
            time.sleep(0.2)
        event_kinds = [event.get("kind") for event in mock.events]
        assert final and final["state"] == "succeeded", final
        forward_commands = [
            command
            for command in mock.base_commands
            if command.get("command") == "drive_relative" and command.get("distance_m", 0.0) > 0.0
        ]
        return_commands = [
            command
            for command in mock.base_commands
            if command.get("command") == "replay_segments"
        ]
        assert forward_commands, mock.base_commands
        assert return_commands, mock.base_commands
        assert mock.grasp_starts == 0, mock.grasp_starts
        assert "base_step_done" in event_kinds, event_kinds
        assert "return_home_done" in event_kinds, event_kinds
        print("road_cleanup mock smoke passed")
        print(json.dumps(final, ensure_ascii=False, sort_keys=True))
    finally:
        mock.cancel_timers()
        executor.shutdown()
        for node in (client_node, mock, server):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
