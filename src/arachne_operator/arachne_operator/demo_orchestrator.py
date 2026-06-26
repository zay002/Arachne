from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arachne_hardware.action import AuboMoveJoint


MANAGED_PROCESS_COMMAND_PARAMS = {
    "camera": "camera_command",
    "depth_pointcloud": "depth_pointcloud_command",
    "viewer": "camera_view_command",
    "grasp_server": "grasp_server_command",
    "cleanup_server": "cleanup_server_command",
}
MANAGED_PROCESS_READY_SERVICES = {
    "grasp_server": ("/arachne/grasp_task/status", "/arachne/grasp_task/start"),
    "cleanup_server": ("/arachne/road_cleanup/status", "/arachne/road_cleanup/start"),
}
MANAGED_PROCESS_READY_TOPICS = {
    "grasp_server": "/arachne/grasp_task/state",
    "cleanup_server": "/arachne/road_cleanup/state",
}


class DemoOrchestrator(Node):
    """Demo-level orchestration for camera, grasp, and cleanup flows.

    This node intentionally does not execute arm motion directly.  Aubo joint
    motion remains behind /arachne/aubo/move_joint and the task servers.
    """

    def __init__(self) -> None:
        super().__init__("demo_orchestrator")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("gripper_status_topic", "/arachne/gripper/status")
        self.declare_parameter("aubo_teach_command_topic", "/arachne/aubo/teach_command")
        self.declare_parameter("aubo_move_joint_action_name", "/arachne/aubo/move_joint")
        self.declare_parameter("camera_color_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("grasp_task_start_service", "/arachne/grasp_task/start")
        self.declare_parameter("grasp_task_stop_service", "/arachne/grasp_task/stop")
        self.declare_parameter("grasp_task_preflight_service", "/arachne/grasp_task/preflight")
        self.declare_parameter("cleanup_task_start_service", "/arachne/road_cleanup/start")
        self.declare_parameter("cleanup_task_stop_service", "/arachne/road_cleanup/stop")
        self.declare_parameter("cleanup_task_preflight_service", "/arachne/road_cleanup/preflight")
        self.declare_parameter("skip_task_preflight", True)
        self.declare_parameter("camera_command", "")
        self.declare_parameter("depth_pointcloud_command", "")
        self.declare_parameter("camera_view_command", "")
        self.declare_parameter("grasp_server_command", "")
        self.declare_parameter("cleanup_server_command", "")
        self.declare_parameter("service_stop_timeout_sec", 4.0)
        self.declare_parameter("runtime_log_root", "log/demo_orchestrator")
        self.declare_parameter("workspace_root", "")
        self.declare_parameter("aubo_teach_exit_settle_sec", 0.5)
        self.declare_parameter(
            "arm_state_joint_names",
            "shoulder_joint,upperArm_joint,foreArm_joint,wrist1_joint,wrist2_joint,wrist3_joint",
        )

        self.workspace_root = self._workspace_root()
        self.runtime_log_dir = self._runtime_log_dir()
        self.runtime_log_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.RLock()
        self.managed_processes: dict[str, subprocess.Popen] = {}
        self.managed_process_logs: dict[str, Path] = {}
        self.managed_process_log_handles: dict[str, Any] = {}
        self.last_error = ""
        self.state = "ready"
        self.last_joint_state: JointState | None = None
        self.last_joint_state_stamp = 0.0
        self.last_odom_stamp = 0.0
        self.last_gripper_status_stamp = 0.0
        self.arm_state_joint_names = self._parse_names(
            str(self.get_parameter("arm_state_joint_names").value)
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.aubo_teach_pub = self.create_publisher(
            String, str(self.get_parameter("aubo_teach_command_topic").value), 10
        )
        self.state_pub = self.create_publisher(String, "/arachne/demo/state", 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_status_topic").value),
            self._gripper_status_callback,
            10,
        )

        self.aubo_move_joint_client = ActionClient(
            self, AuboMoveJoint, str(self.get_parameter("aubo_move_joint_action_name").value)
        )
        self.grasp_start_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_task_start_service").value)
        )
        self.grasp_stop_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_task_stop_service").value)
        )
        self.grasp_preflight_client = self.create_client(
            Trigger, str(self.get_parameter("grasp_task_preflight_service").value)
        )
        self.cleanup_start_client = self.create_client(
            Trigger, str(self.get_parameter("cleanup_task_start_service").value)
        )
        self.cleanup_stop_client = self.create_client(
            Trigger, str(self.get_parameter("cleanup_task_stop_service").value)
        )
        self.cleanup_preflight_client = self.create_client(
            Trigger, str(self.get_parameter("cleanup_task_preflight_service").value)
        )

        self.create_service(Trigger, "/arachne/demo/start_camera", self._srv_start_camera)
        self.create_service(Trigger, "/arachne/demo/stop_camera", self._srv_stop_camera)
        self.create_service(
            Trigger, "/arachne/demo/start_visual_grasp", self._srv_start_visual_grasp
        )
        self.create_service(
            Trigger, "/arachne/demo/start_road_cleanup", self._srv_start_road_cleanup
        )
        self.create_service(
            Trigger, "/arachne/demo/pause_road_cleanup", self._srv_pause_road_cleanup
        )
        self.create_service(Trigger, "/arachne/demo/return_home", self._srv_return_home)
        self.create_service(Trigger, "/arachne/demo/stop", self._srv_stop)
        self.create_service(Trigger, "/arachne/demo/preflight", self._srv_preflight)
        self.create_service(Trigger, "/arachne/demo/status", self._srv_status)
        self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info("demo_orchestrator ready")

    def destroy_node(self) -> bool:
        self._stop_all_processes(quiet=True)
        return super().destroy_node()

    def _srv_start_camera(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_state("starting_camera")
        self._start_process("camera")
        time.sleep(0.8)
        self._start_process("depth_pointcloud")
        self._start_process("viewer")
        self._set_state("ready")
        return self._response(response, True, "camera/depth debug/viewer start requested")

    def _srv_stop_camera(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_process("viewer")
        self._stop_process("depth_pointcloud")
        self._stop_process("camera")
        return self._response(response, True, "camera/depth debug/viewer stop requested")

    def _srv_start_visual_grasp(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._set_state("starting_visual_grasp")
        self._stop_base()
        self._teach_off()
        self._start_process("camera")
        time.sleep(0.8)
        self._start_process("depth_pointcloud")
        self._start_process("viewer")
        self._start_process("grasp_server")
        if not bool(self.get_parameter("skip_task_preflight").value):
            ok, message = self._wait_task_preflight(self.grasp_preflight_client, timeout_sec=30.0)
            if not ok:
                self._set_error(message)
                return self._response(response, False, f"visual grasp preflight failed: {message}")
        ok, message = self._call_trigger(self.grasp_start_client, "grasp start", wait_timeout=8.0)
        self._set_state("visual_grasp_running" if ok else "ready")
        if not ok:
            self._set_error(message)
        return self._response(response, ok, message)

    def _srv_start_road_cleanup(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._set_state("starting_road_cleanup")
        self._stop_base()
        self._teach_off()
        self._start_process("camera")
        self._start_process("depth_pointcloud")
        self._start_process("grasp_server")
        self._start_process("cleanup_server")
        if not bool(self.get_parameter("skip_task_preflight").value):
            ok, message = self._wait_task_preflight(self.cleanup_preflight_client, timeout_sec=75.0)
            if not ok:
                self._set_error(message)
                return self._response(response, False, f"road cleanup preflight failed: {message}")
        ok, message = self._call_trigger(
            self.cleanup_start_client, "road cleanup start", wait_timeout=8.0
        )
        self._set_state("road_cleanup_running" if ok else "ready")
        if not ok:
            self._set_error(message)
        return self._response(response, ok, message)

    def _srv_pause_road_cleanup(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._stop_base()
        ok, message = self._call_trigger(self.cleanup_stop_client, "road cleanup pause", 4.0)
        self._set_state("road_cleanup_paused" if ok else "ready")
        if not ok:
            self._set_error(message)
        return self._response(response, ok, message)

    def _srv_return_home(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_base()
        self._set_state("ready")
        return self._response(response, True, "return_home is orchestration-only in Phase 3B")

    def _srv_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_state("stopping")
        self._call_trigger(self.cleanup_stop_client, "road cleanup stop", 2.0, require_service=False)
        self._call_trigger(self.grasp_stop_client, "grasp stop", 2.0, require_service=False)
        self._stop_base()
        self._stop_all_processes(quiet=False)
        self._set_state("ready")
        return self._response(response, True, "demo stop requested")

    def _srv_preflight(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        checks = self._preflight_checks()
        ok = all(bool(item["ok"]) or not bool(item["required"]) for item in checks)
        payload = {"state": self.state, "checks": checks}
        return self._response(response, ok, json.dumps(payload, separators=(",", ":")))

    def _srv_status(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._response(response, True, json.dumps(self._state_payload(), separators=(",", ":")))

    def _joint_state_callback(self, msg: JointState) -> None:
        with self.lock:
            self.last_joint_state = msg
            self.last_joint_state_stamp = time.monotonic()

    def _odom_callback(self, msg: Odometry) -> None:
        with self.lock:
            self.last_odom_stamp = time.monotonic()

    def _gripper_status_callback(self, msg: String) -> None:
        with self.lock:
            self.last_gripper_status_stamp = time.monotonic()

    def _preflight_checks(self) -> list[dict[str, Any]]:
        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        now = time.monotonic()
        with self.lock:
            joint_state = self.last_joint_state
            joint_age = now - self.last_joint_state_stamp if self.last_joint_state_stamp else 9999.0
            odom_age = now - self.last_odom_stamp if self.last_odom_stamp else 9999.0
            gripper_age = (
                now - self.last_gripper_status_stamp if self.last_gripper_status_stamp else 9999.0
            )
        checks = [
            self._check_topic("color_image", str(self.get_parameter("camera_color_topic").value), topic_names),
            self._check_topic("depth_image", str(self.get_parameter("camera_depth_topic").value), topic_names),
            self._check_service("grasp_task_service", self.grasp_start_client, timeout_sec=0.05),
            self._check_service("road_cleanup_service", self.cleanup_start_client, timeout_sec=0.05),
            self._check_action("aubo_move_joint_action"),
            {
                "name": "joint_states_aubo",
                "ok": bool(
                    joint_state is not None
                    and joint_age < 5.0
                    and all(name in set(joint_state.name) for name in self.arm_state_joint_names)
                ),
                "required": True,
                "message": f"age={joint_age:.1f}s",
            },
            {
                "name": "odom",
                "ok": odom_age < 5.0,
                "required": True,
                "message": f"age={odom_age:.1f}s",
            },
            {
                "name": "gripper",
                "ok": (
                    str(self.get_parameter("gripper_command_topic").value) in topic_names
                    or gripper_age < 5.0
                ),
                "required": False,
                "message": f"status_age={gripper_age:.1f}s",
            },
        ]
        return checks

    def _check_topic(
        self, name: str, topic: str, topic_names: set[str], *, required: bool = True
    ) -> dict[str, Any]:
        return {
            "name": name,
            "ok": topic in topic_names,
            "required": required,
            "message": topic,
        }

    def _check_service(
        self, name: str, client: Any, *, timeout_sec: float, required: bool = True
    ) -> dict[str, Any]:
        service_name = getattr(client, "srv_name", name)
        ok = bool(client.wait_for_service(timeout_sec=timeout_sec))
        return {"name": name, "ok": ok, "required": required, "message": service_name}

    def _check_action(self, name: str) -> dict[str, Any]:
        action_name = str(self.get_parameter("aubo_move_joint_action_name").value)
        try:
            ok = bool(self.aubo_move_joint_client.wait_for_server(timeout_sec=0.05))
        except Exception:
            ok = False
        return {"name": name, "ok": ok, "required": True, "message": action_name}

    def _wait_task_preflight(self, client: Any, *, timeout_sec: float) -> tuple[bool, str]:
        service_name = getattr(client, "srv_name", "preflight")
        if not client.wait_for_service(timeout_sec=min(timeout_sec, 15.0)):
            return False, f"preflight unavailable: {service_name}"
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        last_message = ""
        while time.monotonic() < deadline:
            ok, message = self._call_trigger(client, "preflight", wait_timeout=5.0)
            if ok:
                return True, message
            last_message = message
            time.sleep(0.5)
        return False, last_message or f"preflight timeout: {service_name}"

    def _call_trigger(
        self,
        client: Any,
        label: str,
        wait_timeout: float,
        *,
        require_service: bool = True,
    ) -> tuple[bool, str]:
        service_name = getattr(client, "srv_name", label)
        if not client.wait_for_service(timeout_sec=wait_timeout):
            message = f"{label} unavailable: {service_name}"
            return (False, message) if require_service else (True, message)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(wait_timeout, 0.0)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, f"{label} timeout: {service_name}"
        response = future.result()
        if response is None:
            return False, f"{label} empty response"
        return bool(response.success), str(response.message)

    def _start_process(self, name: str) -> None:
        command_param = MANAGED_PROCESS_COMMAND_PARAMS[name]
        command = str(self.get_parameter(command_param).value).strip()
        if not command:
            self._set_error(f"service {name} has empty command")
            return
        with self.lock:
            existing = self.managed_processes.get(name)
            if existing is not None and existing.poll() is None:
                return
        ready_services = MANAGED_PROCESS_READY_SERVICES.get(name, ())
        ready_topic = MANAGED_PROCESS_READY_TOPICS.get(name, "")
        if ready_services and self._services_exist(ready_services) and self._topic_has_publisher(ready_topic):
            self.get_logger().info(f"service {name} already available")
            self._publish_state()
            return

        log_path = self.runtime_log_dir / f"{name}.log"
        shell_command = "\n".join(
            [
                "set -euo pipefail",
                f"cd {shlex.quote(str(self.workspace_root))}",
                "set +u",
                "source scripts/env/arachne_env.sh",
                "[[ -f install/setup.bash ]] && source install/setup.bash",
                "[[ -f scripts/env/arachne_real_defaults.sh ]] && source scripts/env/arachne_real_defaults.sh",
                "set -u",
                f"exec {command}",
            ]
        )
        handle = None
        try:
            handle = log_path.open("a", encoding="utf-8")
            handle.write(
                f"\n[{datetime.now().isoformat(timespec='seconds')}] start {name}: {command}\n"
            )
            handle.flush()
            process = subprocess.Popen(
                ["bash", "-lc", shell_command],
                cwd=str(self.workspace_root),
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except Exception as exc:
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass
            self._set_error(f"service {name} start failed: {exc}")
            return

        with self.lock:
            old_handle = self.managed_process_log_handles.pop(name, None)
            if old_handle is not None:
                try:
                    old_handle.close()
                except OSError:
                    pass
            self.managed_processes[name] = process
            self.managed_process_logs[name] = log_path
            self.managed_process_log_handles[name] = handle
        self.get_logger().info(f"service {name} started pid={process.pid}; log={log_path}")
        self._publish_state()

    def _services_exist(self, service_names: tuple[str, ...]) -> bool:
        available = {name for name, _types in self.get_service_names_and_types()}
        return all(name in available for name in service_names)

    def _topic_has_publisher(self, topic_name: str) -> bool:
        return bool(topic_name and self.get_publishers_info_by_topic(topic_name))

    def _stop_process(self, name: str, *, quiet: bool = False) -> None:
        timeout_sec = max(float(self.get_parameter("service_stop_timeout_sec").value), 0.5)
        with self.lock:
            process = self.managed_processes.get(name)
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except Exception as exc:
                    if not quiet:
                        self._set_error(f"service {name} terminate failed: {exc}")
            except ProcessLookupError:
                pass
            except Exception as exc:
                if not quiet:
                    self._set_error(f"service {name} stop failed: {exc}")
        result = process.poll()
        with self.lock:
            self.managed_processes.pop(name, None)
            handle = self.managed_process_log_handles.pop(name, None)
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass
        self.get_logger().info(f"service {name} stopped result={result}")
        self._publish_state()

    def _stop_all_processes(self, *, quiet: bool) -> None:
        for name in list(MANAGED_PROCESS_COMMAND_PARAMS):
            self._stop_process(name, quiet=quiet)

    def _stop_base(self) -> None:
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def _teach_off(self) -> None:
        msg = String()
        msg.data = "teach_off"
        self.aubo_teach_pub.publish(msg)
        time.sleep(max(float(self.get_parameter("aubo_teach_exit_settle_sec").value), 0.0))

    def _set_state(self, state: str) -> None:
        with self.lock:
            self.state = state
        self._publish_state()

    def _set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = str(message)
        self.get_logger().warning(str(message))
        self._publish_state()

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._state_payload(), separators=(",", ":"))
        self.state_pub.publish(msg)

    def _state_payload(self) -> dict[str, Any]:
        with self.lock:
            return {
                "state": self.state,
                "camera": self._managed_process_status_locked("camera"),
                "depth_pointcloud": self._managed_process_status_locked("depth_pointcloud"),
                "viewer": self._managed_process_status_locked("viewer"),
                "grasp_server": self._managed_process_status_locked("grasp_server"),
                "cleanup_server": self._managed_process_status_locked("cleanup_server"),
                "last_error": self.last_error,
            }

    def _managed_process_status_locked(self, name: str) -> str:
        process = self.managed_processes.get(name)
        if process is None:
            return "stopped"
        result = process.poll()
        if result is None:
            return f"running pid={process.pid}"
        return f"exited {result}"

    def _workspace_root(self) -> Path:
        raw = str(self.get_parameter("workspace_root").value).strip()
        if raw:
            return Path(raw).expanduser().resolve()
        for parent in Path(__file__).resolve().parents:
            if (parent / "scripts/env/arachne_env.sh").exists():
                return parent
        return Path.cwd().resolve()

    def _runtime_log_dir(self) -> Path:
        raw = str(self.get_parameter("runtime_log_root").value).strip()
        path = Path(raw or "log/demo_orchestrator")
        if not path.is_absolute():
            path = self.workspace_root / path
        return path

    def _parse_names(self, text: str) -> list[str]:
        return [part.strip() for part in str(text).split(",") if part.strip()]

    def _response(
        self, response: Trigger.Response, success: bool, message: str
    ) -> Trigger.Response:
        response.success = bool(success)
        response.message = str(message)
        return response


def main() -> None:
    rclpy.init()
    node = DemoOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
