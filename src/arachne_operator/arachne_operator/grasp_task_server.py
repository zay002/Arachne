from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


AUBO_JOINT_ALIASES = (
    ("shoulder_joint", ("shoulder_joint", "aubo_shoulder_joint")),
    ("upperArm_joint", ("upperArm_joint", "aubo_upperArm_joint", "upper_arm_joint")),
    ("foreArm_joint", ("foreArm_joint", "aubo_foreArm_joint", "fore_arm_joint")),
    ("wrist1_joint", ("wrist1_joint", "aubo_wrist1_joint")),
    ("wrist2_joint", ("wrist2_joint", "aubo_wrist2_joint")),
    ("wrist3_joint", ("wrist3_joint", "aubo_wrist3_joint")),
)


TERMINAL_STATES = {"succeeded", "failed", "canceled"}


@dataclass
class PreflightResult:
    ok: bool
    checks: list[dict[str, Any]] = field(default_factory=list)

    def add(self, name: str, ok: bool, message: str, *, required: bool = True) -> None:
        self.checks.append(
            {
                "name": name,
                "ok": bool(ok),
                "required": bool(required),
                "message": message,
            }
        )
        if required and not ok:
            self.ok = False


@dataclass
class TaskSnapshot:
    task_id: str
    state: str
    message: str
    run_dir: str
    started_at: str
    finished_at: str
    returncode: int | None
    grasp_preview_log_dir: str
    preflight: list[dict[str, Any]]


class GraspTaskServer(Node):
    """Guarded wrapper around the existing grasp_preview real execution demo.

    This node intentionally reuses scripts/vision/grasp_preview_real_sync.sh
    instead of duplicating detection, MoveIt planning, and Aubo SDK execution.
    It adds a task-level state machine, preflight checks, process control, and a
    stable log bundle per run.
    """

    def __init__(self) -> None:
        super().__init__("arachne_grasp_task_server")

        self.declare_parameter("workspace_root", "")
        self.declare_parameter("runner_script", "scripts/vision/grasp_preview_real_sync.sh")
        self.declare_parameter("log_root", "log/grasp_tasks")
        self.declare_parameter("execute_real", False)
        self.declare_parameter("confirm_execute_real", False)
        self.declare_parameter("with_rviz", False)
        self.declare_parameter("classes", "bottle")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device_id", 0)
        self.declare_parameter("real_execute_backend", "sdk_move_joint")
        self.declare_parameter("real_return_home", True)
        self.declare_parameter("real_sdk_move_speed", 0.25)
        self.declare_parameter("real_sdk_move_accel", 0.45)
        self.declare_parameter("grasp_base_offset", "0.06,0.09,-0.10")
        self.declare_parameter("extra_args", "")
        self.declare_parameter("preflight_timeout_sec", 2.0)
        self.declare_parameter("status_publish_period_sec", 0.5)
        self.declare_parameter("require_safety_state_machine", False)
        self.declare_parameter("set_safety_autonomous_on_start", True)
        self.declare_parameter("set_safety_manual_on_finish", True)
        self.declare_parameter("require_aubo_status", True)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("require_gripper_status", True)
        self.declare_parameter("require_odom", False)
        self.declare_parameter("require_camera_topics", False)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("aubo_status_topic", "/arachne/hardware/aubo_status")
        self.declare_parameter("gripper_status_topic", "/arachne/hardware/gripper_status")
        self.declare_parameter("base_status_topic", "/arachne/hardware/base_status")
        self.declare_parameter("safety_state_topic", "/arachne/safety/state")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")

        self.root = self._resolve_workspace_root()
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.process: subprocess.Popen[str] | None = None
        self.worker: threading.Thread | None = None
        self.latest: dict[str, tuple[float, Any]] = {}
        self.state = "idle"
        self.message = "ready"
        self.task_id = ""
        self.run_dir: Path | None = None
        self.started_at = ""
        self.finished_at = ""
        self.returncode: int | None = None
        self.preflight_checks: list[dict[str, Any]] = []
        self.grasp_preview_log_dir = ""

        self.state_pub = self.create_publisher(String, "/arachne/grasp_task/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/grasp_task/event", 10)
        self.create_service(Trigger, "/arachne/grasp_task/start", self._start_cb)
        self.create_service(Trigger, "/arachne/grasp_task/cancel", self._cancel_cb)
        self.create_service(Trigger, "/arachne/grasp_task/status", self._status_cb)
        self.create_service(Trigger, "/arachne/grasp_task/preflight", self._preflight_cb)

        self.set_autonomous_client = self.create_client(
            Trigger, "/arachne/safety/set_autonomous"
        )
        self.set_manual_client = self.create_client(Trigger, "/arachne/safety/set_manual")

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._cache_joint_states,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            lambda msg: self._cache("odom", msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("aubo_status_topic").value),
            lambda msg: self._cache("aubo_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_status_topic").value),
            lambda msg: self._cache("gripper_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_status_topic").value),
            lambda msg: self._cache("base_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("safety_state_topic").value),
            lambda msg: self._cache("safety_state", msg.data),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            lambda msg: self._cache("color_image", msg.header.stamp),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            lambda msg: self._cache("depth_image", msg.header.stamp),
            10,
        )

        period = max(float(self.get_parameter("status_publish_period_sec").value), 0.1)
        self.create_timer(period, self._publish_state)
        self._event("server_ready", {"workspace_root": str(self.root)})

    def _resolve_workspace_root(self) -> Path:
        configured = str(self.get_parameter("workspace_root").value).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        env_root = os.environ.get("ARACHNE_ROOT_DIR", "").strip()
        if env_root:
            return Path(env_root).expanduser().resolve()
        cwd = Path.cwd().resolve()
        if (cwd / "scripts" / "vision" / "grasp_preview_real_sync.sh").exists():
            return cwd
        return Path(__file__).resolve().parents[3]

    def _cache(self, key: str, value: Any) -> None:
        with self.lock:
            self.latest[key] = (time.monotonic(), value)

    def _cache_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        found: dict[str, float] = {}
        for canonical, aliases in AUBO_JOINT_ALIASES:
            for alias in aliases:
                if alias in positions:
                    found[canonical] = float(positions[alias])
                    break
        with self.lock:
            self.latest["joint_states"] = (time.monotonic(), msg)
            if len(found) == len(AUBO_JOINT_ALIASES):
                self.latest["aubo_joints"] = (time.monotonic(), found)

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

    def _cancel_cb(self, _request, response):
        self.cancel_event.set()
        self._terminate_process("cancel requested")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request, response):
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request, response):
        result = self._run_preflight(set_autonomous=False)
        with self.lock:
            self.preflight_checks = result.checks
        response.success = result.ok
        response.message = json.dumps(
            {"ok": result.ok, "checks": result.checks},
            ensure_ascii=False,
            sort_keys=True,
        )
        return response

    def _run_task(self) -> None:
        task_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        run_dir = self._log_root() / task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.task_id = task_id
            self.run_dir = run_dir
            self.started_at = datetime.now().isoformat(timespec="seconds")
            self.finished_at = ""
            self.returncode = None
            self.grasp_preview_log_dir = ""
            self.preflight_checks = []
        self._set_state("preflight", "checking real-hardware readiness")
        self._write_json(run_dir / "task_request.json", self._task_request())

        result = self._run_preflight(
            set_autonomous=bool(self.get_parameter("set_safety_autonomous_on_start").value)
        )
        with self.lock:
            self.preflight_checks = result.checks
        self._write_json(run_dir / "preflight.json", {"ok": result.ok, "checks": result.checks})
        if not result.ok:
            self._finish_task("failed", "preflight failed", returncode=None)
            return

        command, env = self._runner_command()
        self._write_json(
            run_dir / "runner.json",
            {
                "command": command,
                "environment_overrides": self._logged_environment(env),
            },
        )
        self._set_state("running", "grasp_preview pipeline started")
        process_log = run_dir / "process.log"
        try:
            with process_log.open("w", encoding="utf-8") as log_file:
                self.process = subprocess.Popen(
                    command,
                    cwd=str(self.root),
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
                if self.process.stdout is not None:
                    for line in self.process.stdout:
                        log_file.write(line)
                        log_file.flush()
                        self._parse_process_line(line)
                        if self.cancel_event.is_set():
                            self._terminate_process("cancel requested")
                returncode = self.process.wait()
        except FileNotFoundError as exc:
            self._event("runner_error", {"error": str(exc)})
            self._finish_task("failed", f"runner missing: {exc}", returncode=None)
            return
        except Exception as exc:  # pragma: no cover - defensive around live process I/O.
            self._event("runner_error", {"error": str(exc)})
            self._finish_task("failed", f"runner error: {exc}", returncode=None)
            return
        finally:
            self.process = None

        if self.cancel_event.is_set():
            self._finish_task("canceled", "task canceled", returncode=returncode)
        elif returncode == 0:
            self._finish_task("succeeded", "grasp task complete", returncode=returncode)
        else:
            self._finish_task(
                "failed",
                f"grasp task failed with return code {returncode}",
                returncode=returncode,
            )

    def _run_preflight(self, *, set_autonomous: bool) -> PreflightResult:
        timeout = max(float(self.get_parameter("preflight_timeout_sec").value), 0.1)
        result = PreflightResult(ok=True)

        runner = self._runner_path()
        result.add("workspace_root", self.root.exists(), str(self.root), required=True)
        result.add("runner_script", runner.exists() and os.access(runner, os.X_OK), str(runner), required=True)
        result.add(
            "install_setup",
            (self.root / "install" / "setup.bash").exists(),
            str(self.root / "install" / "setup.bash"),
            required=True,
        )
        execute_real = bool(self.get_parameter("execute_real").value)
        confirmed = bool(self.get_parameter("confirm_execute_real").value)
        result.add(
            "real_execution_confirmation",
            (not execute_real) or confirmed,
            "execute_real requires confirm_execute_real:=true",
            required=True,
        )

        if set_autonomous:
            required = bool(self.get_parameter("require_safety_state_machine").value)
            ok, message = self._call_trigger(self.set_autonomous_client, timeout)
            result.add("safety_set_autonomous", ok, message, required=required)

        self._wait_for_fresh_inputs(timeout)
        self._add_cached_check(
            result,
            "aubo_status",
            required=bool(self.get_parameter("require_aubo_status").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: "not reachable" not in str(value).lower(),
        )
        self._add_cached_check(
            result,
            "aubo_joints",
            required=bool(self.get_parameter("require_joint_states").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "gripper_status",
            required=bool(self.get_parameter("require_gripper_status").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "odom",
            required=bool(self.get_parameter("require_odom").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "safety_state",
            required=bool(self.get_parameter("require_safety_state_machine").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: not str(value).lower().startswith(("disabled", "estop", "fault")),
        )
        if bool(self.get_parameter("require_camera_topics").value):
            self._add_cached_check(result, "color_image", required=True, max_age=max(timeout + 1.0, 2.0))
            self._add_cached_check(result, "depth_image", required=True, max_age=max(timeout + 1.0, 2.0))
        return result

    def _wait_for_fresh_inputs(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest:
                    return
            time.sleep(0.05)

    def _add_cached_check(
        self,
        result: PreflightResult,
        name: str,
        *,
        required: bool,
        max_age: float,
        validator=None,
    ) -> None:
        with self.lock:
            item = self.latest.get(name)
        if item is None:
            result.add(name, False, "no recent message", required=required)
            return
        stamp, value = item
        age = time.monotonic() - stamp
        ok = age <= max_age
        message = f"age={age:.2f}s value={self._short_value(value)}"
        if ok and validator is not None:
            try:
                ok = bool(validator(value))
            except Exception as exc:
                ok = False
                message += f" validator_error={exc}"
        result.add(name, ok, message, required=required)

    def _call_trigger(self, client, timeout: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"service unavailable: {getattr(client, 'srv_name', 'trigger')}"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return False, f"service timeout: {getattr(client, 'srv_name', 'trigger')}"
        response = future.result()
        if response is None:
            return False, f"empty response: {getattr(client, 'srv_name', 'trigger')}"
        return bool(response.success), str(response.message)

    def _runner_command(self) -> tuple[list[str], dict[str, str]]:
        command = [str(self._runner_path())]
        if bool(self.get_parameter("execute_real").value):
            command.append("--execute-real")
        extra = shlex.split(str(self.get_parameter("extra_args").value))
        if extra:
            command.append("--")
            command.extend(extra)

        env = os.environ.copy()
        env["ARACHNE_GRASP_WITH_RVIZ"] = "true" if bool(self.get_parameter("with_rviz").value) else "false"
        env["ARACHNE_GRASP_CLASSES"] = str(self.get_parameter("classes").value)
        env["ARACHNE_GRASP_CONF"] = str(self.get_parameter("confidence").value)
        env["ARACHNE_GRASP_DEVICE_ID"] = str(self.get_parameter("device_id").value)
        env["ARACHNE_GRASP_REAL_EXECUTE_BACKEND"] = str(
            self.get_parameter("real_execute_backend").value
        )
        env["ARACHNE_GRASP_REAL_RETURN_HOME"] = (
            "true" if bool(self.get_parameter("real_return_home").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_SPEED"] = str(
            self.get_parameter("real_sdk_move_speed").value
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL"] = str(
            self.get_parameter("real_sdk_move_accel").value
        )
        env["ARACHNE_GRASP_BASE_OFFSET"] = str(self.get_parameter("grasp_base_offset").value)
        if bool(self.get_parameter("confirm_execute_real").value):
            env["ARACHNE_CONFIRM_GRASP_EXECUTE_REAL"] = "YES"
        return command, env

    def _runner_path(self) -> Path:
        configured = Path(str(self.get_parameter("runner_script").value))
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _log_root(self) -> Path:
        configured = Path(str(self.get_parameter("log_root").value)).expanduser()
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _task_request(self) -> dict[str, Any]:
        names = (
            "execute_real",
            "confirm_execute_real",
            "with_rviz",
            "classes",
            "confidence",
            "device_id",
            "real_execute_backend",
            "real_return_home",
            "real_sdk_move_speed",
            "real_sdk_move_accel",
            "grasp_base_offset",
            "extra_args",
        )
        return {name: self.get_parameter(name).value for name in names}

    def _logged_environment(self, env: dict[str, str]) -> dict[str, str]:
        keys = sorted(key for key in env if key.startswith("ARACHNE_GRASP_"))
        keys.append("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL")
        return {key: env.get(key, "") for key in keys}

    def _parse_process_line(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return
        match = re.search(r"Grasp preview logs:\s*(.+)$", stripped)
        if match:
            with self.lock:
                self.grasp_preview_log_dir = match.group(1).strip()
            self._event("grasp_preview_log_dir", {"path": match.group(1).strip()})
        if "REAL arm SDK moveJoint sequence complete" in stripped:
            self._event("arm_sequence_complete", {"line": stripped})
        elif "REAL execution is armed" in stripped:
            self._event("real_execution_armed", {"line": stripped})
        elif "grasp task failed" in stripped.lower() or "failed" in stripped.lower():
            self._event("runner_warning", {"line": stripped})

    def _finish_task(self, state: str, message: str, *, returncode: int | None) -> None:
        with self.lock:
            self.returncode = returncode
            self.finished_at = datetime.now().isoformat(timespec="seconds")
        if bool(self.get_parameter("set_safety_manual_on_finish").value):
            self._call_trigger(self.set_manual_client, timeout=0.8)
        self._set_state(state, message)
        run_dir = self.run_dir
        if run_dir is not None:
            self._write_json(run_dir / "summary.json", asdict(self._snapshot()))

    def _terminate_process(self, reason: str) -> None:
        with self.lock:
            process = self.process
        if process is None or process.poll() is not None:
            return
        self._event("terminate", {"reason": reason})
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().warning(f"SIGINT failed: {exc}")
        deadline = time.monotonic() + 3.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            except Exception as exc:
                self.get_logger().warning(f"SIGTERM failed: {exc}")

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

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
        run_dir = self.run_dir
        if run_dir is not None:
            try:
                with (run_dir / "events.jsonl").open("a", encoding="utf-8") as handle:
                    handle.write(text + "\n")
            except OSError as exc:
                self.get_logger().warning(f"event log write failed: {exc}")
        if kind == "state":
            self.get_logger().info(f"{data.get('state')}: {data.get('message')}")

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._snapshot_json()
        self.state_pub.publish(msg)

    def _snapshot(self) -> TaskSnapshot:
        with self.lock:
            return TaskSnapshot(
                task_id=self.task_id,
                state=self.state,
                message=self.message,
                run_dir=str(self.run_dir or ""),
                started_at=self.started_at,
                finished_at=self.finished_at,
                returncode=self.returncode,
                grasp_preview_log_dir=self.grasp_preview_log_dir,
                preflight=list(self.preflight_checks),
            )

    def _snapshot_json(self) -> str:
        return json.dumps(asdict(self._snapshot()), ensure_ascii=False, sort_keys=True)

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _short_value(self, value: Any) -> str:
        if isinstance(value, dict):
            return json.dumps(value, ensure_ascii=False, sort_keys=True)[:240]
        text = str(value)
        return text if len(text) <= 240 else text[:237] + "..."


def main() -> None:
    rclpy.init()
    node = GraspTaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_event.set()
        node._terminate_process("shutdown")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
