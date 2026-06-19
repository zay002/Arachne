from __future__ import annotations

import math
import time
from typing import Callable

from .client import AuboDirectJsonRpc


StatusCallback = Callable[[str, bool], None]


def read_robot_state(rpc: AuboDirectJsonRpc) -> tuple[str, str]:
    try:
        mode = str(rpc.robot_call("RobotState.getRobotModeType")).strip().lower()
        safety = str(rpc.robot_call("RobotState.getSafetyModeType")).strip().lower()
        return mode, safety
    except Exception as exc:
        return "unknown", f"unknown:{exc}"


def require_running_normal(rpc: AuboDirectJsonRpc) -> None:
    mode, safety = read_robot_state(rpc)
    if mode != "running" or safety not in ("normal", "reducedmode"):
        raise RuntimeError(f"Aubo not ready: mode={mode} safety={safety}")


def stop_joint(
    rpc: AuboDirectJsonRpc,
    accel: float,
    reason: str,
    *,
    warn_only: bool = False,
    status: StatusCallback | None = None,
) -> None:
    try:
        result = rpc.robot_call("MotionControl.stopJoint", [max(float(accel), 0.05)])
    except Exception as exc:
        if warn_only:
            if status is not None:
                status(f"Aubo SDK stopJoint failed during {reason}: {exc}", True)
            return
        raise
    if result not in (0, None):
        message = f"Aubo SDK stopJoint result={result} during {reason}"
        if warn_only:
            if status is not None:
                status(message, True)
        else:
            raise RuntimeError(message)


def exit_servo_mode(
    rpc: AuboDirectJsonRpc,
    *,
    status: StatusCallback | None = None,
) -> None:
    try:
        result = rpc.robot_call("MotionControl.setServoModeSelect", [0])
        if result not in (0, None) and status is not None:
            status(f"Aubo SDK setServoModeSelect(0) result={result}", True)
        return
    except Exception as exc:
        if status is not None:
            status(
                f"Aubo SDK setServoModeSelect unavailable, trying setServoMode(false): {exc}",
                True,
            )
    result = rpc.robot_call("MotionControl.setServoMode", [False])
    if result not in (0, None) and status is not None:
        status(f"Aubo SDK setServoMode(false) result={result}", True)


def wait_exec_complete(
    rpc: AuboDirectJsonRpc,
    label: str,
    timeout: float,
    *,
    cancel_requested: Callable[[], bool] | None = None,
) -> None:
    deadline = time.monotonic() + max(float(timeout), 0.0)
    exec_id = rpc.robot_call("MotionControl.getExecId")
    start_deadline = time.monotonic() + 0.5
    while exec_id == -1 and time.monotonic() < start_deadline and not _cancelled(cancel_requested):
        time.sleep(0.05)
        exec_id = rpc.robot_call("MotionControl.getExecId")
    while exec_id != -1 and time.monotonic() < deadline and not _cancelled(cancel_requested):
        time.sleep(0.05)
        exec_id = rpc.robot_call("MotionControl.getExecId")
    if _cancelled(cancel_requested):
        raise RuntimeError(f"Aubo SDK moveJoint cancelled at {label}")
    if exec_id != -1:
        raise TimeoutError(f"Aubo SDK moveJoint exec timeout at {label}: exec_id={exec_id}")


def wait_arrival(
    rpc: AuboDirectJsonRpc,
    target: list[float],
    label: str,
    *,
    tolerance: float,
    speed: float,
    timeout_padding: float,
    stable_required: float = 0.25,
    cancel_requested: Callable[[], bool] | None = None,
    status: StatusCallback | None = None,
) -> bool:
    target_values = [float(value) for value in target]
    current = [float(value) for value in rpc.robot_call("RobotState.getJointPositions")]
    max_delta = max((abs(angle_diff(t, c)) for t, c in zip(target_values, current)), default=0.0)
    timeout = max(max_delta / max(float(speed), 0.01), 0.5) + max(float(timeout_padding), 0.0)
    deadline = time.monotonic() + timeout
    stable_since: float | None = None
    last_error = max_delta
    tolerance = max(float(tolerance), 0.001)
    while not _cancelled(cancel_requested) and time.monotonic() < deadline:
        current = [float(value) for value in rpc.robot_call("RobotState.getJointPositions")]
        last_error = max(
            (abs(angle_diff(t, c)) for t, c in zip(target_values, current)),
            default=0.0,
        )
        if last_error <= tolerance:
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_required:
                if status is not None:
                    status(f"Aubo SDK moveJoint reached: {label}", False)
                return True
        else:
            stable_since = None
        time.sleep(0.05)
    if status is not None:
        status(
            f"Aubo SDK moveJoint arrival timeout at {label}: "
            f"max_error={last_error:.3f}rad tolerance={tolerance:.3f}rad",
            True,
        )
    return False


def wait_mode(
    rpc: AuboDirectJsonRpc,
    expected: set[str],
    timeout_sec: float,
    poll_sec: float,
    label: str,
    *,
    cancel_requested: Callable[[], bool] | None = None,
    status: StatusCallback | None = None,
) -> str:
    deadline = time.monotonic() + max(float(timeout_sec), 0.0)
    last_mode = ""
    last_safety = ""
    while time.monotonic() < deadline and not _cancelled(cancel_requested):
        last_mode = str(rpc.robot_call("RobotState.getRobotModeType"))
        last_safety = str(rpc.robot_call("RobotState.getSafetyModeType"))
        if last_mode in expected:
            if status is not None:
                status(f"Aubo {label} reached {last_mode}", False)
            return last_mode
        time.sleep(max(float(poll_sec), 0.05))
    if _cancelled(cancel_requested):
        raise RuntimeError(f"Aubo {label} cancelled")
    raise TimeoutError(
        f"Aubo {label} timeout: mode={last_mode or 'unknown'} safety={last_safety or 'unknown'}"
    )


def angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _cancelled(cancel_requested: Callable[[], bool] | None) -> bool:
    return bool(cancel_requested is not None and cancel_requested())
