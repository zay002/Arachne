from __future__ import annotations

import time
from typing import Callable

from .client import AuboDirectJsonRpc
from .safety import stop_joint


StatusCallback = Callable[[str, bool], None]


def speed_joint(
    rpc: AuboDirectJsonRpc,
    velocity: list[float],
    accel: float,
    duration: float,
    *,
    stop_accel: float,
    busy_retry_delay: float,
    status: StatusCallback | None = None,
) -> bool:
    try:
        result = rpc.robot_call("MotionControl.speedJoint", [velocity, accel, duration])
    except Exception:
        raise
    if result in (0, None):
        return True
    if result != 3:
        if status is not None:
            status(f"aubo sdk speedJoint result={result}", True)
        return False
    if status is not None:
        status("aubo sdk speedJoint busy; stopping previous motion and retrying", True)
    stop_joint(rpc, stop_accel, "speedJoint busy preempt", warn_only=True, status=status)
    retry_delay = max(float(busy_retry_delay), 0.0)
    if retry_delay > 0.0:
        time.sleep(retry_delay)
    retry_result = rpc.robot_call("MotionControl.speedJoint", [velocity, accel, duration])
    if retry_result in (0, None):
        return True
    if status is not None:
        status(f"aubo sdk speedJoint retry result={retry_result}", True)
    return False
