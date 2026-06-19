from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from .client import AuboDirectJsonRpc


DEFAULT_AUBO_TEACH_FLAG_PATH = "/tmp/arachne_aubo_teach_mode"


def set_teach_gate(path: Path, enabled: bool) -> None:
    if enabled:
        path.write_text("1\n", encoding="utf-8")
    else:
        clear_teach_gate(path)


def clear_teach_gate(path: Path) -> None:
    path.unlink(missing_ok=True)


def send_teach_rpc(rpc: AuboDirectJsonRpc, method: str, enabled: bool) -> Any:
    if method == "freedrive":
        return rpc.robot_call("RobotManage.freedrive", [enabled])
    if method == "backdrive":
        return rpc.robot_call("RobotManage.backdrive", [enabled])
    if method == "handguide":
        if enabled:
            return rpc.robot_call("RobotManage.handguideMode", [[], []])
        return rpc.robot_call("RobotManage.exitHandguideMode")
    raise RuntimeError(f"unsupported Aubo teach_method: {method}")


def read_teach_status(rpc: AuboDirectJsonRpc, method: str) -> Any:
    try:
        if method == "freedrive":
            return rpc.robot_call("RobotManage.isFreedriveEnabled")
        if method == "backdrive":
            return rpc.robot_call("RobotManage.isBackdriveEnabled")
        if method == "handguide":
            return rpc.robot_call("RobotManage.getHandguideStatus")
    except Exception as exc:
        return f"status unavailable: {exc}"
    return "unknown"


def teach_disabled(status: Any) -> bool:
    return status is False or str(status).strip().lower() in (
        "false",
        "0",
        "disabled",
        "off",
    )


def wait_teach_disabled(
    rpc: AuboDirectJsonRpc,
    method: str,
    timeout: float,
    poll: float,
) -> Any:
    from .safety import read_robot_state

    deadline = time.monotonic() + max(timeout, 0.0)
    poll = max(poll, 0.05)
    status: Any = "unknown"
    while time.monotonic() <= deadline:
        status = read_teach_status(rpc, method)
        mode, safety = read_robot_state(rpc)
        state_ready = mode == "running" and safety in ("normal", "reducedmode")
        if teach_disabled(status) and state_ready:
            return f"teach={status} mode={mode} safety={safety}"
        if isinstance(status, str) and status.startswith("status unavailable"):
            time.sleep(min(timeout, poll))
            status = f"{status}; mode={mode} safety={safety}"
        try:
            send_teach_rpc(rpc, method, False)
        except Exception as exc:
            status = f"disable retry failed: {exc}"
        time.sleep(poll)
    mode, safety = read_robot_state(rpc)
    raise TimeoutError(
        "teach mode did not return to Running/Normal before timeout; "
        f"last teach={status} mode={mode} safety={safety}"
    )
