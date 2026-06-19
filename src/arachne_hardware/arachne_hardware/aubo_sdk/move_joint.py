from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .client import AuboDirectJsonRpc
from .ownership import claim_control_owner, release_control_owner
from .safety import (
    angle_diff,
    exit_servo_mode,
    require_running_normal,
    stop_joint,
    wait_arrival,
    wait_exec_complete,
)
from .teach import clear_teach_gate, set_teach_gate


StatusCallback = Callable[[str, bool], None]
ProgressCallback = Callable[[str, float], None]


@dataclass(frozen=True)
class MoveJointConfig:
    ip: str
    port: int
    rpc_timeout: float
    speed: float
    accel: float
    blend_radius: float
    duration: float
    tolerance: float
    exec_timeout: float
    arrival_timeout_padding: float
    owner_path: Path
    owner_name: str
    teach_flag_path: Path
    gate_settle_sec: float = 0.15


def execute_move_joint(
    target: list[float],
    label: str,
    config: MoveJointConfig,
    *,
    cancel_requested: Callable[[], bool] | None = None,
    progress: ProgressCallback | None = None,
    status: StatusCallback | None = None,
) -> bool:
    owner_owned = False
    gate_owned = False
    target_values = [float(value) for value in target]

    try:
        _progress(progress, "checking_state")
        with AuboDirectJsonRpc(config.ip, config.port, config.rpc_timeout) as rpc:
            require_running_normal(rpc)
            ok, message = claim_control_owner(config.owner_path, config.owner_name)
            if not ok:
                raise RuntimeError(f"Aubo control owner unavailable: {message}")
            owner_owned = True
            _progress(progress, "owner_claimed")
            set_teach_gate(config.teach_flag_path, True)
            gate_owned = True
            _progress(progress, "gate_entered")

            if config.gate_settle_sec > 0.0:
                import time

                time.sleep(config.gate_settle_sec)
            exit_servo_mode(rpc, status=status)
            stop_joint(rpc, config.accel, "pre-move cleanup", warn_only=True, status=status)
            _progress(progress, "motion_started")
            result = rpc.robot_call(
                "MotionControl.moveJoint",
                [
                    target_values,
                    max(float(config.accel), 0.05),
                    max(float(config.speed), 0.01),
                    max(float(config.blend_radius), 0.0),
                    max(float(config.duration), 0.0),
                ],
            )
            if result not in (0, None):
                if status is not None:
                    status(f"Aubo SDK moveJoint failed at {label}: result={result}", True)
                return False
            wait_exec_complete(
                rpc,
                label,
                max(float(config.exec_timeout), 0.5),
                cancel_requested=cancel_requested,
            )
            _progress(progress, "waiting_arrival")
            return wait_arrival(
                rpc,
                target_values,
                label,
                tolerance=config.tolerance,
                speed=config.speed,
                timeout_padding=config.arrival_timeout_padding,
                cancel_requested=cancel_requested,
                status=status,
            )
    finally:
        if gate_owned:
            try:
                with AuboDirectJsonRpc(config.ip, config.port, config.rpc_timeout) as rpc:
                    stop_joint(
                        rpc,
                        config.accel,
                        "post-move cleanup",
                        warn_only=True,
                        status=status,
                    )
            except Exception:
                pass
        if gate_owned:
            try:
                clear_teach_gate(config.teach_flag_path)
            except OSError as exc:
                if status is not None:
                    status(f"failed to release Aubo teach gate {config.teach_flag_path}: {exc}", True)
        if owner_owned:
            release_control_owner(config.owner_path, config.owner_name)


def read_final_error(config: MoveJointConfig, target: list[float]) -> float:
    target_values = [float(value) for value in target]
    with AuboDirectJsonRpc(config.ip, config.port, config.rpc_timeout) as rpc:
        current = [float(value) for value in rpc.robot_call("RobotState.getJointPositions")]
    return max(
        (abs(angle_diff(target_value, current_value)) for target_value, current_value in zip(target_values, current)),
        default=0.0,
    )


def _progress(progress: ProgressCallback | None, state: str, max_error: float = -1.0) -> None:
    if progress is not None:
        progress(state, max_error)
