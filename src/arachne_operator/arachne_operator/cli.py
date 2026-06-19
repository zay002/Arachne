from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path


def root_dir() -> Path:
    path = Path.cwd()
    while path != path.parent:
        if (path / "scripts/env/arachne_env.sh").exists():
            return path
        path = path.parent
    return Path(__file__).resolve().parents[3]


ROOT = root_dir()


def run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    print("+", " ".join(cmd))
    return subprocess.run(cmd, cwd=ROOT, check=True, **kwargs)


def output(cmd: list[str], **kwargs) -> str:
    return subprocess.check_output(cmd, cwd=ROOT, text=True, **kwargs)


def have(name: str) -> bool:
    return shutil.which(name) is not None


def py() -> str:
    return os.environ.get("ARACHNE_SYSTEM_PYTHON", sys.executable)


def tcp_check(ip: str, port: int, timeout: float) -> None:
    with socket.create_connection((ip, port), timeout=timeout):
        print(f"{ip}:{port} open")


def ros_names(kind: str) -> set[str]:
    try:
        return set(output(["ros2", kind, "list"], stderr=subprocess.DEVNULL).splitlines())
    except Exception:
        return set()


def cmd_check_action_stack(_args: argparse.Namespace) -> int:
    for path in (
        "src/arachne_hardware/action/AuboMoveJoint.action",
        "src/arachne_hardware/arachne_hardware/aubo_move_joint_action_server.py",
        "src/arachne_operator/arachne_operator/aubo_move_joint_client.py",
        "src/arachne_operator/arachne_operator/demo_orchestrator.py",
    ):
        if not (ROOT / path).is_file():
            raise SystemExit(f"missing required file: {path}")
    run([py(), "-m", "compileall", "src/arachne_hardware/arachne_hardware", "src/arachne_operator/arachne_operator", "scripts/vision"])
    run(["ros2", "interface", "show", "arachne_hardware/action/AuboMoveJoint"], stdout=subprocess.DEVNULL)
    if "aubo_move_joint_action_server" not in output(["ros2", "pkg", "executables", "arachne_hardware"]):
        raise SystemExit("missing arachne_hardware aubo_move_joint_action_server executable")
    if "demo_orchestrator" not in output(["ros2", "pkg", "executables", "arachne_operator"]):
        raise SystemExit("missing arachne_operator demo_orchestrator executable")
    print("[arachne check aubo-action-stack] passed")
    return 0


def cmd_check_offline(_args: argparse.Namespace) -> int:
    print("[arachne check offline] offline only: no real hardware will be contacted")
    run([py(), "-m", "compileall", "src/arachne_hardware/arachne_hardware", "src/arachne_operator/arachne_operator", "scripts/vision"])
    for script in (
        "scripts/build/check_aubo_action_stack.sh",
        "scripts/hardware/check_aubo_readonly.sh",
        "scripts/operator/teach_panel.sh",
        "scripts/hardware/real_bringup.sh",
        "scripts/hardware/real_teach_demo.sh",
        "scripts/vision/grasp_task_server.sh",
        "scripts/vision/road_cleanup_task_server.sh",
        "scripts/vision/grasp_preview_real_sync.sh",
    ):
        run(["bash", "-n", script])
    run(["./scripts/build/check_workspace.sh"])
    if have("ros2") and have("colcon"):
        run(["colcon", "build", "--base-paths", "src", "--packages-select", "arachne_hardware", "arachne_operator"])
    else:
        print("[arachne check offline] ros2/colcon not found; skipped selected package build")
    print("[arachne check offline] passed")
    return 0


def readonly_probe(ip: str, port: int, timeout: float) -> str:
    tcp_check(ip, port, timeout)
    return output([py(), "scripts/hardware/real_aubo_probe.py", "--ip", ip, "--timeout", str(timeout), "--ports", str(port)])


def cmd_check_aubo_readonly(_args: argparse.Namespace) -> int:
    ip = os.environ.get("AUBO_ROBOT_IP", "192.168.127.128")
    port = int(os.environ.get("AUBO_RPC_PORT", "30004"))
    timeout = float(os.environ.get("AUBO_PROBE_TIMEOUT_SEC", "1.0"))
    print("[arachne check aubo-readonly] no motion commands will be sent")
    run(["bash", "-lc", "ls -l /tmp/arachne_aubo_teach_mode /tmp/arachne_aubo_control_owner 2>/dev/null || true"])
    run(["ping", "-c", "2", "-W", "1", ip])
    print(readonly_probe(ip, port, timeout), end="")
    run(["ros2", "interface", "show", "arachne_hardware/action/AuboMoveJoint"], stdout=subprocess.DEVNULL)
    topics = ros_names("topic")
    actions = ros_names("action")
    print("/joint_states", "present" if "/joint_states" in topics else "not present")
    print("/arachne/hardware/aubo_status", "present" if "/arachne/hardware/aubo_status" in topics else "not present")
    print("/arachne/aubo/move_joint", "action present" if "/arachne/aubo/move_joint" in actions else "action not present")
    print("[arachne check aubo-readonly] completed")
    return 0


def cmd_check_aubo_running(_args: argparse.Namespace) -> int:
    ip = os.environ.get("AUBO_ROBOT_IP", "192.168.127.128")
    port = int(os.environ.get("AUBO_RPC_PORT", "30004"))
    timeout = float(os.environ.get("AUBO_PROBE_TIMEOUT_SEC", "1.0"))
    print("[arachne check aubo-running] read-only only: no goals, no owner claim, no teach gate writes")
    run(["bash", "-lc", "ls -l /tmp/arachne_aubo_teach_mode /tmp/arachne_aubo_control_owner 2>/dev/null || true"])
    probe = readonly_probe(ip, port, timeout)
    print(probe, end="")
    mode = next((line.rsplit(": ", 1)[-1] for line in probe.splitlines() if "getRobotModeType" in line), "unknown")
    safety = next((line.rsplit(": ", 1)[-1] for line in probe.splitlines() if "getSafetyModeType" in line), "unknown")
    print(f"RobotMode={mode}")
    print(f"SafetyMode={safety}")
    topics = ros_names("topic")
    actions = ros_names("action")
    print("/joint_states", "present" if "/joint_states" in topics else "not present")
    print("/arachne/hardware/aubo_status", "present" if "/arachne/hardware/aubo_status" in topics else "not present")
    if "/arachne/aubo/move_joint" in actions:
        print("/arachne/aubo/move_joint action present")
        run(["ros2", "action", "info", "/arachne/aubo/move_joint"])
    else:
        print("/arachne/aubo/move_joint action not present")
    print("[arachne check aubo-running] completed. No motion commands were sent.")
    return 0


def cmd_smoke_aubo_dry_run(_args: argparse.Namespace) -> int:
    print("DRY RUN ONLY: this command must not connect to real Aubo or send real motion.")
    log_dir = ROOT / "log/offline_smoke"
    log_dir.mkdir(parents=True, exist_ok=True)
    server_log = (log_dir / "aubo_move_joint_dry_run_server.log").open("w")
    proc = subprocess.Popen(
        [
            "ros2",
            "run",
            "arachne_hardware",
            "aubo_move_joint_action_server",
            "--ros-args",
            "-p",
            "dry_run:=true",
            "-p",
            "action_name:=/arachne/aubo/move_joint",
        ],
        cwd=ROOT,
        stdout=server_log,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        for _ in range(40):
            if "/arachne/aubo/move_joint" in ros_names("action"):
                break
            time.sleep(0.25)
        else:
            raise SystemExit("dry-run action server did not become available")
        goal = "{target_joints: [0,0,0,0,0,0], speed_rad_sec: 0.1, accel_rad_sec2: 0.1, blend_radius: 0.0, duration_sec: 0.0, goal_tolerance_rad: 0.04, timeout_sec: 3.0, label: 'dry_run_smoke_test'}"
        result = output(["timeout", "15s", "ros2", "action", "send_goal", "/arachne/aubo/move_joint", "arachne_hardware/action/AuboMoveJoint", goal], stderr=subprocess.STDOUT)
        if "success=True" not in result and "success: true" not in result and "success=True" not in result.replace(" ", ""):
            raise SystemExit(f"dry-run goal failed:\n{result}")
        if "dry-run completed" not in result:
            raise SystemExit(f"dry-run message missing:\n{result}")
        print("[arachne smoke aubo-dry-run] passed")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        server_log.close()


def cmd_smoke_demo(_args: argparse.Namespace) -> int:
    print("OFFLINE ONLY: this command must not call start_visual_grasp or start_road_cleanup.")
    log_dir = ROOT / "log/offline_smoke"
    log_dir.mkdir(parents=True, exist_ok=True)
    orch_log = (log_dir / "demo_orchestrator_offline.log").open("w")
    proc = subprocess.Popen(
        ["ros2", "launch", "arachne_operator", "demo_orchestrator.launch.py", "autostart:=false"],
        cwd=ROOT,
        stdout=orch_log,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        for _ in range(40):
            if "/arachne/demo/status" in ros_names("service"):
                break
            time.sleep(0.25)
        else:
            raise SystemExit("demo_orchestrator status service did not become available")
        if "/arachne/demo/state" not in ros_names("topic"):
            raise SystemExit("/arachne/demo/state topic is not present")
        status = output(["timeout", "10s", "ros2", "service", "call", "/arachne/demo/status", "std_srvs/srv/Trigger", "{}"], stderr=subprocess.STDOUT)
        preflight = output(["timeout", "10s", "ros2", "service", "call", "/arachne/demo/preflight", "std_srvs/srv/Trigger", "{}"], stderr=subprocess.STDOUT)
        if "success=True" not in status and "success: true" not in status:
            raise SystemExit(f"demo status failed:\n{status}")
        if "checks" not in preflight:
            raise SystemExit(f"demo preflight response missing checks:\n{preflight}")
        print("[arachne smoke demo-orchestrator] passed")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        orch_log.close()


def cmd_clean_logs(_args: argparse.Namespace) -> int:
    for name in ("build", "install", "log", ".pytest_cache", ".mypy_cache", ".ruff_cache"):
        path = ROOT / name
        if path.exists():
            print(f"remove {name}/")
            shutil.rmtree(path)
    for path in ROOT.rglob("__pycache__"):
        shutil.rmtree(path, ignore_errors=True)
    for pattern in ("*.pyc", "*.pyo"):
        for path in ROOT.rglob(pattern):
            path.unlink(missing_ok=True)
    return 0


def cmd_status(_args: argparse.Namespace) -> int:
    run(["git", "status", "--short"])
    print("topics:", ", ".join(sorted(name for name in ros_names("topic") if name.startswith("/arachne") or name in {"/joint_states", "/cmd_vel"})))
    print("actions:", ", ".join(sorted(name for name in ros_names("action") if name.startswith("/arachne"))))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="arachne")
    sub = parser.add_subparsers(dest="group", required=True)

    check = sub.add_parser("check")
    check_sub = check.add_subparsers(dest="command", required=True)
    check_sub.add_parser("offline").set_defaults(func=cmd_check_offline)
    check_sub.add_parser("aubo-action-stack").set_defaults(func=cmd_check_action_stack)
    check_sub.add_parser("aubo-readonly").set_defaults(func=cmd_check_aubo_readonly)
    check_sub.add_parser("aubo-running").set_defaults(func=cmd_check_aubo_running)

    smoke = sub.add_parser("smoke")
    smoke_sub = smoke.add_subparsers(dest="command", required=True)
    smoke_sub.add_parser("aubo-dry-run").set_defaults(func=cmd_smoke_aubo_dry_run)
    smoke_sub.add_parser("demo-orchestrator").set_defaults(func=cmd_smoke_demo)

    clean = sub.add_parser("clean")
    clean_sub = clean.add_subparsers(dest="command", required=True)
    clean_sub.add_parser("logs").set_defaults(func=cmd_clean_logs)

    sub.add_parser("status").set_defaults(func=cmd_status)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
