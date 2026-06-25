from __future__ import annotations

import importlib
import subprocess
from pathlib import Path

from arachne_operator.process_utils import have, output


ENTRYPOINTS = (
    ("teach_panel", "teach_panel"),
    ("grasp_task_server", "grasp_task_server"),
    ("road_cleanup_task_server", "road_cleanup_task_server"),
    ("cli", "arachne"),
)


def entrypoint_main(module_name: str):
    module = importlib.import_module(f"arachne_operator.{module_name}")
    main_func = getattr(module, "main", None)
    if not callable(main_func):
        raise SystemExit(f"missing callable main(): arachne_operator.{module_name}")
    return main_func


def check_ros2_executable(root: Path, name: str) -> None:
    if not have("ros2"):
        print(f"[arachne check entrypoints] ros2 not found; skipped executable lookup for {name}")
        return
    executables = output(["ros2", "pkg", "executables", "arachne_operator"], cwd=root, stderr=subprocess.DEVNULL)
    if name not in executables.split():
        raise SystemExit(f"missing arachne_operator executable: {name}")


def check_entrypoints(root: Path) -> None:
    for module_name, executable in ENTRYPOINTS:
        entrypoint_main(module_name)
        check_ros2_executable(root, executable)


def smoke_teach_panel() -> None:
    entrypoint_main("teach_panel")(["--headless-check"])


def smoke_grasp_task() -> None:
    entrypoint_main("grasp_task_server")(["--dry-run-check"])


def smoke_road_cleanup() -> None:
    entrypoint_main("road_cleanup_task_server")(["--dry-run-check"])

