from __future__ import annotations

import os
from pathlib import Path


def _looks_like_workspace(path: Path) -> bool:
    return (path / "src/arachne_operator").exists() or (path / "install/setup.bash").exists()


def _ancestor_workspace(path: Path) -> Path | None:
    for parent in (path, *path.parents):
        if _looks_like_workspace(parent):
            return parent
    return None


def root_dir() -> Path:
    env_root = os.environ.get("ARACHNE_ROOT_DIR", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    for start in (Path.cwd().resolve(), Path(__file__).resolve()):
        found = _ancestor_workspace(start)
        if found is not None:
            return found
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("arachne_operator")).resolve()
        found = _ancestor_workspace(share)
        if found is not None:
            return found
        return share
    except Exception:
        return Path.cwd().resolve()
