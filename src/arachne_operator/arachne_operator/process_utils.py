from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], *, cwd: Path, **kwargs) -> subprocess.CompletedProcess:
    print("+", " ".join(cmd))
    return subprocess.run(cmd, cwd=cwd, check=True, **kwargs)


def output(cmd: list[str], *, cwd: Path, **kwargs) -> str:
    return subprocess.check_output(cmd, cwd=cwd, text=True, **kwargs)


def have(name: str) -> bool:
    return shutil.which(name) is not None


def py() -> str:
    return os.environ.get("ARACHNE_SYSTEM_PYTHON", sys.executable)

