from __future__ import annotations

import json
import os
import time
from pathlib import Path


DEFAULT_AUBO_CONTROL_OWNER_PATH = "/tmp/arachne_aubo_control_owner"


def control_owner_payload(owner: str) -> str:
    return json.dumps(
        {"owner": owner, "pid": os.getpid(), "created_at": time.time()},
        separators=(",", ":"),
    ) + "\n"


def parse_control_owner(text: str) -> tuple[str, int | None]:
    text = text.strip()
    if not text:
        return "", None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return text.splitlines()[0].strip(), None
    owner = str(data.get("owner", "")).strip()
    pid_value = data.get("pid")
    try:
        pid = int(pid_value) if pid_value is not None else None
    except (TypeError, ValueError):
        pid = None
    return owner, pid


def pid_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def claim_control_owner(path: Path, owner: str) -> tuple[bool, str]:
    owner = owner.strip() or "unknown"
    for _attempt in range(2):
        try:
            fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError as exc:
                return False, f"unreadable owner file {path}: {exc}"
            active_owner, pid = parse_control_owner(text)
            if active_owner == owner and pid == os.getpid():
                return True, f"already owned by {owner}"
            if pid is not None and not pid_alive(pid):
                try:
                    path.unlink(missing_ok=True)
                except OSError as exc:
                    return False, f"stale owner {active_owner or text!r} could not be cleared: {exc}"
                continue
            pid_text = str(pid) if pid is not None else "unknown"
            return False, f"owned by {active_owner or text.strip() or 'unknown'} pid={pid_text}"
        except OSError as exc:
            return False, f"could not create owner file {path}: {exc}"

        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(control_owner_payload(owner))
        except OSError as exc:
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass
            return False, f"could not write owner file {path}: {exc}"
        return True, f"owned by {owner}"
    return False, f"could not claim owner file {path}"


def release_control_owner(path: Path, owner: str) -> None:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except (FileNotFoundError, OSError):
        return
    active_owner, pid = parse_control_owner(text)
    if active_owner != (owner.strip() or "unknown"):
        return
    if pid is not None and pid != os.getpid():
        return
    try:
        path.unlink(missing_ok=True)
    except OSError:
        return
