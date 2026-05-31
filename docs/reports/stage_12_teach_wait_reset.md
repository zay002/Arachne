# Stage 12: Teach Wait And Reset

## Goal

Add two common demo operations to the real teach panel: insert an N-second wait into the replay queue and reset the teach list and numbering with one button.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: `TeachWaypoint` now has `kind` and `wait_sec`; the UI adds `Add Wait` and `Reset`; replay sleeps on wait steps without sending motion commands.
- `docs/control.zh-CN.md` / `docs/control.md`: documents wait, clear, and reset behavior.

## File Relationships

Saved JSON still uses a single waypoint list. Older files without `kind` load as normal pose waypoints; new wait waypoints use `kind=wait` and `wait_sec`, so they can be mixed with normal robot-state waypoints.
