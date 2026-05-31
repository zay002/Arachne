# Stage 17: Teach Arm Speed Tuning

## Goal

Increase Aubo teach-panel motion speed by about 20% while keeping the same safe trajectory-controller path.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: reduces Aubo jog duration from `1.0 s` to `0.83 s` and replay waypoint duration from `4.5 s` to `3.75 s`.
- `src/arachne_operator/launch/teach_panel.launch.py`: exposes the same defaults for script and launch users.
- `docs/control.md` / `docs/control.zh-CN.md`: document the updated arm replay duration.

## Relationship

The change only adjusts trajectory timing. Waypoint recording, Teach On/Off handling, feedback verification, and base/gripper behavior remain unchanged.
