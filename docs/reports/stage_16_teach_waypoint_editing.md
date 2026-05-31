# Stage 16: Teach Waypoint Editing

## Goal

Make real-hardware demos easier to refine after recording by editing a single waypoint in place, and make arm playback slightly faster while staying conservative.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: adds `Update WP`, which overwrites one selected pose waypoint with the current robot state or updates a selected wait waypoint from `Wait s`.
- `src/arachne_operator/launch/teach_panel.launch.py`: exposes the faster defaults through launch arguments.
- `docs/control.md` / `docs/control.zh-CN.md`: document waypoint editing and the updated replay speeds.

## Relationship

The teach panel remains the single operator-facing editor. Record still appends new points, Duplicate still reuses old points, and Update WP now changes one selected point without rebuilding the full sequence.
