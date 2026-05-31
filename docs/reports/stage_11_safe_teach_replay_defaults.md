# Stage 11: Safe Teach Replay Defaults

## Goal

Keep teach recordings in a stable local project directory and make one-button replay conservative enough for real-hardware demonstrations.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: defaults recordings to `recordings/teach`, keeps base replay conservative, and stretches each arm waypoint to `6.0 s`.
- `src/arachne_operator/launch/teach_panel.launch.py`: mirrors the launch defaults.
- `scripts/teach_panel.sh` / `scripts/real_teach_demo.sh`: pass the absolute project-local recording path `${ROOT_DIR}/recordings/teach`, so recordings do not scatter when launched from different directories.

## File Relationships

The teach UI still commands only through `/cmd_vel`, the Aubo trajectory action, and `/arachne/gripper/command`. The scripts pin the local storage path; the UI handles saving, loading, and slow replay of JSON waypoints.
