# Third-Party Assets

Arachne keeps a minimal third-party subset in `third_party/` so the workspace can build and run demos directly. Large reference material and full asset packs stay out of git and are downloaded from scripts or source links.

## Vendored In Git

- `aubo_description`: official Aubo description package metadata, full URDF/xacro text, and Aubo i5-family DAE/STL runtime meshes from `AuboRobot/aubo_description`. This keeps the Aubo i5 dimensions and joint definitions canonical across desktop and Jetson branches.
- `scout_ros2`: Scout 2.0 ROS2 description, messages, and base node from `agilexrobotics/scout_ros2`.
- `ugv_sdk`: AgileX UGV SDK source and build files, without the large `docs/` manuals.
- `aubo_ros2_driver`: Aubo ROS2 driver with the Arachne real-arm safe-start patches.
- `dh_ag95_gripper_ros2`: optional AG95 gripper description and driver.
- `ms42dc_step_motor_ros2`: Yizhua Robot MS42DC vendor ROS2 example source.
- `MS42DC.step` and `MS42DC_SPLIT/*.stl`: source assets for the active MS42DC movable model; the split STL parts were prepared manually by the project author.

## Downloaded Locally

- Large non-i5 Aubo mesh families, UGV PDF manuals, vendor videos/installers, the `kenney` Godot asset pack, optional `LARA_AUBOi5_AG95` assets, and ROS1 `scout_ros` are not committed.
- To refresh full pinned upstream checkouts:

```bash
ARACHNE_REFRESH_THIRD_PARTY=true ./scripts/fetch_third_party.sh
```

- To fetch the Godot office assets:

```bash
./scripts/fetch_godot_assets.sh
```

When adding CAD, STL, SDK, or manual files later, record the source, license, version, and checksum. If a vendor file cannot be redistributed, document how to obtain it instead of committing it.
