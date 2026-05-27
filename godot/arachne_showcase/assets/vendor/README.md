# Vendor Mesh Links

This directory is populated by `tools/link_assets.sh` or `scripts/godot_showcase.sh`.

The links point to existing Arachne assets instead of copying large third-party meshes:

- `scout`: Scout 2.0 chassis and wheel meshes from `third_party/scout_ros2`.
- `aubo_i5`: Aubo i5 visual links from `third_party/aubo_description`.
- `ms42dc`: user-created movable MS42DC split STL parts from `src/arachne_description`.
- `ag95`: optional AG95 visual meshes from `third_party/dh_ag95_gripper_ros2`.
- `props`: simple object meshes from `third_party/LARA_AUBOi5_AG95` when available.

DAE assets are also converted into local GLB cache files under `assets/generated/` when `assimp` is available. Both the symlinks and generated cache are ignored by git.
