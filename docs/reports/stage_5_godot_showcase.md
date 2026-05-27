# Stage 5: Godot Showcase Frontend

## Summary

Arachne now has a separate Godot 4.x showcase frontend for high-FPS portfolio demos. It keeps Gazebo as the physics rehearsal backend, while Godot focuses on smooth visual interaction: Scout-style driving, a follow camera, simple scene props, MS42DC gripper animation, and Aubo i5 preset pose interpolation.

## Core Files

- `godot/arachne_showcase/project.godot`: Godot 4.x project entry point.
- `godot/arachne_showcase/scenes/arachne_showcase.tscn`: main scene with one scripted root.
- `godot/arachne_showcase/scripts/arachne_showcase.gd`: builds the world, loads robot meshes, handles keyboard/gamepad driving, camera follow, gripper animation, and arm presets.
- `godot/arachne_showcase/scripts/ros2_bridge_placeholder.gd`: dependency-free placeholders for `/cmd_vel`, `/joint_states`, `/odom`, and `/tf`.
- `godot/arachne_showcase/tools/link_assets.sh`: links existing Scout, Aubo i5, MS42DC, AG95, and prop meshes into the Godot project, then converts DAE assets to local GLB cache files for Godot 4.
- `scripts/install_godot4.sh`: installs the pinned Godot 4.x Linux binary and `assimp-utils`.
- `scripts/godot_showcase.sh`: repository-level launcher.

## Notes

The Godot demo intentionally uses simple kinematic motion and visual interpolation. It is a modern interactive frontend for demos and outreach, not a contact-accurate simulator. A later MuJoCo or ROS 2 bridge can attach behind the placeholder interface without replacing the scene.
