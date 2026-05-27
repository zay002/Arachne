# Stage 5: Godot Showcase Frontend

## Summary

Arachne now has a separate Godot 4.x showcase frontend for high-FPS portfolio demos. It keeps Gazebo as the physics rehearsal backend, while Godot focuses on smooth third-person interaction: proportional Scout driving, an office-style initial map, collision-aware movement, pushable rigid-body props, follow camera damping, visual suspension, scene props, MS42DC gripper animation, and Aubo i5 preset pose interpolation.

## Core Files

- `godot/arachne_showcase/project.godot`: Godot 4.x project entry point.
- `godot/arachne_showcase/scenes/arachne_showcase.tscn`: main scene with one scripted root.
- `godot/arachne_showcase/scripts/arachne_showcase.gd`: builds the arena, loads robot meshes, handles keyboard/gamepad driving, camera follow, collision movement, body motion effects, gripper animation, arm presets, and headless self-test.
- `godot/arachne_showcase/scripts/ros2_bridge_placeholder.gd`: dependency-free memory/UDP placeholders for `/cmd_vel`, `/joint_states`, `/odom`, and `/tf`.
- `godot/arachne_showcase/tools/link_assets.sh`: links existing Scout, Aubo i5, MS42DC, AG95, and prop meshes into the Godot project, then converts DAE assets to local GLB cache files for Godot 4.
- `scripts/install_godot4.sh`: installs the pinned Godot 4.x Linux binary and `assimp-utils`.
- `scripts/godot_showcase.sh`: repository-level launcher.
- `scripts/test_godot_showcase.sh`: headless scene test for asset loading, scripted driving, camera distance, mesh count, and bridge message flow.

## Notes

The Godot demo uses a collision-aware character body and tuned arcade physics for a responsive third-person feel. It is not intended to replace Gazebo contact simulation, but it now has enough physical feedback for driving, obstacles, speed bumps, and portfolio capture. A later MuJoCo or ROS2 bridge can attach behind the placeholder interface without replacing the scene.

WSL2 currently uses Mesa D3D12 OpenGL through the launch script because Godot's Vulkan path may select CPU `llvmpipe`. Wheel animation is driven from measured base displacement and yaw delta, so collisions and speed limits remain visually synchronized. Extra Scout side-strip geometry was removed so the Godot robot stays closer to the Gazebo/URDF model.
