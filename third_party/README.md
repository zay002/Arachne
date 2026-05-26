# Third-Party Assets

Arachne uses upstream model packages for the real Scout 2.0 and Aubo i5 descriptions:

- `aubo_description`: cloned from `https://github.com/AuboRobot/aubo_description`
- `scout_ros2`: cloned from `https://github.com/agilexrobotics/scout_ros2`
- `MS42DC.step`: local source CAD for the current flexible gripper. The committed runtime mesh is generated from this file and stored in `src/arachne_description/meshes/gripper/ms42dc/`.

When licensed CAD, STL, SDK, or manual files are added later, keep their source, license, version, and checksum documented here. If a vendor file cannot be redistributed, store it outside the repository and document the expected local path in `docs/references.md`.
