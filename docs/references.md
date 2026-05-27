# References

Primary references named by the development plan:

- Aubo description: https://github.com/AuboRobot/aubo_description
- Aubo ROS support: https://github.com/AuboRobot/aubo_robot
- Scout ROS2 support: https://github.com/agilexrobotics/scout_ros2
- AgileX UGV SDK: https://github.com/agilexrobotics/ugv_sdk
- ROS2 control: https://github.com/ros-controls/ros2_control
- ROS2 controllers: https://github.com/ros-controls/ros2_controllers
- MoveIt2: https://github.com/moveit/moveit2
- Yizhua Robot MS42DC product information: http://www.yizhuarobot.com/
- Optional AG95 ROS2 description and driver: https://github.com/ian-chuang/dh_ag95_gripper_ros2

Third-party model assets are stored under `third_party/` and exposed to the ROS workspace through `src/vendor/` symlinks when they are ROS packages. Add future CAD, STL, SDK, or manual files only after checking their licenses.

## Downloaded Model Sources

- `third_party/aubo_description`: cloned from `AuboRobot/aubo_description` at `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`, package license declares BSD.
- `third_party/scout_ros2`: cloned from `agilexrobotics/scout_ros2` at `bdbb90471613831fb0b2ec01fecac043445313c4`, root license is Apache-2.0 and `scout_description/package.xml` declares BSD.
- `third_party/dh_ag95_gripper_ros2`: cloned from `ian-chuang/dh_ag95_gripper_ros2` at `fc4f80fdfb3acae5626df4359aec1401cb71a9a3`; `dh_ag95_description/package.xml` declares Apache-2.0.
- `third_party/MS42DC.step`: local source CAD for the active Yizhua Robot MS42DC two-finger flexible servo gripper model.
- `third_party/MS42DC_SPLIT/*.stl`: user-created movable split MS42DC runtime parts copied into `src/arachne_description/meshes/gripper/ms42dc/split/`.

The Arachne wrapper files in `src/arachne_description/urdf/` adapt these models into one mobile-manipulator tree.
