# References

Primary references named by the development plan:

- Aubo description: https://github.com/AuboRobot/aubo_description
- Aubo ROS1 support: https://github.com/AuboRobot/aubo_robot
- Aubo ROS2 driver: https://github.com/AuboRobot/aubo_ros2_driver
- Scout ROS2 support: https://github.com/agilexrobotics/scout_ros2
- AgileX UGV SDK: https://github.com/agilexrobotics/ugv_sdk
- ROS2 control: https://github.com/ros-controls/ros2_control
- ROS2 controllers: https://github.com/ros-controls/ros2_controllers
- MoveIt2: https://github.com/moveit/moveit2
- Yizhua Robot MS42DC product information: http://www.yizhuarobot.com/
- MS42DC vendor ROS2 source: `third_party/MS42DC步进电机版柔性机械爪用户资料_V2.2_2024.08.28/5.ROS例程与教程/源码/ROS2.zip`
- Optional AG95 ROS2 description and driver: https://github.com/ian-chuang/dh_ag95_gripper_ros2

Third-party model and runtime assets are stored under `third_party/` and exposed to the ROS workspace through `src/vendor/` symlinks when they are ROS packages. The repository keeps the runnable subset needed for reproducibility, including the official Aubo URDF/xacro text and Aubo i5-family DAE/STL meshes; large manuals, videos, installers, and unrelated full asset packs are script- or link-downloaded.

## Third-Party Runtime Sources

- `third_party/aubo_description`: cloned from `AuboRobot/aubo_description` at `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`, package license declares BSD; official URDF/xacro files plus Aubo i5-family DAE/STL runtime meshes are committed so branches do not need locally rewritten Aubo geometry or joint definitions.
- `third_party/scout_ros2`: cloned from `agilexrobotics/scout_ros2` at `bdbb90471613831fb0b2ec01fecac043445313c4`, root license is Apache-2.0 and `scout_description/package.xml` declares BSD.
- `third_party/ugv_sdk`: cloned from `agilexrobotics/ugv_sdk` at `c3dfaf444f9bae10757e546acae055aaf4a13de7`, used by `scout_base` for CAN communication; the large `docs/` manuals are not committed.
- `third_party/aubo_ros2_driver`: cloned from `AuboRobot/aubo_ros2_driver` at `85684075d6ff06c5385e39611208e99ebf0f94c6`, used for official Aubo i5 TCP/IP and ros2_control integration.
- `third_party/dh_ag95_gripper_ros2`: cloned from `ian-chuang/dh_ag95_gripper_ros2` at `fc4f80fdfb3acae5626df4359aec1401cb71a9a3`; `dh_ag95_description/package.xml` declares Apache-2.0.
- `third_party/MS42DC.step`: local source CAD for the active Yizhua Robot MS42DC two-finger flexible servo gripper model.
- `third_party/MS42DC_SPLIT/*.stl`: user-created movable split MS42DC runtime parts copied into `src/arachne_description/meshes/gripper/ms42dc/split/`.
- `third_party/ms42dc_step_motor_ros2`: Yizhua Robot MS42DC vendor ROS2 example source; provides `serial`, `step_motor`, and demo keyboard packages. It can also be refreshed from the vendor ROS2 zip with `scripts/prepare_ms42dc_ros2.sh`.

The Arachne wrapper files in `src/arachne_description/urdf/` adapt these models into one mobile-manipulator tree. The Arachne hardware package uses the official/vendor ROS packages as runtime dependencies instead of copying their low-level protocols.
