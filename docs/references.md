# References

Primary references named by the development plan:

- Aubo description: https://github.com/AuboRobot/aubo_description
- Aubo ROS support: https://github.com/AuboRobot/aubo_robot
- Scout ROS2 support: https://github.com/agilexrobotics/scout_ros2
- AgileX UGV SDK: https://github.com/agilexrobotics/ugv_sdk
- ROS2 control: https://github.com/ros-controls/ros2_control
- ROS2 controllers: https://github.com/ros-controls/ros2_controllers
- MoveIt2: https://github.com/moveit/moveit2
- MS42DC product information: https://www.youyeetoo.com/blog/detail/youyeetoo-soft-flexible-robot-gripper-claw-whdpakmg0020-whdpakmg0026-206
- DH Robotics AG95 ROS2 description and driver: https://github.com/ian-chuang/dh_ag95_gripper_ros2

Third-party model assets are stored under `third_party/` and exposed to the ROS workspace through `src/vendor/` symlinks. Add future CAD, STL, SDK, or manual files only after checking their licenses.

## Downloaded Model Sources

- `third_party/aubo_description`: cloned from `AuboRobot/aubo_description`, package license declares BSD.
- `third_party/scout_ros2`: cloned from `agilexrobotics/scout_ros2`, root license is Apache-2.0 and `scout_description/package.xml` declares BSD.
- `third_party/dh_ag95_gripper_ros2`: cloned from `ian-chuang/dh_ag95_gripper_ros2`; `dh_ag95_description/package.xml` declares Apache-2.0.

The Arachne wrapper files in `src/arachne_description/urdf/*/*_vendor.xacro` adapt these models into one mobile-manipulator tree.
