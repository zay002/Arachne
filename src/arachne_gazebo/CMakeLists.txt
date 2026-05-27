cmake_minimum_required(VERSION 3.16)
project(arachne_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(gazebo_camera_track_bridge src/gazebo_camera_track_bridge.cpp)
target_compile_features(gazebo_camera_track_bridge PRIVATE cxx_std_17)
ament_target_dependencies(gazebo_camera_track_bridge
  geometry_msgs
  rclcpp
)
target_link_libraries(gazebo_camera_track_bridge
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

add_executable(gazebo_demo_control_bridge src/gazebo_demo_control_bridge.cpp)
target_compile_features(gazebo_demo_control_bridge PRIVATE cxx_std_17)
ament_target_dependencies(gazebo_demo_control_bridge
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(gazebo_demo_control_bridge
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(
  TARGETS
    gazebo_camera_track_bridge
    gazebo_demo_control_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
