cmake_minimum_required(VERSION 3.8)
project(arachne_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AuboMoveJoint.action"
  DEPENDENCIES action_msgs
)

install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aubo_move_joint_action_server
  scripts/aubo_official_status_probe
  scripts/aubo_sdk_velocity_bridge
  scripts/aubo_teach_command_bridge
  scripts/hardware_mock
  scripts/ms42dc_direct_serial_driver
  scripts/ms42dc_official_bridge
  scripts/safety_cmd_vel_gate
  scripts/safety_state_machine
  scripts/scout_official_status_bridge
  scripts/scout_waveshare_serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
