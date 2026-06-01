cmake_minimum_required(VERSION 3.8)
project(step_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp serial)


install(TARGETS
  motor_node

  DESTINATION lib/${PROJECT_NAME}
 )


 set(msg_files
 "msg/Motor.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)


rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
 
target_link_libraries(motor_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
