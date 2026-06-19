from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument("gripper_status_topic", default_value="/arachne/gripper/status"),
            DeclareLaunchArgument("aubo_teach_command_topic", default_value="/arachne/aubo/teach_command"),
            DeclareLaunchArgument("aubo_move_joint_action_name", default_value="/arachne/aubo/move_joint"),
            DeclareLaunchArgument("camera_color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("camera_depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("grasp_task_start_service", default_value="/arachne/grasp_task/start"),
            DeclareLaunchArgument("grasp_task_stop_service", default_value="/arachne/grasp_task/stop"),
            DeclareLaunchArgument(
                "grasp_task_preflight_service", default_value="/arachne/grasp_task/preflight"
            ),
            DeclareLaunchArgument("cleanup_task_start_service", default_value="/arachne/road_cleanup/start"),
            DeclareLaunchArgument("cleanup_task_stop_service", default_value="/arachne/road_cleanup/stop"),
            DeclareLaunchArgument(
                "cleanup_task_preflight_service",
                default_value="/arachne/road_cleanup/preflight",
            ),
            DeclareLaunchArgument(
                "camera_command",
                default_value=(
                    "ros2 launch arachne_sensors gemini335.launch.py "
                    "publish_pointcloud:=false with_color_view:=false with_depth_view:=false "
                    "with_tf:=true camera_parent_frame:=ee_camera_link "
                    "projection_flip_x:=true projection_flip_y:=true color_yuv_layout:=YVYU"
                ),
            ),
            DeclareLaunchArgument(
                "camera_view_command",
                default_value=(
                    "${ARACHNE_SYSTEM_PYTHON:-python3} scripts/vision/raw_image_viewer.py "
                    "--topic /camera/color/image_raw --window \"Arachne Raw Camera\" --max-fps 15"
                ),
            ),
            DeclareLaunchArgument(
                "grasp_server_command",
                default_value=(
                    "scripts/vision/grasp_task_server.sh "
                    "execute_real:=true confirm_execute_real:=true with_rviz:=false "
                    "preview_on_start:=true planning_recovery_base_enabled:=false "
                    "require_odom:=false require_camera_topics:=true "
                    "require_aubo_status:=false require_gripper_status:=false "
                    "max_grasp_attempts:=3 retry_on_gripper_miss:=true"
                ),
            ),
            DeclareLaunchArgument(
                "cleanup_server_command",
                default_value=(
                    "scripts/vision/road_cleanup_task_server.sh "
                    "patrol_distance_m:=1.2 patrol_step_m:=1.2 max_round_trips:=2 "
                    "detection_confidence:=0.35 loop:=true"
                ),
            ),
            DeclareLaunchArgument("service_stop_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument("runtime_log_root", default_value="log/demo_orchestrator"),
            DeclareLaunchArgument("workspace_root", default_value=""),
            Node(
                package="arachne_operator",
                executable="demo_orchestrator",
                name="demo_orchestrator",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                        "gripper_status_topic": LaunchConfiguration("gripper_status_topic"),
                        "aubo_teach_command_topic": LaunchConfiguration("aubo_teach_command_topic"),
                        "aubo_move_joint_action_name": LaunchConfiguration(
                            "aubo_move_joint_action_name"
                        ),
                        "camera_color_topic": LaunchConfiguration("camera_color_topic"),
                        "camera_depth_topic": LaunchConfiguration("camera_depth_topic"),
                        "grasp_task_start_service": LaunchConfiguration("grasp_task_start_service"),
                        "grasp_task_stop_service": LaunchConfiguration("grasp_task_stop_service"),
                        "grasp_task_preflight_service": LaunchConfiguration(
                            "grasp_task_preflight_service"
                        ),
                        "cleanup_task_start_service": LaunchConfiguration(
                            "cleanup_task_start_service"
                        ),
                        "cleanup_task_stop_service": LaunchConfiguration(
                            "cleanup_task_stop_service"
                        ),
                        "cleanup_task_preflight_service": LaunchConfiguration(
                            "cleanup_task_preflight_service"
                        ),
                        "camera_command": LaunchConfiguration("camera_command"),
                        "camera_view_command": LaunchConfiguration("camera_view_command"),
                        "grasp_server_command": LaunchConfiguration("grasp_server_command"),
                        "cleanup_server_command": LaunchConfiguration("cleanup_server_command"),
                        "service_stop_timeout_sec": ParameterValue(
                            LaunchConfiguration("service_stop_timeout_sec"), value_type=float
                        ),
                        "runtime_log_root": LaunchConfiguration("runtime_log_root"),
                        "workspace_root": LaunchConfiguration("workspace_root"),
                        "autostart": ParameterValue(
                            LaunchConfiguration("autostart"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
