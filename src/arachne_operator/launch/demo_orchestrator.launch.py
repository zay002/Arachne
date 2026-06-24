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
                    "confidence:=0.08 "
                    "real_fixed_post_grasp:=true "
                    "real_fixed_search_joints:=-1.72,-0.44,1.66,0.92,1.68,-0.05 "
                    "real_sdk_move_speed:=0.36 real_sdk_move_accel:=0.60 "
                    "extra_args:='--planner-backend local --imgsz 768 --planning-key-waypoints approach,grasp "
                    "--arm-collision-samples-per-link 1 --arm-collision-radius 0.018 "
                    "--collision-margin 0.0 --rear-rack-collision-margin 0.0 "
                    "--trajectory-max-duration 8 --max-grasp-orientation-candidates 1 "
                    "--local-planning-timeout-sec 1.8 --local-ik-max-iterations 80 "
                    "--grasp-orientation-yaw-offsets-deg 0 --grasp-orientation-tilt-offsets-deg 0 "
                    "--local-position-tolerance 0.050 --local-orientation-tolerance 0.55 "
                    "--real-sdk-max-targets 4 --real-sdk-semantic-targets-only' "
                    "preview_on_start:=true planning_recovery_base_enabled:=false "
                    "require_odom:=false require_camera_topics:=true "
                    "require_aubo_status:=false require_gripper_status:=false "
                    "max_grasp_attempts:=2 retry_on_gripper_miss:=true"
                ),
            ),
            DeclareLaunchArgument(
                "cleanup_server_command",
                default_value=(
                    "scripts/vision/road_cleanup_task_server.sh "
                    "patrol_pattern:=line patrol_distance_m:=1.5 patrol_step_m:=0.20 "
                    "max_round_trips:=1 loop:=false "
                    "patrol_base_speed_mps:=0.08 base_step_timeout_sec:=5.0 "
                    "grasp_timeout_sec:=25.0 "
                    "candidate_min_base_z_m:=-0.18 candidate_max_reach_m:=1.03 "
                    "reach_recovery_enabled:=false initial_detection_wait_sec:=0.0 "
                    "detection_confidence:=0.08 detection_timeout_sec:=3.0"
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
