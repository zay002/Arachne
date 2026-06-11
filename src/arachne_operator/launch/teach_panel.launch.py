from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_operator"), "launch", "teach_visualization.launch.py"]
            )
        ),
        launch_arguments={
            "input_joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "with_rviz": LaunchConfiguration("visualization_with_rviz"),
            "gripper_type": LaunchConfiguration("visualization_gripper_type"),
            "with_lidar": LaunchConfiguration("visualization_with_lidar"),
            "with_ee_camera": LaunchConfiguration("visualization_with_ee_camera"),
            "tool_adapter_xyz": LaunchConfiguration("visualization_tool_adapter_xyz"),
            "tool_adapter_rpy": LaunchConfiguration("visualization_tool_adapter_rpy"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_visualization")),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_sensors"), "launch", "gemini335.launch.py"]
            )
        ),
        launch_arguments={
            "color_device": LaunchConfiguration("camera_color_device"),
            "color_fourcc": LaunchConfiguration("camera_color_fourcc"),
            "depth_device": LaunchConfiguration("camera_depth_device"),
            "with_color_view": LaunchConfiguration("camera_with_color_view"),
            "with_depth_view": LaunchConfiguration("camera_with_depth_view"),
            "publish_depth": LaunchConfiguration("camera_publish_depth"),
            "publish_depth_color": LaunchConfiguration("camera_publish_depth_color"),
            "publish_pointcloud": LaunchConfiguration("camera_publish_pointcloud"),
            "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
            "projection_flip_x": LaunchConfiguration("camera_projection_flip_x"),
            "projection_flip_y": LaunchConfiguration("camera_projection_flip_y"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_camera")),
    )

    teach_panel_node = Node(
        package="arachne_operator",
        executable="teach_panel",
        name="arachne_teach_panel",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "arm_follow_joint_trajectory_action": LaunchConfiguration(
                    "arm_follow_joint_trajectory_action"
                ),
                "arm_trajectory_topic": LaunchConfiguration("arm_trajectory_topic"),
                "legacy_arm_trajectory_topic": LaunchConfiguration(
                    "legacy_arm_trajectory_topic"
                ),
                "arm_state_joint_names": LaunchConfiguration("arm_state_joint_names"),
                "arm_command_joint_names": LaunchConfiguration("arm_command_joint_names"),
                "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                "aubo_teach_command_topic": LaunchConfiguration(
                    "aubo_teach_command_topic"
                ),
                "aubo_teach_exit_wait_sec": ParameterValue(
                    LaunchConfiguration("aubo_teach_exit_wait_sec"), value_type=float
                ),
                "base_linear_speed": ParameterValue(
                    LaunchConfiguration("base_linear_speed"), value_type=float
                ),
                "base_angular_speed": ParameterValue(
                    LaunchConfiguration("base_angular_speed"), value_type=float
                ),
                "base_replay_linear_speed": ParameterValue(
                    LaunchConfiguration("base_replay_linear_speed"), value_type=float
                ),
                "base_replay_angular_speed": ParameterValue(
                    LaunchConfiguration("base_replay_angular_speed"), value_type=float
                ),
                "base_manual_publish_rate": ParameterValue(
                    LaunchConfiguration("base_manual_publish_rate"), value_type=float
                ),
                "base_motion_max_segment_sec": ParameterValue(
                    LaunchConfiguration("base_motion_max_segment_sec"), value_type=float
                ),
                "base_ignore_spurious_zero_odom": ParameterValue(
                    LaunchConfiguration("base_ignore_spurious_zero_odom"), value_type=bool
                ),
                "arm_jog_step_m": ParameterValue(
                    LaunchConfiguration("arm_jog_step_m"), value_type=float
                ),
                "arm_jog_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_jog_duration_sec"), value_type=float
                ),
                "arm_rotate_step_rad": ParameterValue(
                    LaunchConfiguration("arm_rotate_step_rad"), value_type=float
                ),
                "arm_rotate_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_rotate_duration_sec"), value_type=float
                ),
                "arm_joint_step_rad": ParameterValue(
                    LaunchConfiguration("arm_joint_step_rad"), value_type=float
                ),
                "arm_hold_period_sec": ParameterValue(
                    LaunchConfiguration("arm_hold_period_sec"), value_type=float
                ),
                "arm_manual_prefer_topic": ParameterValue(
                    LaunchConfiguration("arm_manual_prefer_topic"), value_type=bool
                ),
                "arm_velocity_command_topic": LaunchConfiguration(
                    "arm_velocity_command_topic"
                ),
                "arm_velocity_publish_rate": ParameterValue(
                    LaunchConfiguration("arm_velocity_publish_rate"), value_type=float
                ),
                "arm_velocity_watchdog_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_watchdog_sec"), value_type=float
                ),
                "arm_joint_jog_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_joint_jog_speed_rad_sec"), value_type=float
                ),
                "arm_cartesian_jog_speed_m_sec": ParameterValue(
                    LaunchConfiguration("arm_cartesian_jog_speed_m_sec"), value_type=float
                ),
                "arm_cartesian_rotate_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_cartesian_rotate_speed_rad_sec"), value_type=float
                ),
                "arm_velocity_damping": ParameterValue(
                    LaunchConfiguration("arm_velocity_damping"), value_type=float
                ),
                "arm_velocity_max_joint_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_speed_rad_sec"), value_type=float
                ),
                "arm_velocity_max_joint_accel_rad_sec2": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_accel_rad_sec2"), value_type=float
                ),
                "arm_velocity_max_joint_jerk_rad_sec3": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_jerk_rad_sec3"), value_type=float
                ),
                "arm_velocity_smoothing_tau_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_smoothing_tau_sec"), value_type=float
                ),
                "arm_velocity_keepout_predict_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_keepout_predict_sec"), value_type=float
                ),
                "arm_velocity_keepout_check_interval_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_keepout_check_interval_sec"), value_type=float
                ),
                "arm_velocity_stream_deadman_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_stream_deadman_sec"), value_type=float
                ),
                "arm_waypoint_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_waypoint_duration_sec"), value_type=float
                ),
                "arm_replay_backend": LaunchConfiguration("arm_replay_backend"),
                "aubo_sdk_ip": LaunchConfiguration("aubo_sdk_ip"),
                "aubo_sdk_rpc_port": ParameterValue(
                    LaunchConfiguration("aubo_sdk_rpc_port"), value_type=int
                ),
                "aubo_sdk_rpc_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_rpc_timeout_sec"), value_type=float
                ),
                "aubo_sdk_move_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_speed_rad_sec"), value_type=float
                ),
                "aubo_sdk_move_accel_rad_sec2": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_accel_rad_sec2"), value_type=float
                ),
                "aubo_sdk_blend_radius": ParameterValue(
                    LaunchConfiguration("aubo_sdk_blend_radius"), value_type=float
                ),
                "aubo_sdk_move_duration_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_duration_sec"), value_type=float
                ),
                "aubo_sdk_goal_tolerance_rad": ParameterValue(
                    LaunchConfiguration("aubo_sdk_goal_tolerance_rad"), value_type=float
                ),
                "aubo_sdk_arrival_timeout_padding_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_arrival_timeout_padding_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_power_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_power_timeout_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_startup_timeout_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_poll_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_poll_sec"), value_type=float
                ),
                "aubo_sdk_teach_flag_path": LaunchConfiguration("aubo_sdk_teach_flag_path"),
                "aubo_sdk_control_owner_path": LaunchConfiguration("aubo_sdk_control_owner_path"),
                "aubo_sdk_control_owner_name": LaunchConfiguration("aubo_sdk_control_owner_name"),
                "gripper_settle_sec": ParameterValue(
                    LaunchConfiguration("gripper_settle_sec"), value_type=float
                ),
                "arm_jog_position_tolerance": ParameterValue(
                    LaunchConfiguration("arm_jog_position_tolerance"), value_type=float
                ),
                "arm_orientation_tolerance": ParameterValue(
                    LaunchConfiguration("arm_orientation_tolerance"), value_type=float
                ),
                "arm_jog_orientation_tolerance": ParameterValue(
                    LaunchConfiguration("arm_jog_orientation_tolerance"), value_type=float
                ),
                "arm_keepout_enabled": ParameterValue(
                    LaunchConfiguration("arm_keepout_enabled"), value_type=bool
                ),
                "arm_base_xyz": LaunchConfiguration("arm_base_xyz"),
                "arm_base_rpy": LaunchConfiguration("arm_base_rpy"),
                "rear_rack_keepout_min_xyz": LaunchConfiguration(
                    "rear_rack_keepout_min_xyz"
                ),
                "rear_rack_keepout_max_xyz": LaunchConfiguration(
                    "rear_rack_keepout_max_xyz"
                ),
                "arm_keepout_sample_step_m": ParameterValue(
                    LaunchConfiguration("arm_keepout_sample_step_m"), value_type=float
                ),
                "arm_keepout_joint_step_rad": ParameterValue(
                    LaunchConfiguration("arm_keepout_joint_step_rad"), value_type=float
                ),
                "arm_home_joints_deg": LaunchConfiguration("arm_home_joints_deg"),
                "arm_install_joints_deg": LaunchConfiguration("arm_install_joints_deg"),
                "aubo_payload_mass_kg": ParameterValue(
                    LaunchConfiguration("aubo_payload_mass_kg"), value_type=float
                ),
                "aubo_payload_cog": LaunchConfiguration("aubo_payload_cog"),
                "aubo_payload_aom": LaunchConfiguration("aubo_payload_aom"),
                "aubo_payload_inertia": LaunchConfiguration("aubo_payload_inertia"),
                "arm_goal_tolerance": ParameterValue(
                    LaunchConfiguration("arm_goal_tolerance"), value_type=float
                ),
                "recording_dir": LaunchConfiguration("recording_dir"),
                "teach_config_path": LaunchConfiguration("teach_config_path"),
                "teach_config_autoload": ParameterValue(
                    LaunchConfiguration("teach_config_autoload"), value_type=bool
                ),
                "workspace_root": LaunchConfiguration("workspace_root"),
                "runtime_log_root": LaunchConfiguration("runtime_log_root"),
                "service_stop_timeout_sec": ParameterValue(
                    LaunchConfiguration("service_stop_timeout_sec"), value_type=float
                ),
                "camera_command": LaunchConfiguration("camera_command"),
                "camera_view_command": LaunchConfiguration("camera_view_command"),
                "slam_command": LaunchConfiguration("slam_command"),
                "grasp_server_command": LaunchConfiguration("grasp_server_command"),
                "grasp_task_state_topic": LaunchConfiguration("grasp_task_state_topic"),
                "grasp_task_start_service": LaunchConfiguration("grasp_task_start_service"),
                "grasp_task_stop_service": LaunchConfiguration("grasp_task_stop_service"),
                "grasp_task_restore_service": LaunchConfiguration("grasp_task_restore_service"),
                "grasp_task_status_service": LaunchConfiguration("grasp_task_status_service"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("with_visualization", default_value="true"),
            DeclareLaunchArgument("visualization_with_rviz", default_value="true"),
            DeclareLaunchArgument("visualization_gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("visualization_with_lidar", default_value="true"),
            DeclareLaunchArgument("visualization_with_ee_camera", default_value="true"),
            DeclareLaunchArgument("visualization_tool_adapter_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument(
                "visualization_tool_adapter_rpy",
                default_value="0.0 0.0 0.785398163397",
            ),
            DeclareLaunchArgument("with_camera", default_value="false"),
            DeclareLaunchArgument("camera_color_device", default_value="/dev/video6"),
            DeclareLaunchArgument("camera_color_fourcc", default_value="YUYV"),
            DeclareLaunchArgument("camera_depth_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_with_color_view", default_value="false"),
            DeclareLaunchArgument("camera_with_depth_view", default_value="false"),
            DeclareLaunchArgument("camera_publish_depth", default_value="false"),
            DeclareLaunchArgument("camera_publish_depth_color", default_value="false"),
            DeclareLaunchArgument("camera_publish_pointcloud", default_value="false"),
            DeclareLaunchArgument("camera_parent_frame", default_value="ee_camera_link"),
            DeclareLaunchArgument("camera_projection_flip_x", default_value="true"),
            DeclareLaunchArgument("camera_projection_flip_y", default_value="true"),
            DeclareLaunchArgument(
                "arm_follow_joint_trajectory_action",
                default_value="/joint_trajectory_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_trajectory_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "legacy_arm_trajectory_topic",
                default_value="/aubo_arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_state_joint_names",
                default_value=(
                    "shoulder_joint,upperArm_joint,foreArm_joint,"
                    "wrist1_joint,wrist2_joint,wrist3_joint"
                ),
            ),
            DeclareLaunchArgument(
                "arm_command_joint_names",
                default_value=(
                    "shoulder_joint,upperArm_joint,foreArm_joint,"
                    "wrist1_joint,wrist2_joint,wrist3_joint"
                ),
            ),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument(
                "aubo_teach_command_topic",
                default_value="/arachne/aubo/teach_command",
            ),
            DeclareLaunchArgument("base_linear_speed", default_value="0.16"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.60"),
            DeclareLaunchArgument("base_replay_linear_speed", default_value="0.40"),
            DeclareLaunchArgument("base_replay_angular_speed", default_value="0.48"),
            DeclareLaunchArgument("base_manual_publish_rate", default_value="12.0"),
            DeclareLaunchArgument("base_motion_max_segment_sec", default_value="20.0"),
            DeclareLaunchArgument("base_ignore_spurious_zero_odom", default_value="true"),
            DeclareLaunchArgument("arm_jog_step_m", default_value="0.008"),
            DeclareLaunchArgument("arm_jog_duration_sec", default_value="0.24"),
            DeclareLaunchArgument("arm_rotate_step_rad", default_value="0.0122173"),
            DeclareLaunchArgument("arm_rotate_duration_sec", default_value="0.24"),
            DeclareLaunchArgument("arm_joint_step_rad", default_value="0.00698132"),
            DeclareLaunchArgument("arm_hold_period_sec", default_value="0.10"),
            DeclareLaunchArgument("arm_manual_prefer_topic", default_value="true"),
            DeclareLaunchArgument(
                "arm_velocity_command_topic",
                default_value="/arachne/aubo/joint_velocity_command",
            ),
            DeclareLaunchArgument("arm_velocity_publish_rate", default_value="80.0"),
            DeclareLaunchArgument("arm_velocity_watchdog_sec", default_value="0.20"),
            DeclareLaunchArgument("arm_joint_jog_speed_rad_sec", default_value="0.08"),
            DeclareLaunchArgument("arm_cartesian_jog_speed_m_sec", default_value="0.025"),
            DeclareLaunchArgument("arm_cartesian_rotate_speed_rad_sec", default_value="0.08"),
            DeclareLaunchArgument("arm_velocity_damping", default_value="0.08"),
            DeclareLaunchArgument("arm_velocity_max_joint_speed_rad_sec", default_value="0.25"),
            DeclareLaunchArgument("arm_velocity_max_joint_accel_rad_sec2", default_value="1.60"),
            DeclareLaunchArgument("arm_velocity_max_joint_jerk_rad_sec3", default_value="24.0"),
            DeclareLaunchArgument("arm_velocity_smoothing_tau_sec", default_value="0.08"),
            DeclareLaunchArgument("arm_velocity_keepout_predict_sec", default_value="0.35"),
            DeclareLaunchArgument("arm_velocity_keepout_check_interval_sec", default_value="0.05"),
            DeclareLaunchArgument("arm_velocity_stream_deadman_sec", default_value="0.75"),
            DeclareLaunchArgument("arm_waypoint_duration_sec", default_value="3.75"),
            DeclareLaunchArgument("arm_replay_backend", default_value="sdk_move_joint"),
            DeclareLaunchArgument(
                "aubo_sdk_ip",
                default_value=EnvironmentVariable("AUBO_ROBOT_IP", default_value="192.168.127.128"),
            ),
            DeclareLaunchArgument("aubo_sdk_rpc_port", default_value="30004"),
            DeclareLaunchArgument("aubo_sdk_rpc_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("aubo_sdk_move_speed_rad_sec", default_value="0.25"),
            DeclareLaunchArgument("aubo_sdk_move_accel_rad_sec2", default_value="0.45"),
            DeclareLaunchArgument("aubo_sdk_blend_radius", default_value="0.0"),
            DeclareLaunchArgument("aubo_sdk_move_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("aubo_sdk_goal_tolerance_rad", default_value="0.04"),
            DeclareLaunchArgument("aubo_sdk_arrival_timeout_padding_sec", default_value="3.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_power_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_startup_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_poll_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "aubo_sdk_teach_flag_path",
                default_value="/tmp/arachne_aubo_teach_mode",
            ),
            DeclareLaunchArgument(
                "aubo_sdk_control_owner_path",
                default_value="/tmp/arachne_aubo_control_owner",
            ),
            DeclareLaunchArgument("aubo_sdk_control_owner_name", default_value="teach_panel"),
            DeclareLaunchArgument("gripper_settle_sec", default_value="2.0"),
            DeclareLaunchArgument("arm_jog_position_tolerance", default_value="0.0008"),
            DeclareLaunchArgument("arm_orientation_tolerance", default_value="0.01"),
            DeclareLaunchArgument("arm_jog_orientation_tolerance", default_value="0.004"),
            DeclareLaunchArgument("arm_keepout_enabled", default_value="true"),
            DeclareLaunchArgument("arm_base_xyz", default_value="0.22,0.0,0.155"),
            DeclareLaunchArgument("arm_base_rpy", default_value="0.0,0.0,1.57079632679"),
            DeclareLaunchArgument("rear_rack_keepout_min_xyz", default_value="-0.41,-0.22,0.04"),
            DeclareLaunchArgument("rear_rack_keepout_max_xyz", default_value="0.09,0.22,0.82"),
            DeclareLaunchArgument("arm_keepout_sample_step_m", default_value="0.035"),
            DeclareLaunchArgument("arm_keepout_joint_step_rad", default_value="0.06"),
            DeclareLaunchArgument(
                "arm_home_joints_deg",
                default_value="-90.00,11.55,95.09,27.80,96.07,43.79",
            ),
            DeclareLaunchArgument(
                "arm_install_joints_deg",
                default_value="-90.00,11.55,95.09,27.80,96.07,43.79",
            ),
            DeclareLaunchArgument("aubo_payload_mass_kg", default_value="0.818"),
            DeclareLaunchArgument("aubo_payload_cog", default_value="0.039927,0.045067,0.143233"),
            DeclareLaunchArgument("aubo_payload_aom", default_value="0,0,0"),
            DeclareLaunchArgument("aubo_payload_inertia", default_value="0,0,0,0,0,0"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("aubo_teach_exit_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("recording_dir", default_value="recordings/teach"),
            DeclareLaunchArgument(
                "teach_config_path",
                default_value="recordings/teach/teach_panel_config.json",
            ),
            DeclareLaunchArgument("teach_config_autoload", default_value="true"),
            DeclareLaunchArgument("workspace_root", default_value=""),
            DeclareLaunchArgument("runtime_log_root", default_value="log/teach_panel"),
            DeclareLaunchArgument("service_stop_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument(
                "camera_command",
                default_value=(
                    "ros2 launch arachne_sensors gemini335.launch.py "
                    "publish_pointcloud:=false with_color_view:=false with_depth_view:=false "
                    "with_tf:=true camera_parent_frame:=ee_camera_link "
                    "projection_flip_x:=true projection_flip_y:=true"
                ),
            ),
            DeclareLaunchArgument(
                "camera_view_command",
                default_value=(
                    "${ARACHNE_SYSTEM_PYTHON:-python3} scripts/vision/raw_image_viewer.py "
                    "--topic /camera/color/image_raw --window \"Arachne Raw Camera\" --max-fps 15"
                ),
            ),
            DeclareLaunchArgument("slam_command", default_value="scripts/hardware/real_lidar_nav.sh"),
            DeclareLaunchArgument(
                "grasp_server_command",
                default_value=(
                    "scripts/vision/grasp_task_server.sh "
                    "execute_real:=true confirm_execute_real:=true with_rviz:=false "
                    "preview_on_start:=false planning_recovery_base_enabled:=false "
                    "require_odom:=false require_camera_topics:=true "
                    "require_aubo_status:=false require_gripper_status:=false "
                    "max_grasp_attempts:=3 retry_on_gripper_miss:=true"
                ),
            ),
            DeclareLaunchArgument("grasp_task_state_topic", default_value="/arachne/grasp_task/state"),
            DeclareLaunchArgument("grasp_task_start_service", default_value="/arachne/grasp_task/start"),
            DeclareLaunchArgument("grasp_task_stop_service", default_value="/arachne/grasp_task/stop"),
            DeclareLaunchArgument(
                "grasp_task_restore_service", default_value="/arachne/grasp_task/restore"
            ),
            DeclareLaunchArgument("grasp_task_status_service", default_value="/arachne/grasp_task/status"),
            visualization,
            camera,
            teach_panel_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=teach_panel_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="teach panel closed"))],
                )
            ),
        ]
    )
