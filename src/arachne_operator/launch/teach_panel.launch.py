from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            "depth_device": LaunchConfiguration("camera_depth_device"),
            "with_color_view": LaunchConfiguration("camera_with_color_view"),
            "with_depth_view": LaunchConfiguration("camera_with_depth_view"),
            "publish_pointcloud": LaunchConfiguration("camera_publish_pointcloud"),
            "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
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
                "arm_goal_tolerance": ParameterValue(
                    LaunchConfiguration("arm_goal_tolerance"), value_type=float
                ),
                "recording_dir": LaunchConfiguration("recording_dir"),
                "teach_config_path": LaunchConfiguration("teach_config_path"),
                "teach_config_autoload": ParameterValue(
                    LaunchConfiguration("teach_config_autoload"), value_type=bool
                ),
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
            DeclareLaunchArgument("with_camera", default_value="false"),
            DeclareLaunchArgument("camera_color_device", default_value="/dev/video6"),
            DeclareLaunchArgument("camera_depth_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_with_color_view", default_value="false"),
            DeclareLaunchArgument("camera_with_depth_view", default_value="false"),
            DeclareLaunchArgument("camera_publish_pointcloud", default_value="false"),
            DeclareLaunchArgument("camera_parent_frame", default_value="ee_camera_link"),
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
            DeclareLaunchArgument("base_linear_speed", default_value="0.08"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.30"),
            DeclareLaunchArgument("base_replay_linear_speed", default_value="0.20"),
            DeclareLaunchArgument("base_replay_angular_speed", default_value="0.24"),
            DeclareLaunchArgument("base_manual_publish_rate", default_value="12.0"),
            DeclareLaunchArgument("base_motion_max_segment_sec", default_value="20.0"),
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
                default_value="-88.28,3.40,116.60,103.48,88.33,-0.13",
            ),
            DeclareLaunchArgument(
                "arm_install_joints_deg",
                default_value="-88.28,3.40,116.60,103.48,88.33,-0.13",
            ),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("aubo_teach_exit_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("recording_dir", default_value="recordings/teach"),
            DeclareLaunchArgument(
                "teach_config_path",
                default_value="recordings/teach/teach_panel_config.json",
            ),
            DeclareLaunchArgument("teach_config_autoload", default_value="true"),
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
