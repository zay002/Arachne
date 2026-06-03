from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
            DeclareLaunchArgument("arm_jog_duration_sec", default_value="0.11"),
            DeclareLaunchArgument("arm_rotate_step_rad", default_value="0.0122173"),
            DeclareLaunchArgument("arm_rotate_duration_sec", default_value="0.11"),
            DeclareLaunchArgument("arm_joint_step_rad", default_value="0.00698132"),
            DeclareLaunchArgument("arm_hold_period_sec", default_value="0.07"),
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
            Node(
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
            ),
        ]
    )
