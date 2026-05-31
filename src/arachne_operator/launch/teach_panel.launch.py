from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
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
            DeclareLaunchArgument("arm_jog_step_m", default_value="0.02"),
            DeclareLaunchArgument("arm_jog_duration_sec", default_value="1.2"),
            DeclareLaunchArgument("arm_rotate_step_rad", default_value="0.0872665"),
            DeclareLaunchArgument("arm_rotate_duration_sec", default_value="1.2"),
            DeclareLaunchArgument("arm_waypoint_duration_sec", default_value="6.0"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("aubo_teach_exit_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("recording_dir", default_value="recordings/teach"),
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
                        "arm_waypoint_duration_sec": ParameterValue(
                            LaunchConfiguration("arm_waypoint_duration_sec"), value_type=float
                        ),
                        "arm_goal_tolerance": ParameterValue(
                            LaunchConfiguration("arm_goal_tolerance"), value_type=float
                        ),
                        "recording_dir": LaunchConfiguration("recording_dir"),
                    }
                ],
            ),
        ]
    )
