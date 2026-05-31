from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    confirm_motion = LaunchConfiguration("confirm_motion")
    run_base_test = LaunchConfiguration("run_base_test")
    run_arm_test = LaunchConfiguration("run_arm_test")
    run_gripper_test = LaunchConfiguration("run_gripper_test")
    sequence_mode = LaunchConfiguration("sequence_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("confirm_motion", default_value="false"),
            DeclareLaunchArgument("run_base_test", default_value="true"),
            DeclareLaunchArgument("run_arm_test", default_value="true"),
            DeclareLaunchArgument("run_gripper_test", default_value="true"),
            DeclareLaunchArgument("sequence_mode", default_value="parallel"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("arm_command_mode", default_value="topic"),
            DeclareLaunchArgument(
                "arm_follow_joint_trajectory_action",
                default_value="/aubo_arm_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_trajectory_topic",
                default_value="/aubo_arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "legacy_arm_trajectory_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_state_joint_names",
                default_value=(
                    "aubo_shoulder_joint,aubo_upperArm_joint,aubo_foreArm_joint,"
                    "aubo_wrist1_joint,aubo_wrist2_joint,aubo_wrist3_joint"
                ),
            ),
            DeclareLaunchArgument(
                "arm_command_joint_names",
                default_value=(
                    "aubo_shoulder_joint,aubo_upperArm_joint,aubo_foreArm_joint,"
                    "aubo_wrist1_joint,aubo_wrist2_joint,aubo_wrist3_joint"
                ),
            ),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument("base_distance_m", default_value="0.2"),
            DeclareLaunchArgument("base_linear_speed", default_value="0.06"),
            DeclareLaunchArgument("base_yaw_deg", default_value="30.0"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.22"),
            DeclareLaunchArgument("arm_z_delta_m", default_value="0.2"),
            DeclareLaunchArgument("arm_z_frame", default_value="aubo_base"),
            DeclareLaunchArgument("arm_duration_sec", default_value="4.0"),
            DeclareLaunchArgument("arm_max_joint_delta", default_value="1.0"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.03"),
            DeclareLaunchArgument("arm_goal_time_margin_sec", default_value="4.0"),
            DeclareLaunchArgument("arm_circle_radius_m", default_value="0.1"),
            DeclareLaunchArgument("arm_circle_points", default_value="32"),
            DeclareLaunchArgument("arm_circle_revolutions", default_value="1.0"),
            DeclareLaunchArgument("arm_circle_max_joint_delta", default_value="0.75"),
            DeclareLaunchArgument("gripper_cycles", default_value="5"),
            DeclareLaunchArgument("gripper_pause_sec", default_value="4.5"),
            DeclareLaunchArgument("gripper_final_state", default_value="open"),
            Node(
                package="arachne_operator",
                executable="real_hardware_acceptance_test",
                name="arachne_real_hardware_acceptance_test",
                output="screen",
                parameters=[
                    {
                        "confirm_motion": ParameterValue(confirm_motion, value_type=bool),
                        "run_base_test": ParameterValue(run_base_test, value_type=bool),
                        "run_arm_test": ParameterValue(run_arm_test, value_type=bool),
                        "run_gripper_test": ParameterValue(run_gripper_test, value_type=bool),
                        "sequence_mode": sequence_mode,
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        "arm_command_mode": LaunchConfiguration("arm_command_mode"),
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
                        "base_distance_m": ParameterValue(
                            LaunchConfiguration("base_distance_m"), value_type=float
                        ),
                        "base_linear_speed": ParameterValue(
                            LaunchConfiguration("base_linear_speed"), value_type=float
                        ),
                        "base_yaw_deg": ParameterValue(
                            LaunchConfiguration("base_yaw_deg"), value_type=float
                        ),
                        "base_angular_speed": ParameterValue(
                            LaunchConfiguration("base_angular_speed"), value_type=float
                        ),
                        "arm_z_delta_m": ParameterValue(
                            LaunchConfiguration("arm_z_delta_m"), value_type=float
                        ),
                        "arm_z_frame": LaunchConfiguration("arm_z_frame"),
                        "arm_duration_sec": ParameterValue(
                            LaunchConfiguration("arm_duration_sec"), value_type=float
                        ),
                        "arm_max_joint_delta": ParameterValue(
                            LaunchConfiguration("arm_max_joint_delta"), value_type=float
                        ),
                        "arm_goal_tolerance": ParameterValue(
                            LaunchConfiguration("arm_goal_tolerance"), value_type=float
                        ),
                        "arm_goal_time_margin_sec": ParameterValue(
                            LaunchConfiguration("arm_goal_time_margin_sec"), value_type=float
                        ),
                        "arm_circle_radius_m": ParameterValue(
                            LaunchConfiguration("arm_circle_radius_m"), value_type=float
                        ),
                        "arm_circle_points": ParameterValue(
                            LaunchConfiguration("arm_circle_points"), value_type=int
                        ),
                        "arm_circle_revolutions": ParameterValue(
                            LaunchConfiguration("arm_circle_revolutions"), value_type=float
                        ),
                        "arm_circle_max_joint_delta": ParameterValue(
                            LaunchConfiguration("arm_circle_max_joint_delta"), value_type=float
                        ),
                        "gripper_cycles": ParameterValue(
                            LaunchConfiguration("gripper_cycles"), value_type=int
                        ),
                        "gripper_pause_sec": ParameterValue(
                            LaunchConfiguration("gripper_pause_sec"), value_type=float
                        ),
                        "gripper_final_state": LaunchConfiguration("gripper_final_state"),
                    }
                ],
            ),
        ]
    )
