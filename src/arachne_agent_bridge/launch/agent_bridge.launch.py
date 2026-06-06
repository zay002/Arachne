from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("motion_enabled", default_value="false"),
            DeclareLaunchArgument("confirm_agent_motion", default_value="false"),
            DeclareLaunchArgument("allow_mode_change", default_value="false"),
            DeclareLaunchArgument("command_topic", default_value="/arachne/agent/command"),
            DeclareLaunchArgument("status_topic", default_value="/arachne/agent/status"),
            DeclareLaunchArgument("event_topic", default_value="/arachne/agent/event"),
            DeclareLaunchArgument("tools_topic", default_value="/arachne/agent/tools"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("arm_velocity_command_topic", default_value="/arachne/aubo/joint_velocity_command"),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument("base_task_command_topic", default_value="/arachne/grasp_task/base_command"),
            DeclareLaunchArgument("max_base_linear_x", default_value="0.20"),
            DeclareLaunchArgument("max_base_angular_z", default_value="0.35"),
            DeclareLaunchArgument("max_base_relative_distance_m", default_value="0.30"),
            DeclareLaunchArgument("max_base_relative_yaw_deg", default_value="30.0"),
            DeclareLaunchArgument("arm_cartesian_speed_m_sec", default_value="0.015"),
            DeclareLaunchArgument("arm_joint_speed_rad_sec", default_value="0.06"),
            DeclareLaunchArgument("max_arm_cartesian_step_m", default_value="0.03"),
            DeclareLaunchArgument("max_arm_joint_step_rad", default_value="0.12"),
            Node(
                package="arachne_agent_bridge",
                executable="agent_bridge",
                name="arachne_agent_bridge",
                output="screen",
                parameters=[
                    {
                        "motion_enabled": ParameterValue(
                            LaunchConfiguration("motion_enabled"), value_type=bool
                        ),
                        "confirm_agent_motion": ParameterValue(
                            LaunchConfiguration("confirm_agent_motion"), value_type=bool
                        ),
                        "allow_mode_change": ParameterValue(
                            LaunchConfiguration("allow_mode_change"), value_type=bool
                        ),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "event_topic": LaunchConfiguration("event_topic"),
                        "tools_topic": LaunchConfiguration("tools_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "arm_velocity_command_topic": LaunchConfiguration(
                            "arm_velocity_command_topic"
                        ),
                        "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                        "base_task_command_topic": LaunchConfiguration("base_task_command_topic"),
                        "max_base_linear_x": ParameterValue(
                            LaunchConfiguration("max_base_linear_x"), value_type=float
                        ),
                        "max_base_angular_z": ParameterValue(
                            LaunchConfiguration("max_base_angular_z"), value_type=float
                        ),
                        "max_base_relative_distance_m": ParameterValue(
                            LaunchConfiguration("max_base_relative_distance_m"),
                            value_type=float,
                        ),
                        "max_base_relative_yaw_deg": ParameterValue(
                            LaunchConfiguration("max_base_relative_yaw_deg"), value_type=float
                        ),
                        "arm_cartesian_speed_m_sec": ParameterValue(
                            LaunchConfiguration("arm_cartesian_speed_m_sec"), value_type=float
                        ),
                        "arm_joint_speed_rad_sec": ParameterValue(
                            LaunchConfiguration("arm_joint_speed_rad_sec"), value_type=float
                        ),
                        "max_arm_cartesian_step_m": ParameterValue(
                            LaunchConfiguration("max_arm_cartesian_step_m"), value_type=float
                        ),
                        "max_arm_joint_step_rad": ParameterValue(
                            LaunchConfiguration("max_arm_joint_step_rad"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
