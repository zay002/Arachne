from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    status_topic = LaunchConfiguration("status_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    arm_trajectory_topic = LaunchConfiguration("arm_trajectory_topic")
    legacy_arm_trajectory_topic = LaunchConfiguration("legacy_arm_trajectory_topic")
    gripper_command_topic = LaunchConfiguration("gripper_command_topic")
    array_arm_mode = LaunchConfiguration("array_arm_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/arachne/vla/action_chunk"),
            DeclareLaunchArgument("status_topic", default_value="/arachne/vla/translator/status"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "arm_trajectory_topic",
                default_value="/aubo_arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "legacy_arm_trajectory_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument("array_arm_mode", default_value="delta"),
            Node(
                package="arachne_operator",
                executable="action_chunk_translator",
                name="arachne_action_chunk_translator",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "status_topic": status_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "arm_trajectory_topic": arm_trajectory_topic,
                        "legacy_arm_trajectory_topic": legacy_arm_trajectory_topic,
                        "gripper_command_topic": gripper_command_topic,
                        "array_arm_mode": array_arm_mode,
                    }
                ],
            )
        ]
    )
