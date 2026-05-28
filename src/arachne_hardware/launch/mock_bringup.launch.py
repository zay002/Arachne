from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_safety_gate", default_value="false"),
            Node(
                package="arachne_hardware",
                executable="safety_state_machine",
                name="arachne_safety_state_machine",
                output="screen",
            ),
            Node(
                package="arachne_hardware",
                executable="hardware_mock",
                name="arachne_hardware_mock",
                output="screen",
            ),
            Node(
                package="arachne_hardware",
                executable="safety_cmd_vel_gate",
                name="arachne_safety_cmd_vel_gate",
                condition=IfCondition(LaunchConfiguration("use_safety_gate")),
                output="screen",
            ),
        ]
    )
