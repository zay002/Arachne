from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arachne_operator",
                executable="operator_panel",
                name="arachne_operator_panel",
                output="screen",
            )
        ]
    )
