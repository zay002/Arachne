from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arachne_operator",
                executable="sequence_executor",
                name="arachne_sequence_executor",
                output="screen",
            )
        ]
    )
