from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detection_topic", default_value="/arachne/perception/taco_instances"
            ),
            DeclareLaunchArgument(
                "base_command_topic", default_value="/arachne/grasp_task/base_command"
            ),
            DeclareLaunchArgument("base_state_topic", default_value="/arachne/grasp_task/base_state"),
            DeclareLaunchArgument("patrol_distance_m", default_value="2.0"),
            DeclareLaunchArgument("patrol_step_m", default_value="0.12"),
            DeclareLaunchArgument("detection_confidence", default_value="0.35"),
            DeclareLaunchArgument("detection_timeout_sec", default_value="1.2"),
            DeclareLaunchArgument("base_step_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("grasp_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("loop", default_value="true"),
            Node(
                package="arachne_operator",
                executable="road_cleanup_task_server",
                name="arachne_road_cleanup_task_server",
                output="screen",
                parameters=[
                    {
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "base_command_topic": LaunchConfiguration("base_command_topic"),
                        "base_state_topic": LaunchConfiguration("base_state_topic"),
                        "patrol_distance_m": ParameterValue(
                            LaunchConfiguration("patrol_distance_m"), value_type=float
                        ),
                        "patrol_step_m": ParameterValue(
                            LaunchConfiguration("patrol_step_m"), value_type=float
                        ),
                        "detection_confidence": ParameterValue(
                            LaunchConfiguration("detection_confidence"), value_type=float
                        ),
                        "detection_timeout_sec": ParameterValue(
                            LaunchConfiguration("detection_timeout_sec"), value_type=float
                        ),
                        "base_step_timeout_sec": ParameterValue(
                            LaunchConfiguration("base_step_timeout_sec"), value_type=float
                        ),
                        "grasp_timeout_sec": ParameterValue(
                            LaunchConfiguration("grasp_timeout_sec"), value_type=float
                        ),
                        "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                    }
                ],
            ),
        ]
    )
