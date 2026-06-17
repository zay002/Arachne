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
                "restart_search_topic", default_value="/arachne/grasp_preview/restart_search"
            ),
            DeclareLaunchArgument(
                "base_command_topic", default_value="/arachne/grasp_task/base_command"
            ),
            DeclareLaunchArgument("base_state_topic", default_value="/arachne/grasp_task/base_state"),
            DeclareLaunchArgument("patrol_distance_m", default_value="1.2"),
            DeclareLaunchArgument("patrol_step_m", default_value="0.12"),
            DeclareLaunchArgument("detection_confidence", default_value="0.35"),
            DeclareLaunchArgument("detection_timeout_sec", default_value="1.2"),
            DeclareLaunchArgument("base_step_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("grasp_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("reach_recovery_enabled", default_value="true"),
            DeclareLaunchArgument("reach_recovery_max_attempts", default_value="3"),
            DeclareLaunchArgument("reach_recovery_step_m", default_value="0.10"),
            DeclareLaunchArgument("reach_recovery_wait_detection_sec", default_value="3.0"),
            DeclareLaunchArgument("reach_recovery_continue_on_exhausted", default_value="true"),
            DeclareLaunchArgument("loop", default_value="true"),
            Node(
                package="arachne_operator",
                executable="road_cleanup_task_server",
                name="arachne_road_cleanup_task_server",
                output="screen",
                parameters=[
                    {
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "restart_search_topic": LaunchConfiguration("restart_search_topic"),
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
                        "reach_recovery_enabled": ParameterValue(
                            LaunchConfiguration("reach_recovery_enabled"), value_type=bool
                        ),
                        "reach_recovery_max_attempts": ParameterValue(
                            LaunchConfiguration("reach_recovery_max_attempts"), value_type=int
                        ),
                        "reach_recovery_step_m": ParameterValue(
                            LaunchConfiguration("reach_recovery_step_m"), value_type=float
                        ),
                        "reach_recovery_wait_detection_sec": ParameterValue(
                            LaunchConfiguration("reach_recovery_wait_detection_sec"),
                            value_type=float,
                        ),
                        "reach_recovery_continue_on_exhausted": ParameterValue(
                            LaunchConfiguration("reach_recovery_continue_on_exhausted"),
                            value_type=bool,
                        ),
                        "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                    }
                ],
            ),
        ]
    )
