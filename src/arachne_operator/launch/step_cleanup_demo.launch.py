from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("max_approach_steps", default_value="5"),
        DeclareLaunchArgument("approach_step_m", default_value="0.12"),
        DeclareLaunchArgument("max_grasps", default_value="1"),
        DeclareLaunchArgument("grasp_min_base_x_m", default_value="0.30"),
        DeclareLaunchArgument("grasp_max_base_x_m", default_value="0.90"),
        DeclareLaunchArgument("target_base_x_m", default_value="0.72"),
    ]
    node = Node(
        package="arachne_operator",
        executable="step_cleanup_demo",
        name="arachne_step_cleanup_demo",
        output="screen",
        parameters=[
            {
                "max_approach_steps": ParameterValue(
                    LaunchConfiguration("max_approach_steps"), value_type=int
                ),
                "approach_step_m": ParameterValue(
                    LaunchConfiguration("approach_step_m"), value_type=float
                ),
                "max_grasps": ParameterValue(LaunchConfiguration("max_grasps"), value_type=int),
                "grasp_min_base_x_m": ParameterValue(
                    LaunchConfiguration("grasp_min_base_x_m"), value_type=float
                ),
                "grasp_max_base_x_m": ParameterValue(
                    LaunchConfiguration("grasp_max_base_x_m"), value_type=float
                ),
                "target_base_x_m": ParameterValue(
                    LaunchConfiguration("target_base_x_m"), value_type=float
                ),
            }
        ],
    )
    return LaunchDescription([*args, node])
