from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    nav_share = Path(get_package_share_directory("arachne_nav"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    params_path = Path(LaunchConfiguration("params_file").perform(context))
    map_path = nav_share / "maps" / "empty.yaml"

    robot_description = xacro.process_file(
        str(model_path),
        mappings={"gripper_type": LaunchConfiguration("gripper_type").perform(context)},
    ).toxml()

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_robot_state_publisher")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / "launch" / "bringup_launch.py")),
            launch_arguments={
                "slam": "True",
                "map": str(map_path),
                "params_file": str(params_path),
                "use_sim_time": "False",
                "autostart": "True",
                "use_composition": LaunchConfiguration("use_composition").perform(context),
                "log_level": LaunchConfiguration("log_level").perform(context),
            }.items(),
        ),
    ]

    if _as_bool(LaunchConfiguration("with_lslidar_driver").perform(context)):
        lslidar_share = Path(get_package_share_directory("lslidar_c16_decoder"))
        actions.insert(
            0,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lslidar_share / "launch" / "lslidar_c16_launch.py"))
            ),
        )

    return actions


def generate_launch_description():
    nav_share = Path(get_package_share_directory("arachne_nav"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(nav_share / "config" / "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("with_lslidar_driver", default_value="false"),
            DeclareLaunchArgument("with_robot_state_publisher", default_value="false"),
            DeclareLaunchArgument("use_composition", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            OpaqueFunction(function=launch_setup),
        ]
    )
