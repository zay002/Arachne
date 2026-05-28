from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    nav_share = Path(get_package_share_directory("arachne_nav"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    map_path = nav_share / "maps" / "empty.yaml"
    params_path = nav_share / "config" / "nav2_params.yaml"

    robot_description = xacro.process_file(
        str(model_path),
        mappings={"gripper_type": LaunchConfiguration("gripper_type").perform(context)},
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_robot_state_publisher")),
        ),
        Node(
            package="arachne_sim",
            executable="base_sim_controller",
            name="base_sim_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_base_sim")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mock_map_to_odom",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "odom",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_mock_map_odom")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / "launch" / "bringup_launch.py")),
            launch_arguments={
                "slam": "False",
                "map": str(map_path),
                "params_file": str(params_path),
                "use_sim_time": "False",
                "autostart": "True",
                "use_composition": "False",
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_base_sim", default_value="true"),
            DeclareLaunchArgument("with_mock_map_odom", default_value="true"),
            DeclareLaunchArgument("with_robot_state_publisher", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
