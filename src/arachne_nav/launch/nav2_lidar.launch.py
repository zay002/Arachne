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


def _launch_bool(context, name: str) -> str:
    return "True" if _as_bool(LaunchConfiguration(name).perform(context)) else "False"


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    nav_share = Path(get_package_share_directory("arachne_nav"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    params_path = Path(LaunchConfiguration("params_file").perform(context))
    map_path = nav_share / "maps" / "empty.yaml"
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context))

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
                "use_composition": _launch_bool(context, "use_composition"),
                "log_level": LaunchConfiguration("log_level").perform(context),
            }.items(),
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="lslidar_pointcloud_to_scan",
            remappings=[
                ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                ("scan", LaunchConfiguration("scan_topic")),
            ],
            parameters=[
                {
                    "target_frame": LaunchConfiguration("laser_target_frame"),
                    "transform_tolerance": 0.2,
                    "min_height": -0.35,
                    "max_height": 0.35,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.0087,
                    "scan_time": 0.10,
                    "range_min": 0.15,
                    "range_max": 18.0,
                    "use_inf": True,
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_pointcloud_to_scan")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="arachne_nav_topdown_rviz",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("with_rviz")),
            output="screen",
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
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(nav_share / "rviz" / "arachne_nav_topdown.rviz"),
            ),
            DeclareLaunchArgument("with_lslidar_driver", default_value="false"),
            DeclareLaunchArgument("with_pointcloud_to_scan", default_value="true"),
            DeclareLaunchArgument("with_robot_state_publisher", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/lslidar_point_cloud"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("laser_target_frame", default_value="lidar_link"),
            DeclareLaunchArgument("use_composition", default_value="True"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            OpaqueFunction(function=launch_setup),
        ]
    )
