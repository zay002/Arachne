from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("arachne_description"))
    model_path = pkg_share / "urdf" / "arachne.urdf.xacro"
    rviz_config = pkg_share / "rviz" / "arachne_model.rviz"
    generated_urdf = Path("/tmp/arachne_display.urdf")

    mappings = {
        "arm_mount_xyz": LaunchConfiguration("arm_mount_xyz").perform(context),
        "arm_mount_rpy": LaunchConfiguration("arm_mount_rpy").perform(context),
        "tool_adapter_xyz": LaunchConfiguration("tool_adapter_xyz").perform(context),
        "tool_adapter_rpy": LaunchConfiguration("tool_adapter_rpy").perform(context),
        "with_lidar": LaunchConfiguration("with_lidar").perform(context),
        "with_ee_camera": LaunchConfiguration("with_ee_camera").perform(context),
    }

    robot_description = xacro.process_file(str(model_path), mappings=mappings).toxml()
    generated_urdf.write_text(robot_description)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            arguments=[str(generated_urdf)],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_mount_xyz", default_value="0.22 0.0 0.155"),
            DeclareLaunchArgument("arm_mount_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument("tool_adapter_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("tool_adapter_rpy", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("with_lidar", default_value="true"),
            DeclareLaunchArgument("with_ee_camera", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
