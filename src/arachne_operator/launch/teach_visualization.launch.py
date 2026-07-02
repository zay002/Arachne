from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context))
    generated_urdf = Path("/tmp/arachne_display.urdf")

    mappings = {
        "arm_mount_xyz": LaunchConfiguration("arm_mount_xyz").perform(context),
        "arm_mount_rpy": LaunchConfiguration("arm_mount_rpy").perform(context),
        "ee_support_xyz": LaunchConfiguration("ee_support_xyz").perform(context),
        "ee_support_rpy": LaunchConfiguration("ee_support_rpy").perform(context),
        "tool_adapter_xyz": LaunchConfiguration("tool_adapter_xyz").perform(context),
        "tool_adapter_rpy": LaunchConfiguration("tool_adapter_rpy").perform(context),
        "gripper_type": LaunchConfiguration("gripper_type").perform(context),
        "with_lidar": LaunchConfiguration("with_lidar").perform(context),
        "with_ee_camera": LaunchConfiguration("with_ee_camera").perform(context),
        "with_ros2_control": "false",
        "with_gazebo_plugins": "false",
    }
    robot_description = xacro.process_file(str(model_path), mappings=mappings).toxml()
    generated_urdf.write_text(robot_description)

    visualization_joint_states = LaunchConfiguration("visualization_joint_states_topic")

    actions = [
        Node(
            package="arachne_operator",
            executable="teach_visualization_joint_states",
            name="teach_visualization_joint_states",
            parameters=[
                {
                    "input_topic": LaunchConfiguration("input_joint_states_topic"),
                    "output_topic": visualization_joint_states,
                    "aubo_sdk_fallback_enabled": ParameterValue(
                        LaunchConfiguration("aubo_sdk_fallback_enabled"), value_type=bool
                    ),
                    "aubo_sdk_ip": LaunchConfiguration("aubo_sdk_ip"),
                    "aubo_sdk_port": ParameterValue(
                        LaunchConfiguration("aubo_sdk_port"), value_type=int
                    ),
                    "aubo_sdk_timeout_sec": ParameterValue(
                        LaunchConfiguration("aubo_sdk_timeout_sec"), value_type=float
                    ),
                }
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="arachne_teach_robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[
                ("joint_states", visualization_joint_states),
                (
                    "robot_description",
                    LaunchConfiguration("display_robot_description_topic"),
                ),
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="arachne_teach_rviz",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("with_rviz")),
            output="screen",
        ),
    ]
    if str(LaunchConfiguration("with_lslidar_driver").perform(context)).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        lslidar_share = Path(get_package_share_directory("lslidar_driver"))
        actions.insert(
            0,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lslidar_share / "launch" / "lslidar_cx_launch.py"))
            ),
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "visualization_joint_states_topic",
                default_value="/arachne/teach_visualization/joint_states",
            ),
            DeclareLaunchArgument(
                "display_robot_description_topic",
                default_value="/arachne/display/robot_description",
            ),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    Path(get_package_share_directory("arachne_description"))
                    / "rviz"
                    / "arachne_lidar_fusion.rviz"
                ),
            ),
            DeclareLaunchArgument("arm_mount_xyz", default_value="0.22 0.0 0.105"),
            DeclareLaunchArgument("arm_mount_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument("ee_support_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("ee_support_rpy", default_value="0.0 0.0 2.35619449019"),
            DeclareLaunchArgument("tool_adapter_xyz", default_value="-0.049334103 0.049874070 0.021816675"),
            DeclareLaunchArgument("tool_adapter_rpy", default_value="1.570796327 0.0 0.0"),
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_lidar", default_value="true"),
            DeclareLaunchArgument("with_lslidar_driver", default_value="false"),
            DeclareLaunchArgument("with_ee_camera", default_value="true"),
            DeclareLaunchArgument("aubo_sdk_fallback_enabled", default_value="true"),
            DeclareLaunchArgument("aubo_sdk_ip", default_value="192.168.127.128"),
            DeclareLaunchArgument("aubo_sdk_port", default_value="30004"),
            DeclareLaunchArgument("aubo_sdk_timeout_sec", default_value="0.5"),
            OpaqueFunction(function=launch_setup),
        ]
    )
