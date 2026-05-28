from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    moveit_share = Path(get_package_share_directory("arachne_moveit_config"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    gripper_type = LaunchConfiguration("gripper_type").perform(context)
    srdf_path = moveit_share / "config" / f"arachne_{gripper_type}.srdf.xacro"

    robot_description = xacro.process_file(
        str(model_path),
        mappings={
            "gripper_type": gripper_type,
            "with_ros2_control": "true",
            "with_mimic_joints": "false",
        },
    ).toxml()
    robot_description_semantic = xacro.process_file(str(srdf_path)).toxml()

    moveit_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": load_yaml(moveit_share / "config" / "kinematics.yaml"),
        "robot_description_planning": load_yaml(moveit_share / "config" / "joint_limits.yaml"),
        **load_yaml(moveit_share / "config" / "ompl_planning.yaml"),
        **load_yaml(moveit_share / "config" / "moveit_controllers.yaml"),
    }

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            parameters=[moveit_params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
