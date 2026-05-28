from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    control_share = Path(get_package_share_directory("arachne_control"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    controllers_path = control_share / "config" / "ros2_controllers.yaml"
    gripper_type = LaunchConfiguration("gripper_type").perform(context)

    robot_description = xacro.process_file(
        str(model_path),
        mappings={
            "gripper_type": gripper_type,
            "with_ros2_control": "true",
            "with_mimic_joints": "false",
        },
    ).toxml()

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["aubo_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["scout_diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    gripper_controller = (
        "ag95_gripper_controller" if gripper_type == "ag95" else "ms42dc_gripper_controller"
    )
    spawners.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[gripper_controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, str(controllers_path)],
            output="screen",
        ),
        *spawners,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            OpaqueFunction(function=launch_setup),
        ]
    )
