from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name: str, relative_path: str, arguments: dict[str, object]):
    share = Path(get_package_share_directory(package_name))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / relative_path)),
        launch_arguments={key: value for key, value in arguments.items()}.items(),
    )


def generate_launch_description():
    gripper_type = LaunchConfiguration("gripper_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_operator = LaunchConfiguration("launch_operator")
    launch_action_translator = LaunchConfiguration("launch_action_translator")
    use_safety_gate = LaunchConfiguration("use_safety_gate")
    with_mock_map_odom = LaunchConfiguration("with_mock_map_odom")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("launch_operator", default_value="true"),
            DeclareLaunchArgument("launch_action_translator", default_value="true"),
            DeclareLaunchArgument("use_safety_gate", default_value="false"),
            DeclareLaunchArgument("with_mock_map_odom", default_value="true"),
            include_launch(
                "arachne_hardware",
                "launch/mock_bringup.launch.py",
                {"use_safety_gate": use_safety_gate},
            ),
            include_launch(
                "arachne_nav",
                "launch/nav2_sim.launch.py",
                {
                    "gripper_type": gripper_type,
                    "with_base_sim": "false",
                    "with_robot_state_publisher": "false",
                    "with_mock_map_odom": with_mock_map_odom,
                },
            ),
            include_launch(
                "arachne_moveit_config",
                "launch/moveit_planning.launch.py",
                {
                    "gripper_type": gripper_type,
                    "launch_rviz": launch_rviz,
                    "with_robot_state_publisher": "true",
                },
            ),
            include_launch(
                "arachne_operator",
                "launch/sequence_executor.launch.py",
                {},
            ),
            Node(
                package="arachne_operator",
                executable="action_chunk_translator",
                name="arachne_action_chunk_translator",
                output="screen",
                condition=IfCondition(launch_action_translator),
            ),
            Node(
                package="arachne_operator",
                executable="operator_panel",
                name="arachne_operator_panel",
                output="screen",
                condition=IfCondition(launch_operator),
            ),
        ]
    )
