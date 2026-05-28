from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _package_launch(package_name: str, *relative: str) -> PythonLaunchDescriptionSource:
    try:
        package_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Missing ROS2 package `{package_name}`. Run scripts/fetch_third_party.sh "
            "and scripts/prepare_ms42dc_ros2.sh, then rebuild the workspace."
        ) from exc
    return PythonLaunchDescriptionSource(str(package_share.joinpath(*relative)))


def _launch_setup(context, *args, **kwargs):
    actions = []

    def enabled(name: str) -> bool:
        value = LaunchConfiguration(name).perform(context).strip().lower()
        return value in ("1", "true", "yes", "on")

    if enabled("use_scout"):
        actions.append(
            IncludeLaunchDescription(
                _package_launch("scout_base", "launch", "scout_base.launch.py"),
                launch_arguments={
                    "port_name": LaunchConfiguration("scout_port"),
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "odom_topic_name": "/odom",
                    "is_scout_mini": "false",
                    "is_omni_wheel": "false",
                    "simulated_robot": "false",
                }.items(),
            )
        )

        actions.append(
            Node(
                package="arachne_hardware",
                executable="scout_official_status_bridge",
                name="scout_official_status_bridge",
                parameters=[{"odom_topic": "/odom"}],
                output="screen",
            )
        )

    if enabled("use_ms42dc"):
        actions.append(
            Node(
                package="step_motor",
                executable="motor_node",
                name="ms42dc_step_motor_node",
                parameters=[
                    {
                        "usart_port_name": LaunchConfiguration("ms42dc_port"),
                        "serial_baud_rate": ParameterValue(
                            LaunchConfiguration("ms42dc_baudrate"), value_type=int
                        ),
                    }
                ],
                output="screen",
            )
        )

        actions.append(
            Node(
                package="arachne_hardware",
                executable="ms42dc_official_bridge",
                name="ms42dc_official_bridge",
                parameters=[
                    {
                        "device_id": ParameterValue(
                            LaunchConfiguration("ms42dc_device_id"), value_type=int
                        ),
                        "sub_divide": ParameterValue(
                            LaunchConfiguration("ms42dc_sub_divide"), value_type=int
                        ),
                        "mode": ParameterValue(
                            LaunchConfiguration("ms42dc_mode"), value_type=int
                        ),
                        "open_angle_tenths": ParameterValue(
                            LaunchConfiguration("ms42dc_open_angle_tenths"), value_type=int
                        ),
                        "close_angle_tenths": ParameterValue(
                            LaunchConfiguration("ms42dc_close_angle_tenths"), value_type=int
                        ),
                        "speed_tenths": ParameterValue(
                            LaunchConfiguration("ms42dc_speed_tenths"), value_type=int
                        ),
                    }
                ],
                output="screen",
            )
        )

    if enabled("use_aubo"):
        actions.append(
            IncludeLaunchDescription(
                _package_launch("aubo_ros2_driver", "launch", "aubo_control.launch.py"),
                launch_arguments={
                    "aubo_type": "aubo_i5",
                    "robot_ip": LaunchConfiguration("aubo_robot_ip"),
                    "use_fake_hardware": "false",
                }.items(),
            )
        )

        actions.append(
            Node(
                package="arachne_hardware",
                executable="aubo_official_status_probe",
                name="aubo_official_status_probe",
                parameters=[
                    {
                        "robot_ip": LaunchConfiguration("aubo_robot_ip"),
                        "aubo_port": ParameterValue(
                            LaunchConfiguration("aubo_port"), value_type=int
                        ),
                    }
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_scout", default_value="true"),
            DeclareLaunchArgument("use_ms42dc", default_value="true"),
            DeclareLaunchArgument("use_aubo", default_value="true"),
            DeclareLaunchArgument("scout_port", default_value="can0"),
            DeclareLaunchArgument("ms42dc_port", default_value="/dev/motor_serial"),
            DeclareLaunchArgument("ms42dc_baudrate", default_value="115200"),
            DeclareLaunchArgument("ms42dc_device_id", default_value="1"),
            DeclareLaunchArgument("ms42dc_sub_divide", default_value="32"),
            DeclareLaunchArgument("ms42dc_mode", default_value="2"),
            DeclareLaunchArgument("ms42dc_open_angle_tenths", default_value="18720"),
            DeclareLaunchArgument("ms42dc_close_angle_tenths", default_value="18720"),
            DeclareLaunchArgument("ms42dc_speed_tenths", default_value="200"),
            DeclareLaunchArgument("aubo_robot_ip", default_value="192.168.127.128"),
            DeclareLaunchArgument("aubo_port", default_value="80"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
