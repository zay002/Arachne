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
        scout_driver = LaunchConfiguration("scout_driver").perform(context).strip().lower()
        if scout_driver in ("official", "socketcan", "scout_base"):
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
        elif scout_driver in ("waveshare", "serial", "usb_can_a", "usb-can-a"):
            actions.append(
                Node(
                    package="arachne_hardware",
                    executable="scout_waveshare_serial_driver",
                    name="scout_waveshare_serial_driver",
                    parameters=[
                        {
                            "port": LaunchConfiguration("scout_port"),
                            "usb_baudrate": ParameterValue(
                                LaunchConfiguration("scout_usb_baudrate"), value_type=int
                            ),
                            "can_bitrate": ParameterValue(
                                LaunchConfiguration("scout_can_bitrate"), value_type=int
                            ),
                            "frame_type": LaunchConfiguration("scout_frame_type"),
                            "cmd_vel_topic": "/cmd_vel",
                            "odom_topic": "/odom",
                            "odom_frame": "odom",
                            "base_frame": "base_link",
                        }
                    ],
                    output="screen",
                )
            )
        else:
            raise RuntimeError(
                "Unsupported scout_driver value "
                f"{scout_driver!r}; use waveshare or official."
            )

    if enabled("use_ms42dc"):
        ms42dc_driver = LaunchConfiguration("ms42dc_driver").perform(context).strip().lower()
        if ms42dc_driver in ("vendor", "official", "step_motor"):
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
        elif ms42dc_driver in ("direct", "serial", "arachne"):
            actions.append(
                Node(
                    package="arachne_hardware",
                    executable="ms42dc_direct_serial_driver",
                    name="ms42dc_direct_serial_driver",
                    parameters=[
                        {
                            "port": LaunchConfiguration("ms42dc_port"),
                            "baudrate": ParameterValue(
                                LaunchConfiguration("ms42dc_baudrate"), value_type=int
                            ),
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
        else:
            raise RuntimeError(
                "Unsupported ms42dc_driver value "
                f"{ms42dc_driver!r}; use direct or vendor."
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

        actions.append(
            Node(
                package="arachne_hardware",
                executable="aubo_teach_command_bridge",
                name="aubo_teach_command_bridge",
                parameters=[
                    {
                        "command_topic": LaunchConfiguration("aubo_teach_command_topic"),
                        "jsonrpc_service": LaunchConfiguration("aubo_jsonrpc_service"),
                        "teach_method": LaunchConfiguration("aubo_teach_method"),
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
            DeclareLaunchArgument("scout_driver", default_value="waveshare"),
            DeclareLaunchArgument(
                "scout_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("scout_usb_baudrate", default_value="2000000"),
            DeclareLaunchArgument("scout_can_bitrate", default_value="500000"),
            DeclareLaunchArgument("scout_frame_type", default_value="standard"),
            DeclareLaunchArgument("ms42dc_driver", default_value="direct"),
            DeclareLaunchArgument("ms42dc_port", default_value="/dev/motor_serial"),
            DeclareLaunchArgument("ms42dc_baudrate", default_value="115200"),
            DeclareLaunchArgument("ms42dc_device_id", default_value="1"),
            DeclareLaunchArgument("ms42dc_sub_divide", default_value="32"),
            DeclareLaunchArgument("ms42dc_mode", default_value="2"),
            DeclareLaunchArgument("ms42dc_open_angle_tenths", default_value="18720"),
            DeclareLaunchArgument("ms42dc_close_angle_tenths", default_value="18720"),
            DeclareLaunchArgument("ms42dc_speed_tenths", default_value="150"),
            DeclareLaunchArgument("aubo_robot_ip", default_value="192.168.127.128"),
            DeclareLaunchArgument("aubo_port", default_value="80"),
            DeclareLaunchArgument("aubo_teach_command_topic", default_value="/arachne/aubo/teach_command"),
            DeclareLaunchArgument("aubo_jsonrpc_service", default_value="jsonrpc_service"),
            DeclareLaunchArgument("aubo_teach_method", default_value="freedrive"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
