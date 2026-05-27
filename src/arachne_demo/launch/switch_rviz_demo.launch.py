from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("arachne_description"))
    display_launch = description_share / "launch" / "display.launch.py"

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(display_launch)),
        launch_arguments={
            "gripper_type": LaunchConfiguration("gripper_type"),
            "use_gui": "false",
            "with_rviz": LaunchConfiguration("with_rviz"),
            "with_base_sim": LaunchConfiguration("with_base_sim"),
            "with_base_gui": "false",
            "with_gripper_sim": "true",
            "with_gripper_gui": "false",
            "gripper_sim_profile": LaunchConfiguration("gripper_type"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_description")),
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="switch_joy",
        parameters=[
            {
                "dev": LaunchConfiguration("joy_dev"),
                "deadzone": ParameterValue(LaunchConfiguration("joy_deadzone"), value_type=float),
                "autorepeat_rate": ParameterValue(LaunchConfiguration("joy_rate"), value_type=float),
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_joy")),
        output="screen",
    )

    web_gamepad = Node(
        package="arachne_demo",
        executable="web_gamepad_bridge",
        name="web_gamepad_bridge",
        parameters=[
            {
                "host": LaunchConfiguration("web_gamepad_host"),
                "port": ParameterValue(LaunchConfiguration("web_gamepad_port"), value_type=int),
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_web_gamepad")),
        output="screen",
    )

    switch_teleop = Node(
        package="arachne_demo",
        executable="switch_teleop",
        name="switch_teleop",
        parameters=[
            {
                "deadzone": ParameterValue(LaunchConfiguration("teleop_deadzone"), value_type=float),
                "linear_scale": ParameterValue(LaunchConfiguration("linear_scale"), value_type=float),
                "angular_scale": ParameterValue(LaunchConfiguration("angular_scale"), value_type=float),
                "drive_mode": LaunchConfiguration("drive_mode"),
                "forward_axis_multiplier": ParameterValue(
                    LaunchConfiguration("forward_axis_multiplier"), value_type=float
                ),
                "lateral_axis_multiplier": ParameterValue(
                    LaunchConfiguration("lateral_axis_multiplier"), value_type=float
                ),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "reset_topic": LaunchConfiguration("reset_topic"),
                "base_reset_service": LaunchConfiguration("base_reset_service"),
                "joint_velocity_scale": ParameterValue(
                    LaunchConfiguration("joint_velocity_scale"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    camera_follow = Node(
        package="arachne_demo",
        executable="camera_follow_controller",
        name="camera_follow_controller",
        parameters=[
            {
                "deadzone": ParameterValue(LaunchConfiguration("teleop_deadzone"), value_type=float),
                "gazebo_gui_camera": ParameterValue(
                    LaunchConfiguration("gazebo_gui_camera"), value_type=bool
                ),
                "gazebo_distance": ParameterValue(
                    LaunchConfiguration("gazebo_camera_distance"), value_type=float
                ),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "gazebo_offset_topic": LaunchConfiguration("gazebo_camera_offset_topic"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_description", default_value="true"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("with_base_sim", default_value="true"),
            DeclareLaunchArgument("with_joy", default_value="true"),
            DeclareLaunchArgument("with_web_gamepad", default_value="false"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_rate", default_value="30.0"),
            DeclareLaunchArgument("web_gamepad_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("web_gamepad_port", default_value="8787"),
            DeclareLaunchArgument("teleop_deadzone", default_value="0.12"),
            DeclareLaunchArgument("linear_scale", default_value="0.55"),
            DeclareLaunchArgument("angular_scale", default_value="1.1"),
            DeclareLaunchArgument("drive_mode", default_value="body"),
            DeclareLaunchArgument("forward_axis_multiplier", default_value="-1.0"),
            DeclareLaunchArgument("lateral_axis_multiplier", default_value="1.0"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("reset_topic", default_value="/arachne/demo/reset"),
            DeclareLaunchArgument("base_reset_service", default_value="/arachne/base/reset"),
            DeclareLaunchArgument("joint_velocity_scale", default_value="0.85"),
            DeclareLaunchArgument("gazebo_gui_camera", default_value="false"),
            DeclareLaunchArgument("gazebo_camera_distance", default_value="2.0"),
            DeclareLaunchArgument(
                "gazebo_camera_offset_topic",
                default_value="/arachne/gazebo_camera/follow_offset",
            ),
            display,
            joy,
            web_gamepad,
            switch_teleop,
            camera_follow,
        ]
    )
