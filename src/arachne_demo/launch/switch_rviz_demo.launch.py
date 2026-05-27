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
            "with_base_sim": "true",
            "with_base_gui": "false",
            "with_gripper_sim": "true",
            "with_gripper_gui": "false",
            "gripper_sim_profile": LaunchConfiguration("gripper_type"),
        }.items(),
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

    switch_teleop = Node(
        package="arachne_demo",
        executable="switch_teleop",
        name="switch_teleop",
        parameters=[
            {
                "deadzone": ParameterValue(LaunchConfiguration("teleop_deadzone"), value_type=float),
                "linear_scale": ParameterValue(LaunchConfiguration("linear_scale"), value_type=float),
                "angular_scale": ParameterValue(LaunchConfiguration("angular_scale"), value_type=float),
                "joint_velocity_scale": ParameterValue(
                    LaunchConfiguration("joint_velocity_scale"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("with_joy", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_rate", default_value="30.0"),
            DeclareLaunchArgument("teleop_deadzone", default_value="0.12"),
            DeclareLaunchArgument("linear_scale", default_value="0.55"),
            DeclareLaunchArgument("angular_scale", default_value="1.1"),
            DeclareLaunchArgument("joint_velocity_scale", default_value="0.85"),
            display,
            joy,
            switch_teleop,
        ]
    )
