import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    demo_share = Path(get_package_share_directory("arachne_demo"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"

    mappings = {
        "gripper_type": LaunchConfiguration("gripper_type").perform(context),
        "with_lidar": "true",
        "with_ee_camera": "false",
        "with_gazebo_plugins": "true",
    }
    robot_description = xacro.process_file(str(model_path), mappings=mappings).toxml()

    with_gazebo = LaunchConfiguration("with_gazebo").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_arachne_showroom_model",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
            "-name",
            "arachne",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.22",
            "-string",
            robot_description,
        ],
        output="screen",
    )

    switch_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(demo_share / "launch" / "switch_rviz_demo.launch.py")),
        launch_arguments={
            "gripper_type": LaunchConfiguration("gripper_type"),
            "with_rviz": LaunchConfiguration("with_rviz"),
            "with_joy": LaunchConfiguration("with_joy"),
            "with_web_gamepad": LaunchConfiguration("with_web_gamepad"),
            "joy_dev": LaunchConfiguration("joy_dev"),
            "web_gamepad_host": LaunchConfiguration("web_gamepad_host"),
            "web_gamepad_port": LaunchConfiguration("web_gamepad_port"),
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arachne_gazebo_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
    )

    actions = [switch_demo]
    if with_gazebo:
        actions.extend([spawn, bridge])
    return actions


def generate_launch_description():
    demo_share = Path(get_package_share_directory("arachne_demo"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    world_path = demo_share / "worlds" / "arachne_showroom.sdf"
    mesh_resource_path = os.pathsep.join(
        str(Path(get_package_share_directory(package_name)).parent)
        for package_name in (
            "arachne_description",
            "aubo_description",
            "scout_description",
            "dh_ag95_description",
        )
    )

    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [mesh_resource_path, os.pathsep, EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        condition=IfCondition(LaunchConfiguration("with_gazebo")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_gazebo", default_value="true"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("with_joy", default_value="true"),
            DeclareLaunchArgument("with_web_gamepad", default_value="false"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("web_gamepad_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("web_gamepad_port", default_value="8787"),
            DeclareLaunchArgument("world_name", default_value="arachne_showroom"),
            DeclareLaunchArgument("gz_args", default_value=f"-r {world_path}"),
            gz_resource_path,
            gz_sim,
            OpaqueFunction(function=launch_setup),
        ]
    )
