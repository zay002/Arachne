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
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    description_share = Path(get_package_share_directory("arachne_description"))
    model_path = description_share / "urdf" / "arachne.urdf.xacro"
    robot_description = xacro.process_file(
        str(model_path),
        mappings={
            "gripper_type": LaunchConfiguration("gripper_type").perform(context),
            "with_lidar": "true",
            "with_ee_camera": "false",
            "with_gazebo_plugins": "true",
            "with_mimic_joints": "false",
            "gazebo_drive_wheels": "true",
        },
    ).toxml()

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_arachne_autopick_model",
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
        condition=IfCondition(LaunchConfiguration("with_gazebo")),
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arachne_autopick_gazebo_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/gz/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/arachne/gazebo/aubo_shoulder_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
            "/arachne/gazebo/aubo_upperArm_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
            "/arachne/gazebo/aubo_foreArm_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
            "/arachne/gazebo/aubo_wrist1_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
            "/arachne/gazebo/aubo_wrist2_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
            "/arachne/gazebo/aubo_wrist3_joint/command@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        condition=IfCondition(LaunchConfiguration("with_gazebo")),
        output="screen",
    )

    camera_follow = Node(
        package="arachne_demo",
        executable="camera_follow_controller",
        name="autopick_camera_follow_controller",
        parameters=[
            {
                "gazebo_gui_camera": True,
                "gazebo_distance": ParameterValue(
                    LaunchConfiguration("gazebo_camera_distance"), value_type=float
                ),
                "odom_topic": "/gz/odom",
                "gazebo_offset_topic": LaunchConfiguration("gazebo_camera_offset_topic"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_camera_tracking")),
        output="screen",
    )

    camera_track_bridge = Node(
        package="arachne_gazebo",
        executable="gazebo_camera_track_bridge",
        name="autopick_gazebo_camera_track_bridge",
        parameters=[
            {
                "offset_topic": LaunchConfiguration("gazebo_camera_offset_topic"),
                "target_name": "arachne",
                "follow_pgain": 0.85,
                "track_pgain": 1.0,
                "publish_rate": 60.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_camera_tracking")),
        output="screen",
    )

    demo_control_bridge = Node(
        package="arachne_gazebo",
        executable="gazebo_demo_control_bridge",
        name="autopick_gazebo_demo_control_bridge",
        parameters=[
            {
                "gripper_type": LaunchConfiguration("gripper_type"),
                "world_name": LaunchConfiguration("world_name"),
                "gripper_closed_position": 0.6,
                "arm_trajectory_topic": "/model/arachne/joint_trajectory",
            }
        ],
        output="screen",
    )

    autopick = Node(
        package="arachne_demo",
        executable="gazebo_autopick_planner",
        name="gazebo_autopick_planner",
        parameters=[
            {
                "odom_topic": "/gz/odom",
                "target_name": "pick_bottle",
                "target_x": 3.40,
                "target_y": -2.35,
                "target_z": 0.38,
                "approach_x": 2.62,
                "approach_y": -2.35,
                "approach_standoff": 0.76,
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_autopick")),
        output="screen",
    )

    return [spawn, bridge, camera_follow, camera_track_bridge, demo_control_bridge, autopick]


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
            DeclareLaunchArgument("with_autopick", default_value="true"),
            DeclareLaunchArgument("with_camera_tracking", default_value="true"),
            DeclareLaunchArgument("gazebo_camera_distance", default_value="2.2"),
            DeclareLaunchArgument(
                "gazebo_camera_offset_topic",
                default_value="/arachne/gazebo_camera/follow_offset",
            ),
            DeclareLaunchArgument("gazebo_render_backend", default_value="opengl"),
            DeclareLaunchArgument("gazebo_update_rate", default_value="180"),
            DeclareLaunchArgument("world_name", default_value="arachne_showroom"),
            DeclareLaunchArgument(
                "gz_args",
                default_value=[
                    "-r -v 1 -z ",
                    LaunchConfiguration("gazebo_update_rate"),
                    " --render-engine ogre2 --render-engine-api-backend ",
                    LaunchConfiguration("gazebo_render_backend"),
                    " ",
                    str(world_path),
                ],
            ),
            gz_resource_path,
            gz_sim,
            OpaqueFunction(function=launch_setup),
        ]
    )
