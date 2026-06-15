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
    moveit_share = Path(get_package_share_directory("arachne_moveit_config"))
    display_launch = description_share / "launch" / "display.launch.py"
    moveit_launch = moveit_share / "launch" / "moveit_planning.launch.py"
    rviz_config = description_share / "rviz" / "moveit_grasp_demo.rviz"

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(display_launch)),
        launch_arguments={
            "with_rviz": "false",
            "with_base_gui": "false",
            "with_gripper_gui": "false",
            "with_gripper_sim": "false",
            "use_gui": "false",
            "gripper_type": LaunchConfiguration("gripper_type"),
            "display_frame_prefix": "",
            "display_joint_states_topic": "/joint_states",
            "display_robot_description_topic": "/robot_description",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_launch)),
        launch_arguments={
            "launch_rviz": "false",
            "with_robot_state_publisher": "false",
            "gripper_type": LaunchConfiguration("gripper_type"),
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    demo = Node(
        package="arachne_sim",
        executable="moveit_grasp_planning_demo",
        name="moveit_grasp_planning_demo",
        parameters=[
            {
                "planner_id": LaunchConfiguration("planner_id"),
                "playback_speed": ParameterValue(
                    LaunchConfiguration("playback_speed"), value_type=float
                ),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "allow_interpolation_fallback": ParameterValue(
                    LaunchConfiguration("allow_interpolation_fallback"), value_type=bool
                ),
            }
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="arachne_moveit_grasp_demo_rviz",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(LaunchConfiguration("launch_demo_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("playback_speed", default_value="0.8"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("launch_demo_rviz", default_value="true"),
            DeclareLaunchArgument("allow_interpolation_fallback", default_value="false"),
            display,
            moveit,
            demo,
            rviz,
        ]
    )
