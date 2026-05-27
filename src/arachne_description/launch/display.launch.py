from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DISPLAY_ARM_ZEROS = {
    "zeros.aubo_shoulder_joint": 1.664,
    "zeros.aubo_upperArm_joint": 0.034,
    "zeros.aubo_foreArm_joint": -1.324,
    "zeros.aubo_wrist1_joint": 0.034,
    "zeros.aubo_wrist2_joint": -1.732,
    "zeros.aubo_wrist3_joint": 0.0,
}


def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("arachne_description"))
    model_path = pkg_share / "urdf" / "arachne.urdf.xacro"
    rviz_config = pkg_share / "rviz" / "arachne_model.rviz"
    generated_urdf = Path("/tmp/arachne_display.urdf")

    mappings = {
        "arm_mount_xyz": LaunchConfiguration("arm_mount_xyz").perform(context),
        "arm_mount_rpy": LaunchConfiguration("arm_mount_rpy").perform(context),
        "tool_adapter_xyz": LaunchConfiguration("tool_adapter_xyz").perform(context),
        "tool_adapter_rpy": LaunchConfiguration("tool_adapter_rpy").perform(context),
        "gripper_type": LaunchConfiguration("gripper_type").perform(context),
        "with_lidar": LaunchConfiguration("with_lidar").perform(context),
        "with_ee_camera": LaunchConfiguration("with_ee_camera").perform(context),
    }

    robot_description = xacro.process_file(str(model_path), mappings=mappings).toxml()
    generated_urdf.write_text(robot_description)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="default_joint_state_publisher",
            arguments=[str(generated_urdf)],
            parameters=[DISPLAY_ARM_ZEROS],
            remappings=[("joint_states", "/arachne/default_joint_states")],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            arguments=[str(generated_urdf)],
            parameters=[DISPLAY_ARM_ZEROS],
            remappings=[("joint_states", "/arachne/gui_joint_states")],
            condition=IfCondition(LaunchConfiguration("use_gui")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="joint_state_mux",
            name="joint_state_mux",
            output="screen",
        ),
        Node(
            package="arachne_sim",
            executable="base_sim_controller",
            name="base_sim_controller",
            parameters=[
                {
                    "max_linear_velocity": ParameterValue(
                        LaunchConfiguration("base_max_linear_velocity"), value_type=float
                    ),
                    "max_angular_velocity": ParameterValue(
                        LaunchConfiguration("base_max_angular_velocity"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_base_sim")),
            output="screen",
        ),
        Node(
            package="arachne_sim",
            executable="base_teleop_gui",
            name="base_teleop_gui",
            parameters=[
                {
                    "linear_speed": ParameterValue(
                        LaunchConfiguration("base_linear_speed"), value_type=float
                    ),
                    "angular_speed": ParameterValue(
                        LaunchConfiguration("base_angular_speed"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_base_gui")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="gripper_sim_controller",
            name="gripper_sim_controller",
            parameters=[
                {
                    "profile": LaunchConfiguration("gripper_sim_profile"),
                    "open_position": ParameterValue(
                        LaunchConfiguration("gripper_open_position"), value_type=float
                    ),
                    "closed_position": ParameterValue(
                        LaunchConfiguration("gripper_closed_position"), value_type=float
                    ),
                    "max_velocity": ParameterValue(
                        LaunchConfiguration("gripper_max_velocity"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_gripper_sim")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="gripper_state_gui",
            name="gripper_state_gui",
            condition=IfCondition(LaunchConfiguration("with_gripper_gui")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("with_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_mount_xyz", default_value="0.22 0.0 0.155"),
            DeclareLaunchArgument("arm_mount_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument("tool_adapter_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("tool_adapter_rpy", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_base_sim", default_value="true"),
            DeclareLaunchArgument("with_base_gui", default_value="false"),
            DeclareLaunchArgument("base_linear_speed", default_value="0.25"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.65"),
            DeclareLaunchArgument("base_max_linear_velocity", default_value="0.8"),
            DeclareLaunchArgument("base_max_angular_velocity", default_value="1.4"),
            DeclareLaunchArgument("with_gripper_sim", default_value="false"),
            DeclareLaunchArgument("with_gripper_gui", default_value="false"),
            DeclareLaunchArgument("gripper_sim_profile", default_value="ms42dc"),
            DeclareLaunchArgument("gripper_open_position", default_value="-1.0"),
            DeclareLaunchArgument("gripper_closed_position", default_value="-1.0"),
            DeclareLaunchArgument("gripper_max_velocity", default_value="-1.0"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("with_lidar", default_value="true"),
            DeclareLaunchArgument("with_ee_camera", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
