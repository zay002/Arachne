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
    nav_share = Path(get_package_share_directory("arachne_nav"))
    display_launch = description_share / "launch" / "display.launch.py"
    moveit_launch = moveit_share / "launch" / "moveit_planning.launch.py"
    rviz_config = description_share / "rviz" / "urban_trash_sorting_demo.rviz"
    default_slam_map = nav_share / "maps" / "road_lab_apriltag.yaml"

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(display_launch)),
        launch_arguments={
            "with_rviz": "false",
            "with_base_sim": "true",
            "with_base_gui": "false",
            "with_gripper_gui": "false",
            "with_gripper_sim": "false",
            "use_gui": "false",
            "gripper_type": LaunchConfiguration("gripper_type"),
            "display_frame_prefix": "",
            "display_joint_states_topic": "/joint_states",
            "display_robot_description_topic": "/arachne/display/robot_description",
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
        condition=IfCondition(LaunchConfiguration("use_moveit")),
    )

    demo = Node(
        package="arachne_sim",
        executable="urban_trash_sorting_demo",
        name="urban_trash_sorting_demo",
        parameters=[
            {
                "planner_id": LaunchConfiguration("planner_id"),
                "playback_speed": ParameterValue(
                    LaunchConfiguration("playback_speed"), value_type=float
                ),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "patrol_pattern": LaunchConfiguration("patrol_pattern"),
                "patrol_distance_m": ParameterValue(
                    LaunchConfiguration("patrol_distance_m"), value_type=float
                ),
                "patrol_box_width_m": ParameterValue(
                    LaunchConfiguration("patrol_box_width_m"), value_type=float
                ),
                "patrol_box_height_m": ParameterValue(
                    LaunchConfiguration("patrol_box_height_m"), value_type=float
                ),
                "patrol_entry_m": ParameterValue(
                    LaunchConfiguration("patrol_entry_m"), value_type=float
                ),
                "show_keepout_markers": ParameterValue(
                    LaunchConfiguration("show_keepout_markers"), value_type=bool
                ),
                "slam_map_yaml": LaunchConfiguration("slam_map_yaml"),
                "map_frame_id": LaunchConfiguration("map_frame_id"),
                "trash_seed": ParameterValue(LaunchConfiguration("trash_seed"), value_type=int),
                "trash_count": ParameterValue(LaunchConfiguration("trash_count"), value_type=int),
                "synthetic_grasp_benchmark": ParameterValue(
                    LaunchConfiguration("synthetic_grasp_benchmark"), value_type=bool
                ),
                "synthetic_grasp_trials": ParameterValue(
                    LaunchConfiguration("synthetic_grasp_trials"), value_type=int
                ),
                "synthetic_ground_z_m": ParameterValue(
                    LaunchConfiguration("synthetic_ground_z_m"), value_type=float
                ),
                "scan_arc_radius_m": ParameterValue(
                    LaunchConfiguration("scan_arc_radius_m"), value_type=float
                ),
                "scan_arc_angle_deg": ParameterValue(
                    LaunchConfiguration("scan_arc_angle_deg"), value_type=float
                ),
                "scan_arc_samples": ParameterValue(
                    LaunchConfiguration("scan_arc_samples"), value_type=int
                ),
                "scan_cycle_duration_sec": ParameterValue(
                    LaunchConfiguration("scan_cycle_duration_sec"), value_type=float
                ),
                "detection_lock_frames": ParameterValue(
                    LaunchConfiguration("detection_lock_frames"), value_type=int
                ),
            }
        ],
        output="screen",
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="urban_trash_map_to_odom",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            LaunchConfiguration("map_frame_id"),
            "--child-frame-id",
            "odom",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="arachne_urban_trash_sorting_demo_rviz",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(LaunchConfiguration("launch_demo_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("playback_speed", default_value="0.85"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("patrol_pattern", default_value="line"),
            DeclareLaunchArgument("patrol_distance_m", default_value="1.5"),
            DeclareLaunchArgument("patrol_box_width_m", default_value="1.0"),
            DeclareLaunchArgument("patrol_box_height_m", default_value="1.2"),
            DeclareLaunchArgument("patrol_entry_m", default_value="0.3"),
            DeclareLaunchArgument("show_keepout_markers", default_value="false"),
            DeclareLaunchArgument("slam_map_yaml", default_value=str(default_slam_map)),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("trash_seed", default_value="26"),
            DeclareLaunchArgument("trash_count", default_value="10"),
            DeclareLaunchArgument("synthetic_grasp_benchmark", default_value="false"),
            DeclareLaunchArgument("synthetic_grasp_trials", default_value="60"),
            DeclareLaunchArgument("synthetic_ground_z_m", default_value="-0.22"),
            DeclareLaunchArgument("scan_arc_radius_m", default_value="0.32"),
            DeclareLaunchArgument("scan_arc_angle_deg", default_value="72.0"),
            DeclareLaunchArgument("scan_arc_samples", default_value="9"),
            DeclareLaunchArgument("scan_cycle_duration_sec", default_value="4.2"),
            DeclareLaunchArgument("detection_lock_frames", default_value="1"),
            DeclareLaunchArgument("launch_demo_rviz", default_value="true"),
            display,
            moveit,
            map_to_odom,
            demo,
            rviz,
        ]
    )
