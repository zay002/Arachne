from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detection_topic", default_value="/arachne/perception/taco_instances"
            ),
            DeclareLaunchArgument(
                "restart_search_topic", default_value="/arachne/grasp_preview/restart_search"
            ),
            DeclareLaunchArgument(
                "real_search_scan_control_topic",
                default_value="/arachne/grasp_preview/real_search_scan",
            ),
            DeclareLaunchArgument(
                "base_command_topic", default_value="/arachne/grasp_task/base_command"
            ),
            DeclareLaunchArgument("base_state_topic", default_value="/arachne/grasp_task/base_state"),
            DeclareLaunchArgument("patrol_pattern", default_value="line"),
            DeclareLaunchArgument("patrol_distance_m", default_value="1.5"),
            DeclareLaunchArgument("patrol_step_m", default_value="0.20"),
            DeclareLaunchArgument("patrol_box_width_m", default_value="1.0"),
            DeclareLaunchArgument("patrol_box_height_m", default_value="1.2"),
            DeclareLaunchArgument("patrol_entry_m", default_value="0.3"),
            DeclareLaunchArgument("patrol_base_speed_mps", default_value="0.08"),
            DeclareLaunchArgument("max_round_trips", default_value="1"),
            DeclareLaunchArgument("detection_confidence", default_value="0.08"),
            DeclareLaunchArgument("detection_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("initial_detection_wait_sec", default_value="0.0"),
            DeclareLaunchArgument("require_3d_candidate", default_value="true"),
            DeclareLaunchArgument("candidate_min_base_x_m", default_value="0.25"),
            DeclareLaunchArgument("candidate_max_base_x_m", default_value="1.03"),
            DeclareLaunchArgument("candidate_max_abs_base_y_m", default_value="0.60"),
            DeclareLaunchArgument("candidate_min_base_z_m", default_value="-0.18"),
            DeclareLaunchArgument("candidate_max_reach_m", default_value="1.03"),
            DeclareLaunchArgument("candidate_max_depth_m", default_value="0.85"),
            DeclareLaunchArgument("patrol_turn_scale", default_value="1.0"),
            DeclareLaunchArgument("base_step_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("grasp_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("reach_recovery_enabled", default_value="true"),
            DeclareLaunchArgument("reach_recovery_max_attempts", default_value="3"),
            DeclareLaunchArgument("reach_recovery_step_m", default_value="0.10"),
            DeclareLaunchArgument("reach_recovery_wait_detection_sec", default_value="3.0"),
            DeclareLaunchArgument("reach_recovery_continue_on_exhausted", default_value="true"),
            DeclareLaunchArgument("auto_return_home_on_empty_route", default_value="true"),
            DeclareLaunchArgument("loop", default_value="false"),
            Node(
                package="arachne_operator",
                executable="road_cleanup_task_server",
                name="arachne_road_cleanup_task_server",
                output="screen",
                parameters=[
                    {
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "restart_search_topic": LaunchConfiguration("restart_search_topic"),
                        "real_search_scan_control_topic": LaunchConfiguration(
                            "real_search_scan_control_topic"
                        ),
                        "base_command_topic": LaunchConfiguration("base_command_topic"),
                        "base_state_topic": LaunchConfiguration("base_state_topic"),
                        "patrol_pattern": LaunchConfiguration("patrol_pattern"),
                        "patrol_distance_m": ParameterValue(
                            LaunchConfiguration("patrol_distance_m"), value_type=float
                        ),
                        "patrol_step_m": ParameterValue(
                            LaunchConfiguration("patrol_step_m"), value_type=float
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
                        "patrol_base_speed_mps": ParameterValue(
                            LaunchConfiguration("patrol_base_speed_mps"), value_type=float
                        ),
                        "max_round_trips": ParameterValue(
                            LaunchConfiguration("max_round_trips"), value_type=int
                        ),
                        "detection_confidence": ParameterValue(
                            LaunchConfiguration("detection_confidence"), value_type=float
                        ),
                        "detection_timeout_sec": ParameterValue(
                            LaunchConfiguration("detection_timeout_sec"), value_type=float
                        ),
                        "initial_detection_wait_sec": ParameterValue(
                            LaunchConfiguration("initial_detection_wait_sec"), value_type=float
                        ),
                        "require_3d_candidate": ParameterValue(
                            LaunchConfiguration("require_3d_candidate"), value_type=bool
                        ),
                        "candidate_min_base_x_m": ParameterValue(
                            LaunchConfiguration("candidate_min_base_x_m"), value_type=float
                        ),
                        "candidate_max_base_x_m": ParameterValue(
                            LaunchConfiguration("candidate_max_base_x_m"), value_type=float
                        ),
                        "candidate_max_abs_base_y_m": ParameterValue(
                            LaunchConfiguration("candidate_max_abs_base_y_m"), value_type=float
                        ),
                        "candidate_min_base_z_m": ParameterValue(
                            LaunchConfiguration("candidate_min_base_z_m"), value_type=float
                        ),
                        "candidate_max_reach_m": ParameterValue(
                            LaunchConfiguration("candidate_max_reach_m"), value_type=float
                        ),
                        "candidate_max_depth_m": ParameterValue(
                            LaunchConfiguration("candidate_max_depth_m"), value_type=float
                        ),
                        "patrol_turn_scale": ParameterValue(
                            LaunchConfiguration("patrol_turn_scale"), value_type=float
                        ),
                        "base_step_timeout_sec": ParameterValue(
                            LaunchConfiguration("base_step_timeout_sec"), value_type=float
                        ),
                        "grasp_timeout_sec": ParameterValue(
                            LaunchConfiguration("grasp_timeout_sec"), value_type=float
                        ),
                        "reach_recovery_enabled": ParameterValue(
                            LaunchConfiguration("reach_recovery_enabled"), value_type=bool
                        ),
                        "reach_recovery_max_attempts": ParameterValue(
                            LaunchConfiguration("reach_recovery_max_attempts"), value_type=int
                        ),
                        "reach_recovery_step_m": ParameterValue(
                            LaunchConfiguration("reach_recovery_step_m"), value_type=float
                        ),
                        "reach_recovery_wait_detection_sec": ParameterValue(
                            LaunchConfiguration("reach_recovery_wait_detection_sec"),
                            value_type=float,
                        ),
                        "reach_recovery_continue_on_exhausted": ParameterValue(
                            LaunchConfiguration("reach_recovery_continue_on_exhausted"),
                            value_type=bool,
                        ),
                        "auto_return_home_on_empty_route": ParameterValue(
                            LaunchConfiguration("auto_return_home_on_empty_route"),
                            value_type=bool,
                        ),
                        "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                    }
                ],
            ),
        ]
    )
