from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='TCP server IP'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='30004',
            description='TCP server port'
        ),
        DeclareLaunchArgument(
            'robot', 
            default_value='rob1'
        ),
        DeclareLaunchArgument(
            'log_level', 
            default_value='info'
        ),
        Node(
            package='aubo_ros2_driver',
            executable='aubo_client_node.py',
            name='aubo_client',
            output='screen',
            parameters=[{
                'tcp_client.ip': LaunchConfiguration('robot_ip'),
                'tcp_client.port': LaunchConfiguration('port'),
                'tcp_client.robot_prefix': LaunchConfiguration('robot'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
