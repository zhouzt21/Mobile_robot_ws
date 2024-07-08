from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracer_base',
            executable='tracer_base_node',
            name='tracer_base_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[
                {'port_name': 'can0'},
                {'simulated_robot': 'false'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
            ],
        ),
    ])