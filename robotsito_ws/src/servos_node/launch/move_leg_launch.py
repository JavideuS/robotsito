from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servos_node',
            executable='server',
            name='server'
        ),
        Node(
            package='servos_node',
            executable='client',
            name='client'
        )
    ])

