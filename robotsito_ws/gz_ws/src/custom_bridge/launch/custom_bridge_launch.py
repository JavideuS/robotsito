from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('custom_bridge')
    # Construct the path to the YAML file
    yaml_path = os.path.join(package_dir, 'config', 'custom_bridge.yaml')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='custom_bridge',
            output='screen',
            parameters=[
                {'config_file': yaml_path}
            ]
        )
    ])
