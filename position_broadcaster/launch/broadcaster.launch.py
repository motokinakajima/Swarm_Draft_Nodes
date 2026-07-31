import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('position_broadcaster')

    config_file = os.path.join(pkg_share, 'config', 'position_config.yaml')

    return LaunchDescription([
        Node(
            package='position_broadcaster',
            executable='broadcaster_node',
            name='position_broadcaster_node',
            output='screen',
            parameters=[config_file]
        )
    ])
