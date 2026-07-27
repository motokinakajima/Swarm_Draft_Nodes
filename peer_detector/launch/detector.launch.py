import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('peer_detector')
    
    config_file = os.path.join(pkg_share, 'config', 'cameras_configs.yaml')

    return LaunchDescription([
        Node(
            package='peer_detector',
            executable='detector_node',
            name='peer_detector_node',
            output='screen',
            parameters=[config_file]
        )
    ])