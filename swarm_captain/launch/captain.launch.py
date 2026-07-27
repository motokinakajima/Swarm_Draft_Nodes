import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_captain')
    
    config_file = os.path.join(pkg_share, 'config', 'captain_config.yaml')

    return LaunchDescription([
        Node(
            package='swarm_captain',
            executable='captain_node',
            name='swarm_captain_node',
            output='screen',
            parameters=[config_file]
        )
    ])