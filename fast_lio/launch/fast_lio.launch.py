import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('fast_lio'), 'config', 'l1_lidar.yaml')
    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='fast_lio_node',
            name='fast_lio',
            output='screen',
            parameters=[cfg],
        )
    ])
