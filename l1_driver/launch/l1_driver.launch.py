from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate',    default_value='2000000'),

        Node(
            package='l1_driver',
            executable='l1_driver_node',
            name='l1_driver_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate':    LaunchConfiguration('baudrate'),
                'lidar_frame': 'lidar_link',
                'imu_frame':   'imu_link',
            }],
        ),
    ])
