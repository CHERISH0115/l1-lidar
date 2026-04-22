from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('baudrate',    default_value='9600'),

        Node(
            package='lora_bridge',
            executable='lora_bridge_node',
            name='lora_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate':    LaunchConfiguration('baudrate'),
            }],
        ),
    ])
