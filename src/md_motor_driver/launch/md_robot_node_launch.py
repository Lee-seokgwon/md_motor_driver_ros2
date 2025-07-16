from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='md_motor_driver',
            executable='md_motor_driver_node',
            name='md_motor_driver_node',
            output='screen',
            parameters=[{
                'use_MDUI': 0,
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 57600,
                'wheel_radius': 0.0935,
                'wheel_length': 0.454,
                'reduction': 30,
                'reverse_direction': 0,
                'maxrpm': 1000,
                'enable_encoder': 0,
                'encoder_PPR': 900,
                'slow_start': 300,
                'slow_down': 300,
            }]
        )
    ])
