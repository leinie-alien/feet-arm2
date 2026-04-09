from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dm_motor_sdk_ros',
            executable='dm_motor_minimal_node',
            name='dm_motor_minimal_node',
            output='screen',
            parameters=[{
                'channel': 1,
                'can_baud': 1000000,
                'canfd_baud': 5000000,
                'loop_hz': 200.0,
            }],
        )
    ])
