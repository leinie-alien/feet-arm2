from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('ftservo_hls3625_teach')
    default_config = os.path.join(package_share, 'config', 'servo_bus.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the shared servo bus config YAML',
    )

    node = Node(
        package='ftservo_hls3625_teach',
        executable='bus_state_publisher',
        name='bus_state_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([
        config_arg,
        node,
    ])
