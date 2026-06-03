from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(get_package_share_directory("suction_serial_bridge")) / "config" / "suction_service.yaml"
    return LaunchDescription([
        Node(
            package="suction_serial_bridge",
            executable="suction_service_node",
            name="suction_service_node",
            output="screen",
            parameters=[str(config_path)],
        )
    ])
