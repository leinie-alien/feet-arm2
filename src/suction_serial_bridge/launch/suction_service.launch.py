from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="suction_serial_bridge",
            executable="suction_service_node",
            name="suction_service_node",
            output="screen",
            parameters=["/home/primarymage/WorkFile/esp_ws/ros2_suction_ws/src/suction_serial_bridge/config/suction_service.yaml"],
        )
    ])
