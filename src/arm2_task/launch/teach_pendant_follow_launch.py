import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "arm2_task"
    pkg_share = get_package_share_directory(package_name)
    default_config_path = os.path.join(pkg_share, "config", "params.yaml")

    params_path_arg = DeclareLaunchArgument(
        "params_path",
        default_value=default_config_path,
        description="Full path to the ROS2 parameters file to use",
    )

    teach_pendant_follow_node = Node(
        package=package_name,
        executable="teach_pendant_follow_node",
        name="teach_pendant_follow_node",
        output="screen",
        parameters=[LaunchConfiguration("params_path")],
        namespace="",
        emulate_tty=True,
    )

    return LaunchDescription([
        params_path_arg,
        teach_pendant_follow_node,
    ])
