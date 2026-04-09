import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "dm_motor_sdk_ros"
    pkg_share = get_package_share_directory(package_name)
    default_driver_config = os.path.join(pkg_share, "config", "dm_motor_robot_driver.yaml")
    default_stress_config = os.path.join(pkg_share, "config", "dm_motor_robot_stress.yaml")

    driver_params_arg = DeclareLaunchArgument(
        "driver_params",
        default_value=default_driver_config,
        description="Full path to the driver parameter file",
    )

    stress_params_arg = DeclareLaunchArgument(
        "stress_params",
        default_value=default_stress_config,
        description="Full path to the stress-node parameter file",
    )

    drive_node = Node(
        package=package_name,
        executable="dm_motor_robot_driver_node",
        name="dm_motor_driver_node",
        output="screen",
        parameters=[LaunchConfiguration("driver_params")],
        emulate_tty=True,
    )

    stress_node = Node(
        package=package_name,
        executable="dm_motor_robot_stress_node",
        name="dm_motor_robot_stress_node",
        output="screen",
        parameters=[LaunchConfiguration("stress_params")],
        emulate_tty=True,
    )

    return LaunchDescription([
        driver_params_arg,
        stress_params_arg,
        drive_node,
        stress_node,
    ])
