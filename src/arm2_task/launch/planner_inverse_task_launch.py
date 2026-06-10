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

    trajectory_planner_node = Node(
        package=package_name,
        executable="trajectory_planner_node",
        name="trajectory_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("params_path")],
        namespace="",
        emulate_tty=True,
    )

    inverse_dynamics_node = Node(
        package=package_name,
        executable="inverse_dynamics_node",
        name="inverse_dynamics_node",
        output="screen",
        parameters=[LaunchConfiguration("params_path")],
        namespace="",
        emulate_tty=True,
    )

    xterm_prefix = (
        "xterm -hold -u8 -T 'Task Manager Control Panel' "
        "-fn '-misc-fixed-medium-r-normal--18-120-100-100-c-90-iso10646-1' -e"
    )

    task_node = Node(
        package=package_name,
        executable="task_node",
        name="task_node",
        output="screen",
        parameters=[LaunchConfiguration("params_path")],
        namespace="",
        prefix=xterm_prefix,
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            params_path_arg,
            trajectory_planner_node,
            inverse_dynamics_node,
            task_node,
        ]
    )
