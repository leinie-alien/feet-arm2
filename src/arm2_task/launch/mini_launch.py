import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包的路径
    package_name = 'arm2_task'
    pkg_share = get_package_share_directory(package_name)

    # 2. 定义可配置的参数路径 (可以通过 ros2 launch ... params_path:=/new/path 修改)
    default_config_path = os.path.join(pkg_share, 'config', 'params.yaml')
    
    params_path_arg = DeclareLaunchArgument(
        'params_path',
        default_value=default_config_path,
        description='Full path to the ROS2 parameters file to use'
    )

    # 3. 定义控制节点 (Control Node)
    # 注意：确保 executable 名字与 CMakeLists.txt 中的 add_executable 一致
    control_node = Node(
        package=package_name,
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[LaunchConfiguration('params_path')],
        # 强制将节点置于根命名空间，确保匹配 YAML 中的 /**
        namespace='', 
        emulate_tty=True
    )

    # 4. 定义任务管理节点 (Task Node)
    # 使用 -hold 确保报错时窗口不闪退
    xterm_prefix = (
        "xterm -hold -u8 -T 'Task Manager Control Panel' "
        "-fn '-misc-fixed-medium-r-normal--18-120-100-100-c-90-iso10646-1' -e"
    )

    task_node = Node(
        package=package_name,
        executable='task_node',
        name='task_node', 
        output='screen',
        parameters=[LaunchConfiguration('params_path')],
        namespace='',
        prefix=xterm_prefix
    )

    # 5. 返回启动描述
    return LaunchDescription([
        params_path_arg,
        control_node,
        task_node
    ])
