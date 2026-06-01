cmake_minimum_required(VERSION 3.8)
project(arm2_task)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(robot_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 2. 包含头文件
include_directories(include)

# 3. 声明并配置核心算法库
add_library(arm2_algorithms SHARED  # 显式声明为 SHARED
  src/kinematics_engine.cpp 
  src/dynamics_manager.cpp
)
# 确保库链接了所有必要的第三方库
target_link_libraries(arm2_algorithms pinocchio::pinocchio)

# 编译控制器节点
add_executable(control_node src/control_node.cpp)
# 修改：先链接本地库，再处理 ament 依赖
target_link_libraries(control_node arm2_algorithms)

ament_target_dependencies(control_node 
  rclcpp 
  geometry_msgs 
  pinocchio 
  robot_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
)


# 编译任务管理节点
add_executable(task_node src/task_node.cpp)
target_link_libraries(task_node arm2_algorithms)
ament_target_dependencies(task_node 
  rclcpp 
  geometry_msgs 
  pinocchio 
  robot_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
)

add_executable(gravity_comp_test_node src/gravity_comp_test_node.cpp)
target_link_libraries(gravity_comp_test_node arm2_algorithms)
ament_target_dependencies(gravity_comp_test_node
  rclcpp
  pinocchio
  robot_msgs
  std_msgs
  ament_index_cpp
)

add_executable(inverse_dynamics_node src/inverse_dynamics_node.cpp)
target_link_libraries(inverse_dynamics_node arm2_algorithms)
ament_target_dependencies(inverse_dynamics_node
  rclcpp
  geometry_msgs
  pinocchio
  robot_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)
ament_target_dependencies(trajectory_planner_node
  rclcpp
  robot_msgs
  std_msgs
  rclcpp_action
)

add_executable(inverse_dynamics_target_test_node src/inverse_dynamics_target_test_node.cpp)
ament_target_dependencies(inverse_dynamics_target_test_node
  rclcpp
  pinocchio
  robot_msgs
  std_msgs
)

add_executable(teach_pendant_follow_node src/teach_pendant_follow_node.cpp)
ament_target_dependencies(teach_pendant_follow_node
  rclcpp
  pinocchio
  sensor_msgs
  robot_msgs
  std_msgs
)

add_executable(debug_node src/debug_node.cpp)
ament_target_dependencies(debug_node
  rclcpp
  robot_msgs
  std_msgs
)

# 5. 分类安装 (核心修复)
# 将库安装到标准的 lib/ 目录下
install(TARGETS arm2_algorithms
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 将可执行节点安装到 lib/${PROJECT_NAME} (ROS2 惯例)
install(TARGETS
  control_node
  task_node
  gravity_comp_test_node
  inverse_dynamics_node
  trajectory_planner_node
  inverse_dynamics_target_test_node
  teach_pendant_follow_node
  debug_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置与启动文件
install(DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# 6. 导出接口 (确保其他包能找到这个库)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs pinocchio robot_msgs std_msgs ament_index_cpp)

ament_package()
