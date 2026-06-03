cmake_minimum_required(VERSION 3.8)
project(suction_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

add_executable(suction_service_node
  src/suction_service_node.cpp
)
target_compile_features(suction_service_node PUBLIC cxx_std_17)
ament_target_dependencies(suction_service_node
  rclcpp
  robot_msgs
)

add_executable(suction_keyboard_client
  src/suction_keyboard_client.cpp
)
target_compile_features(suction_keyboard_client PUBLIC cxx_std_17)
ament_target_dependencies(suction_keyboard_client
  rclcpp
  robot_msgs
)

install(TARGETS
  suction_service_node
  suction_keyboard_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
