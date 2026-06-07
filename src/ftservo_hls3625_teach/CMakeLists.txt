cmake_minimum_required(VERSION 3.8)
project(ftservo_hls3625_teach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ftservo_teach_core STATIC
  src/vendor/scservo/SCS.cpp
  src/vendor/scservo/SCSerial.cpp
  src/vendor/scservo/SMSCL.cpp
  src/servo_bus.cpp
  src/pose_file.cpp
)
target_include_directories(ftservo_teach_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vendor/scservo
)
target_compile_features(ftservo_teach_core PUBLIC cxx_std_17)
ament_target_dependencies(ftservo_teach_core PUBLIC rclcpp)

add_executable(bus_state_publisher
  src/bus_state_publisher.cpp
)
target_compile_features(bus_state_publisher PUBLIC cxx_std_17)
ament_target_dependencies(bus_state_publisher rclcpp sensor_msgs)
target_link_libraries(bus_state_publisher ftservo_teach_core)

add_executable(capture_zero_offsets
  src/capture_zero_offsets.cpp
)
target_compile_features(capture_zero_offsets PUBLIC cxx_std_17)
ament_target_dependencies(capture_zero_offsets rclcpp)
target_link_libraries(capture_zero_offsets ftservo_teach_core)

add_executable(capture_pose
  src/capture_pose.cpp
)
target_compile_features(capture_pose PUBLIC cxx_std_17)
ament_target_dependencies(capture_pose rclcpp)
target_link_libraries(capture_pose ftservo_teach_core)

add_executable(play_pose
  src/play_pose.cpp
)
target_compile_features(play_pose PUBLIC cxx_std_17)
ament_target_dependencies(play_pose rclcpp)
target_link_libraries(play_pose ftservo_teach_core)

add_executable(set_torque
  src/set_torque.cpp
)
target_compile_features(set_torque PUBLIC cxx_std_17)
ament_target_dependencies(set_torque rclcpp)
target_link_libraries(set_torque ftservo_teach_core)

install(TARGETS
  bus_state_publisher
  capture_zero_offsets
  capture_pose
  play_pose
  set_torque
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  poses
  udev
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
