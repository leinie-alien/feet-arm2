cmake_minimum_required(VERSION 3.10)
project(dm_motor_sdk_ros)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(DM_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(DM_SDK_DIR "${DM_THIRD_PARTY_DIR}/dm_sdk/linux")
set(DM_RUNTIME_DIR "${DM_THIRD_PARTY_DIR}/runtime/linux")
set(DM_SDK_SO "${DM_SDK_DIR}/libdm_device.so")
set(DM_RUNTIME_LIBSTDCXX "${DM_RUNTIME_DIR}/libstdc++.so.6")
set(DM_RUNTIME_LIBGCC "${DM_RUNTIME_DIR}/libgcc_s.so.1")

if(NOT EXISTS ${DM_SDK_SO})
  message(FATAL_ERROR "libdm_device.so not found at ${DM_SDK_SO}")
endif()
if(NOT EXISTS ${DM_RUNTIME_LIBSTDCXX})
  message(FATAL_ERROR "libstdc++.so.6 not found at ${DM_RUNTIME_LIBSTDCXX}")
endif()
if(NOT EXISTS ${DM_RUNTIME_LIBGCC})
  message(FATAL_ERROR "libgcc_s.so.1 not found at ${DM_RUNTIME_LIBGCC}")
endif()

include_directories(
  include
  third_party/dm_sdk/include
)

add_library(dm_sdk_can_bsp
  src/can_bsp_sdk.cpp
)
target_link_libraries(dm_sdk_can_bsp
  ${DM_RUNTIME_LIBSTDCXX}
  ${DM_RUNTIME_LIBGCC}
  ${DM_SDK_SO}
  ${LIBUSB_LIBRARIES}
  pthread
)
target_include_directories(dm_sdk_can_bsp PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_options(dm_sdk_can_bsp INTERFACE
  "-Wl,-rpath-link,${DM_SDK_DIR}"
  "-Wl,-rpath-link,${DM_RUNTIME_DIR}"
)

add_library(dm_motor_protocol
  src/dm_motor_drv.c
  src/dm_motor_ctrl.c
)

add_executable(dm_motor_robot_driver_node
  src/dm_motor_robot_driver_node.cpp
)
ament_target_dependencies(dm_motor_robot_driver_node
  rclcpp
  robot_msgs
  std_msgs
)
target_link_libraries(dm_motor_robot_driver_node
  dm_motor_protocol
  dm_sdk_can_bsp
  ${LIBUSB_LIBRARIES}
)

add_executable(dm_motor_sdk_example
  src/dm_motor_sdk_example.cpp
)
target_link_libraries(dm_motor_sdk_example
  dm_motor_protocol
  dm_sdk_can_bsp
  ${LIBUSB_LIBRARIES}
)

add_executable(dm_motor_sdk_stress_test
  src/dm_motor_sdk_stress_test.cpp
)
target_link_libraries(dm_motor_sdk_stress_test
  dm_motor_protocol
  dm_sdk_can_bsp
  ${LIBUSB_LIBRARIES}
)

set_target_properties(dm_sdk_can_bsp dm_motor_robot_driver_node dm_motor_sdk_example dm_motor_sdk_stress_test PROPERTIES
  BUILD_RPATH "${DM_SDK_DIR};${DM_RUNTIME_DIR}"
  INSTALL_RPATH "\$ORIGIN;\$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS
  dm_sdk_can_bsp
  dm_motor_protocol
  dm_motor_robot_driver_node
  dm_motor_sdk_example
  dm_motor_sdk_stress_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  third_party/dm_sdk/linux/libdm_device.so
  DESTINATION lib
)

install(FILES
  third_party/runtime/linux/libstdc++.so.6
  third_party/runtime/linux/libstdc++.so.6.0.35
  third_party/runtime/linux/libgcc_s.so.1
  DESTINATION lib
)

install(FILES
  launch/dm_motor_robot_driver.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
