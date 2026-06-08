
cmake_minimum_required(VERSION 3.5)
project(robot_msgs VERSION 3.0.0)

if($ENV{ROS_DISTRO} MATCHES "noetic")
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
  )
  add_message_files(
    FILES
    MotorCommand.msg
    MotorState.msg
    RobotCommand.msg
    RobotState.msg
    IMU.msg
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )
  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
  )
  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(geometry_msgs REQUIRED) 

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
    "msg/MotorState.msg"
    "msg/RobotCommand.msg"
    "msg/RobotState.msg"
    "msg/IMU.msg"
    "srv/GetPickPos.srv"
    "srv/GetPlacePos.srv"
    "srv/SetSuction.srv"
    "srv/SetControllerMode.srv"
    "srv/GetPayloadEstimate.srv"
    "srv/SetPayloadState.srv"
    "action/MoveToPose.action"
    "action/MoveJoint.action"
    # 指定依赖项 [cite: 1]
    DEPENDENCIES geometry_msgs
  )
  ament_package()
endif()
