# dm_motor_sdk_ros (ROS2 Humble)

A minimal ROS2 (`ament_cmake`) package for Linux + USB2CANFD_Dual.

## Design

- Transport layer: `libdm_device.so` (dynamic link)
- Protocol layer: original `dm_motor_drv` + `dm_motor_ctrl` (STM32 style)
- Adapter layer: `can_bsp_sdk.cpp` implements `canx_send_data/canx_receive/hcan1`

## Build

Place package into your ROS2 workspace `src/`, then:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dm_motor_sdk_ros
source install/setup.bash
```

## Run

```bash
ros2 launch dm_motor_sdk_ros dm_motor_minimal.launch.py
```

## Topics

- Subscribe: `/dm_motor/pos_cmd` (`std_msgs/msg/Float32MultiArray`)
  - up to 3 values -> target position for motor1..motor3
- Publish: `/dm_motor/feedback` (`std_msgs/msg/Float32MultiArray`)
  - repeated tuple: `[id, pos, vel, tor]`

## Notes

- Default channel is 1 for USB2CANFD_Dual.
- Package ships `third_party/dm_sdk/linux/libdm_device.so` and links it directly.
