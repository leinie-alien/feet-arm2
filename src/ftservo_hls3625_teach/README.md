# ftservo_hls3625_teach

Minimal self-contained ROS 2 teaching package for FEETECH HLS3625 bus servos.

## What is included

- `bus_state_publisher`: reads the bus and publishes calibrated `sensor_msgs/msg/JointState`
- `capture_zero_offsets`: samples the current raw positions and writes a shared ROS 2 YAML config
- `set_torque`: enables or disables torque so the arm can be moved by hand
- `capture_pose`: records the current raw multi-servo pose into a small YAML file
- `play_pose`: replays a recorded raw pose back to the same set of servos

The package is self-contained and can be copied into any ROS 2 workspace under `src/`.

## Fixed device name

To pin the FEETECH USB serial adapter to a stable short name, install the included udev rule:

```bash
sudo cp src/ftservo_hls3625_teach/udev/99-ftservo-hls3625.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After replugging the adapter, the port will be available as:

```bash
/dev/ftservo_hls3625
```

The default package config already uses this fixed path.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ftservo_hls3625_teach --base-paths src/ftservo_hls3625_teach
source install/setup.bash
```

If the package is already inside a normal workspace `src/`, a regular `colcon build --packages-select ftservo_hls3625_teach` also works.

## Shared config

Start from `config/servo_bus.yaml` and set at least:

- `port`
- `ids`
- `joint_names`
- `directions`
- `zero_offsets_raw`

You can generate a fresh config from the current arm pose:

```bash
ros2 run ftservo_hls3625_teach capture_zero_offsets \
  --ros-args \
  --params-file src/ftservo_hls3625_teach/config/servo_bus.yaml \
  -p output_yaml:=./servo_bus.generated.yaml
```

## Minimal teaching loop

1. Disable torque so the arm can be posed by hand:

```bash
ros2 run ftservo_hls3625_teach set_torque \
  --ros-args \
  --params-file ./servo_bus.generated.yaml \
  -p torque_enabled:=false
```

2. Move the arm by hand, then capture the pose:

```bash
ros2 run ftservo_hls3625_teach capture_pose \
  --ros-args \
  --params-file ./servo_bus.generated.yaml \
  -p pose_file:=./pick_pose.yaml
```

3. Re-enable torque:

```bash
ros2 run ftservo_hls3625_teach set_torque \
  --ros-args \
  --params-file ./servo_bus.generated.yaml \
  -p torque_enabled:=true
```

4. Replay the saved pose:

```bash
ros2 run ftservo_hls3625_teach play_pose \
  --ros-args \
  --params-file ./servo_bus.generated.yaml \
  -p pose_file:=./pick_pose.yaml
```

## Joint state publisher

```bash
ros2 launch ftservo_hls3625_teach state_publisher.launch.py \
  config:=./servo_bus.generated.yaml
```

## Notes

- `capture_pose` and `play_pose` use raw servo positions to avoid calibration drift during teaching.
- `bus_state_publisher` applies `zero_offsets_raw` and `directions` before publishing `JointState`.
- `move_speed_raw` and `move_acc_raw` in the config control playback aggressiveness.
- The current implementation assumes the recorded pose file uses the same servo id ordering as the active config.
