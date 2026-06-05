# dm_motor_sdk_ros

`dm_motor_sdk_ros` 是机械臂 5 轴达妙电机的 ROS2 驱动包。

它负责三件事：

- 通过 `USB2CANFD_Dual` 收发达妙电机 CAN/CANFD 帧
- 把底层电机反馈转换成 ROS 关节状态 `/arm2/_lowState/joint`
- 把上层关节命令 `/arm2/_lowCmd/command` 转换成底层 MIT 控制命令

当前驱动实现不是通用多圈关节展开器，而是针对这套 5 轴机械臂的实际硬件行为做了明确建模。本文档描述当前代码采用的电机模型、坐标约定和各轴处理规则。

## 1. 包结构

- 传输层：`third_party/dm_sdk/.../libdm_device.so`
- 协议层：`src/dm_motor_drv.c`、`src/dm_motor_ctrl.c`
- ROS 驱动节点：`src/dm_motor_robot_driver_node.cpp`
- 启动文件：`launch/dm_motor_robot_driver.launch.py`
- 参数文件：`config/dm_motor_robot_driver.yaml`

## 2. 硬件位置模型

### 2.1 编码器本体

- 电机编码器本体是单圈绝对值编码器
- 单圈原始角度可理解为 `[-pi, pi]`

### 2.2 驱动器上电后的反馈行为

虽然编码器本体是单圈绝对值，但电机硬件驱动器在上电后会自行“记圈”，因此 ROS 驱动读到的 `feedback.pos` 应按下面方式理解：

- `feedback.pos` 是硬件已经累计过圈数后的连续位置
- 该连续位置在本次上电周期内有效
- 掉电后记圈信息丢失，重新上电需要重新建立参考

这意味着：

- ROS 层**不需要**再对 1 轴做运行时连续展开
- ROS 层只需要对 2 轴做一次启动时选圈

### 2.3 底层位置映射范围

底层协议层用 `PMAX` 做位置编解码范围，默认位置映射范围约为 `[-12.5, 12.5] rad`，见：

- 反馈解码：`src/dm_motor_drv.c` 中 `dm_motor_fbdata()`
- 命令编码：`src/dm_motor_drv.c` 中 `mit_ctrl()`

因此 `feedback.pos` 本身就不是按 `[-pi, pi]` 截断解释的。

## 3. ROS 坐标变换

每个轴的反馈统一先做一次基础变换：

```text
q_base = dir * feedback.pos - zero_offset
dq_ros = dir * feedback.vel
tau_ros = dir * feedback.tor
```

其中：

- `dir`：由 `inverted_motor_ids` 决定，正常方向为 `+1`，反向电机为 `-1`
- `zero_offset`：`joint_zero_offsets[i]`

`zero_offset` 的用途只有一个：

- 对硬件零点和 ROS 零点之间的残余误差做微调

它**不用于判圈**，也**不用于多圈展开**。

## 4. 各轴处理规则

当前驱动写死为 5 轴机械臂规则。

### 4.1 1 轴

1 轴的前提：

- 上电时实际姿态保证落在 `[-pi, pi]`
- 硬件零点已经与 ROS 零点对齐

因此 1 轴处理非常直接：

- 不做判圈
- 不做连续展开
- 只做微调和硬限位

ROS 限位：

```text
[-240 deg, 240 deg] = [-4*pi/3, 4*pi/3]
```

这样做的目的主要是防止 1 轴过转扯线。

### 4.2 2 轴

2 轴有机械限位，ROS 工作范围约为：

```text
[0.0, 3.5] rad
```

2 轴的特殊点在于：

- 掉电重上电后，底层硬件连续位置参考可能与 ROS 期望圈差一圈
- 这个问题只需要在启动时处理一次

驱动在拿到首帧有效反馈后执行一次启动选圈：

- 如果 `q_base` 落在 `[0.0, 3.5]`
  - 本圈就是 ROS 期望圈
  - `bias = 0`
- 如果 `q_base` 落在 `[-2*pi, 3.5 - 2*pi]`
  - 即近似 `[-6.283185, -2.783185]`
  - 说明 ROS 期望圈比当前硬件反馈多一圈
  - `bias = +2*pi`
- 其他区间
  - 视为异常启动姿态
  - 驱动会报警并回退为 `bias = 0`

之后在整个上电周期内固定使用：

```text
q_ros = clamp(q_base + bias, 0.0, 3.5)
```

注意：

- 2 轴是“启动一次性选圈”
- 不是“每帧动态判圈”

### 4.3 3 / 4 / 5 轴

这三个轴活动范围都不大，不需要判圈或展开，只做硬限位：

- 3 轴：`[-3.0, 0.15]`
- 4 轴：`[-2.0, 1.57]`
- 5 轴：`[-pi, pi]`

## 5. 命令侧变换

上层给到驱动的是 ROS 关节角 `q_cmd`。

驱动先对 ROS 命令做每轴限位，然后再反算到底层位置设定值：

```text
pos_set = dir * (clamp(q_cmd) - bias + zero_offset)
```

解释：

- 1 轴：`bias = 0`
- 2 轴：`bias` 为启动时确定的 `0` 或 `2*pi`
- 3/4/5 轴：`bias = 0`

这样保证反馈侧和命令侧是对称的：

- 反馈：`q_base + bias`
- 命令：`q_cmd - bias`

## 6. 反馈发布与跳变检测

驱动节点周期性发布：

- 话题：`/arm2/_lowState/joint`
- 类型：`robot_msgs/msg/RobotState`

每个电机状态包含：

- `q`
- `dq`
- `tau_est`
- `valid`

当前位置跳变检测已经按“连续位置”语义处理，不再使用基于回卷角的最短角距离，而是直接比较当前发布值与上一帧有效值的普通差值：

```text
abs(q_now - q_last)
```

阈值由参数控制：

- `q_jump_threshold_rad`

如果超过阈值：

- 本帧状态回退到上一帧有效状态
- `valid = false`

## 7. 当前硬编码限位

驱动当前内置的五轴 ROS 限位如下：

| 轴号 | ROS 限位 |
| --- | --- |
| 1 | `[-4.188790, 4.188790]` |
| 2 | `[0.0, 3.5]` |
| 3 | `[-3.0, 0.15]` |
| 4 | `[-2.0, 1.57]` |
| 5 | `[-3.141593, 3.141593]` |

这些限位已经在驱动内部同时作用于：

- 反馈发布
- 命令下发

## 8. 关键参数

`config/dm_motor_robot_driver.yaml` 当前仍保留的主要参数：

- `motor_ids`
- `feedback_ids`
- `inverted_motor_ids`
- `joint_zero_offsets`
- `motor_pmax`
- `motor_vmax`
- `motor_tmax`
- `feedback_timeout_ms`
- `command_timeout_ms`
- `q_jump_threshold_rad`
- `temperature_topic`
- `temperature_publish_hz`

已经移除的旧参数：

- `joint1_angle_window`
- `joint2_publish_window`

原因是旧版本逻辑会在 ROS 层再做 1/2 轴的 `q ± 2pi` 动态选圈；现在这套逻辑已经被更明确的轴规则替代。

## 9. 启动流程

驱动节点启动后按以下流程工作：

1. 打开 CAN 设备
2. 初始化底层电机对象
3. 发送电机使能命令
4. 等待每个电机首帧反馈
5. 对 2 轴执行一次启动选圈，确定 `bias`
6. 开始周期性收发控制与状态

如果首帧反馈迟迟收不到：

- 节点会重复补发使能
- 超过重试次数后抛异常退出

## 10. 运行与编译

### 编译

```bash
cd <your_ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select dm_motor_sdk_ros
source install/setup.bash
```

### 启动

```bash
ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py
```

如需指定参数文件：

```bash
ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py \
  params_path:=/absolute/path/to/dm_motor_robot_driver.yaml
```

## 11. 代码定位

和本文档最相关的代码位置：

- 反馈解码：`src/dm_motor_drv.c`
- 电机对象与 CAN 接收分发：`src/dm_motor_ctrl.c`
- ROS 侧轴规则、限位、2 轴启动选圈：`src/dm_motor_robot_driver_node.cpp`

如果后续修改轴规则，优先同步更新：

- `src/dm_motor_robot_driver_node.cpp`
- `config/dm_motor_robot_driver.yaml`
- 本 README
