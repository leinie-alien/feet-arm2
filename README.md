# feet-arm2 — 5-DOF 机械臂控制栈

> 基于 ROS2 Humble 的 5 轴机械臂控制项目，包含电机驱动、运动控制（FK/IK/逆动力学）、
> 任务编排（抓取/放置/叠放）、视觉感知接口、吸盘控制和示教器功能。

---

## 硬件概览

| 组件 | 型号 | 说明 |
|---|---|---|
| 机械臂 | 5-DOF 串联臂 | Yaw + Pitch1 + Pitch2 + Pitch3 + Roll |
| 电机 ×5 | 达妙（Damiao） | CAN/CANFD 通信，通过 USB2CANFD_Dual 适配器连接 |
| 末端执行器 | 吸盘 | ESP32-C3 控制，串口通信 |
| 手眼相机 | — | 安装在 Link_4，用于抓取目标检测 |
| 狗头相机 | — | 固定安装，用于放置目标检测 |
| 示教舵机 | FEETECH HLS3625 | 可选，用于拖拽示教 |

---

## 软件架构

```
┌─────────────────────────────────────────────────┐
│  task_node (arm2_task)                           │
│  任务编排：抓取 / 放置 / 叠放 / 视觉对齐          │
│  远程控制：/arm/cmd ← /arm/status →              │
└──────────────────┬──────────────────────────────┘
                   │ joint_target + service calls
┌──────────────────▼──────────────────────────────┐
│  control_node (arm2_task)                        │
│  逆动力学 + 摩擦力补偿 + PD 控制                  │
│  TF 广播（FK + 静态外参）                         │
│  控制器模式切换 (idle/moving/loaded/gravity_comp) │
└──────────────────┬──────────────────────────────┘
                   │ /arm2/_lowCmd/command
┌──────────────────▼──────────────────────────────┐
│  dm_motor_sdk_ros                                │
│  CAN 帧收发 · 关节状态发布 · 限位保护             │
│  2 轴启动选圈 · 跳变检测                          │
└──────────────────┬──────────────────────────────┘
                   │ USB2CANFD_Dual
┌──────────────────▼──────────────────────────────┐
│  达妙电机 ×5（物理层）                            │
└─────────────────────────────────────────────────┘
```

### 辅助节点

| 节点 | 包 | 作用 |
|---|---|---|
| `suction_service_node` | `suction_serial_bridge` | 串口控制吸盘开关 |
| `bus_state_publisher` | `ftservo_hls3625_teach` | 示教舵机状态发布 |
| 感知服务 | neweyes workspace | `get_pick_pos` / `get_place_pos` / `get_stack_pos` |

---

## 项目结构

```
feet-arm2/
├── run_arm.sh                        # 一键启动脚本
├── README.md
├── LICENSE
├── .ros_domain_id.env                # ROS_DOMAIN_ID=91
├── .gitignore
├── scripts/
│   ├── install_esp32_suction_udev.sh # 吸盘串口 udev 规则
│   ├── run_suction_receiver.sh       # 吸盘接收端（ESP32 侧）
│   ├── tail_task_log.sh              # 查看 task_node 日志
│   └── debug/                        # 调试用启动脚本
├── tools/
│   └── remote_control_test/          # 远程控制测试工具 + 接口文档
└── src/
    ├── robot_msgs/                   # 自定义消息/服务/动作接口
    ├── arm2_task/                    # 主控包（task + control + 运动规划）
    ├── dm_motor_sdk_ros/             # 达妙电机 ROS2 驱动
    ├── suction_serial_bridge/        # 吸盘串口桥接
    ├── ftservo_hls3625_teach/        # 示教器包
    └── DM_DeviceSDK/                 # 达妙官方 SDK（第三方）
```

---

## 环境要求

| 依赖 | 版本/说明 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble Hawksbill |
| 编译器 | GCC 支持 C++17 |
| Pinocchio | 刚体动力学库（用于逆动力学） |
| 达妙 SDK | `src/dm_motor_sdk_ros/third_party/`（需自行下载） |
| 达妙 DeviceSDK | `src/DM_DeviceSDK/`（需自行下载） |

编译前需要 source nav 工作区（如果使用 navigation 集成）：
```bash
source /opt/ros/humble/setup.bash
source ~/task/nav_ws/install/setup.bash   # 可选，navigation 集成
```

---

## 编译

```bash
cd feet-arm2
source /opt/ros/humble/setup.bash

# 完整编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或只编译核心包
colcon build --packages-select robot_msgs suction_serial_bridge dm_motor_sdk_ros arm2_task \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 启动

### 真机模式

```bash
bash run_arm.sh
```

脚本自动完成：
1. 安装达妙 USB udev 规则（首次）
2. 启动 `dm_motor_sdk_ros` 驱动，等待 `/robot_driver/ready`
3. 启动 `control_node`（逆动力学 + TF 广播）
4. 启动吸盘节点（如果设备存在）
5. 启动 `task_node`（交互菜单 / 远程控制）

### 仿真模式

```bash
# 终端 1：启动 MuJoCo 仿真器
bash ~/data/robotics/arm_mujuco_ws/sim_arm.sh

# 终端 2：启动控制栈
bash run_arm.sh --sim
```

### 其他选项

```bash
bash run_arm.sh --build        # 启动前编译
bash run_arm.sh --no-xterm     # SSH 场景，不弹窗
bash run_arm.sh -h             # 查看完整帮助
```

---

## 任务模式（task_node）

启动后进入交互菜单，支持以下操作：

| Case | 命令 | 说明 |
|---|---|---|
| 1 | Set Preset | 移动到预设姿态（load / look_out / reset / carry） |
| 2 | Set Controller Mode | 切换控制模式（idle / moving / loaded / gravity_comp） |
| 3 | Set Payload State | 设置负载参数（质量/质心） |
| 4 | Place to Frame | 自动放置到目标方框（需 `get_place_pos` 服务） |
| 5 | Manual Place | 手动输入坐标放置 |
| 6 | Auto Grasp | 自动抓取箱子（需 `get_pick_pos` 服务） |
| 7 | Manual Grasp | 手动输入坐标抓取 |
| 8 | Switch Suction | 吸盘开关 |
| 9 | Inverse Dynamics | 进入逆动力学控制模式 |
| 10 | Teach Pendant | 示教器跟随模式 |
| 11 | Drag Record | 拖拽轨迹录制 |
| 12 | Drag Play | 回放录制的轨迹 |
| 13 | Drag Trajectory | 执行 trajectory YAML |
| 14 | Auto Stack | 自动叠放箱子（需 `get_stack_pos` 服务） |
| 15 | Manual Stack | 手动输入坐标叠放 |

### 控制模式

| 模式 | PD 增益 | 用途 |
|---|---|---|
| `idle` | 低 | 待机，允许手动推过，防止电机高频振动 |
| `moving` | 标准 | 常规运动（瞭望、俯瞰等空载移动） |
| `loaded` | 高 | 带负载运动，提高刚性减少静差 |
| `gravity_comp` | 极低 | 纯重力补偿，实现"透明"手感，用于零力示教 |

---

## ROS2 接口

### 自定义服务 (`robot_msgs/srv/`)

| 服务 | 请求 | 响应 | 说明 |
|---|---|---|---|
| `GetPickPos` | `object_name` | `success` + `pick_pose` (camera_link) | 抓取目标检测 |
| `GetPlacePos` | `frame_name` | `success` + `place_pose` (dog_camera_link) | 放置目标检测 |
| `SetControllerMode` | `mode` | `success` + `message` | 切换控制模式 |
| `SetSuction` | `activate` | `success` | 吸盘开关 |
| `SetPayloadState` | — | — | 设置负载参数 |
| `GetPayloadEstimate` | — | — | 获取负载估计值 |
| `StringCommand` | — | — | 通用字符串命令 |

### 核心话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/arm2/_lowState/joint` | `RobotState` | 驱动 → 上层 | 关节状态（q, dq, tau, valid） |
| `/arm2/_lowCmd/command` | `RobotState` | 上层 → 驱动 | 关节命令（MIT 控制） |
| `/joint_target_state` | `JointState` | control_node → 驱动 | 目标关节角 |
| `/robot_driver/ready` | `Bool` (transient_local) | 驱动 → 上层 | 驱动就绪信号 |
| `/arm/cmd` | `String` | 外部 → task_node | 远程命令（`"grasp"` / `"place"`） |
| `/arm/status` | `String` | task_node → 外部 | 状态广播 |

### TF 树

```
world（= arm_base_link）
├── dog_camera_link          ← 静态（dog_camera_extrinsics）
│   用于：get_place_pos / get_stack_pos
│
└── Link_1 ... Link_4        ← 动态（control_node FK 实时广播）
    └── camera_link          ← 静态（camera_extrinsics）
        用于：get_pick_pos
```

---

## 配置参数

核心配置文件位于 `src/arm2_task/config/`：

| 文件 | 说明 |
|---|---|
| `params.yaml` | 完整参数（两套外参、PD 增益、预设角度、几何参数、任务参数等） |
| `control_params.yaml` | control_node 专用参数 |
| `task_params.yaml` | task_node 专用参数 |

### 关键参数说明

```yaml
# 相机外参（手眼相机，已标定）
camera_extrinsics:
  parent_frame: Link_4
  child_frame: camera_link
  pos: [-0.031763, -0.090418, 0.041408]
  quat: [0.04806441, 0.69986392, -0.06693250, 0.70950712]

# 狗头相机外参（已标定）
dog_camera_extrinsics:
  parent_frame: world
  child_frame: dog_camera_link
  pos: [0.101263, 0.046662, -0.032759]
  quat: [-0.57775393, 0.64149970, -0.36065040, 0.35300115]

# 预设姿态（角度制）
presets:
  load:     [0.0, 90.0, -80.0, -90.0, 0.0]    # 俯瞰
  look_out: [-90.0, 90.0, -145.0, 0.0, 0.0]    # 瞭望
  reset:    [0.0, 175.0, -170.0, 10.0, 0.0]    # 收回
  carry:    [0.0, 175.0, -140.0, 10.0, 0.0]    # 持箱

# 抓取参数
task_step6:
  grasp_pitch: -1.57          # 抓取俯仰角 (rad)
  tool_pitch_offset: 0.25     # 吸盘前倾补偿 (rad)
  pre_grasp_offset: 0.10      # 预抓取悬停高度 (m)

# 关节限位（与驱动内部限位一致）
angle_window_lower: [-4.188, 0.0, -3.0, -2.0, -3.141]
angle_window_upper: [4.188, 3.5, 0.15, 1.57, 3.141]
```

---

## 远程控制接口

机械臂支持通过 ROS2 topic 进行外部控制（如机器狗集成）：

```bash
# 发送抓取命令
ros2 topic pub --once /arm/cmd std_msgs/msg/String "{data: 'grasp'}"

# 发送放置命令
ros2 topic pub --once /arm/cmd std_msgs/msg/String "{data: 'place'}"

# 监听状态
ros2 topic echo /arm/status
```

状态流转：`reset` → 收到 `grasp` → `grasped` → `stowed` → 收到 `place` → `placed` → `reset`

详细接口说明见 `tools/remote_control_test/ARM_INTERFACE.md`。

---

## 子包说明

### arm2_task

主控包，包含：
- **task_node** — 任务编排，交互菜单，抓取/放置/叠放序列，视觉对齐
- **control_node** — 逆动力学控制器，FK/IK，TF 广播，模式切换，摩擦力补偿
- **trajectory_planner** — 关节空间轨迹规划
- **teach_drag_record** — 拖拽示教轨迹录制
- **teach_pendant** — 示教器跟随

### dm_motor_sdk_ros

达妙电机 ROS2 驱动，负责：
- USB2CANFD_Dual 设备管理
- CAN/CANFD 帧收发
- 关节状态发布（/arm2/_lowState/joint）
- 命令转发达妙 MIT 控制协议
- 2 轴启动选圈、硬限位、跳变检测

详见 `src/dm_motor_sdk_ros/README.md`。

### suction_serial_bridge

ESP32-C3 吸盘串口控制桥接节点，提供 `set_suction` 服务。

### ftservo_hls3625_teach

FEETECH HLS3625 舵机示教器包，支持力矩开关、姿态录制/回放、零点标定。

详见 `src/ftservo_hls3625_teach/README.md`。

---

## 已知经验

### 段错误：GetPickPos.srv 不一致

两个 workspace（arm 和 neweyes）的 `robot_msgs/srv/GetPickPos.srv` 必须保持同步。
字段不一致会导致 ROS2 序列化布局错位，症状是运行时**段错误**而非编译错误，极难排查。

**修改任意一边的 `.srv` 后，必须同步另一边并重新编译。**

### 2 轴启动选圈

2 轴掉电重上电后，底层硬件位置可能与 ROS 期望差一圈。驱动在启动时执行一次性选圈（`bias = 0` 或 `+2π`），上电周期内固定。

### 坐标变换正确性

`task_node.cpp` 的 TF2 变换写法正确：直接读取 `frame_id`，让 TF2 做 `lookupTransform` → `doTransform`。只要 frame_id 填对了、TF 链存在，结果就是正确的。

最大风险点在**相机外参标定**——如果 `pos/quat` 是目测值而非实测标定，world 坐标会有系统性误差。

---

## TODO

- [ ] 修改 case 1 的 PD 参数，提高稳定性
- [ ] 让机械臂绕开固定位置
- [ ] 修改偏放逻辑
- [ ] 完善气泵控制
- [ ] 实现 `get_place_pos` 感知节点
- [ ] 实现 `get_stack_pos` 感知节点（复用 get_place_pos 改 3 行）