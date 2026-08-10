# feet-arm2 — 5-DOF 机械臂控制栈

> 基于 ROS2 Humble 的 5 轴机械臂项目，安装在四足机器狗上，用于箱子的自动抓取、放置和叠放。
> 包含电机驱动、逆动力学控制、任务编排、视觉感知对接、吸盘控制、示教器和导航集成。

---

## 目录

- [硬件概览](#硬件概览)
- [软件架构](#软件架构)
- [项目结构](#项目结构)
- [环境要求](#环境要求)
- [编译](#编译)
- [启动](#启动)
- [控制模式](#控制模式)
- [任务面板（交互菜单）](#任务面板交互菜单)
- [三阶段抓取/放置管线](#三阶段抓取放置管线)
- [导航集成](#导航集成)
- [示教功能](#示教功能)
- [ROS2 接口](#ros2-接口)
- [配置文件](#配置文件)
- [启动文件说明](#启动文件说明)
- [调试工具](#调试工具)
- [已知问题与经验](#已知问题与经验)

---

## 硬件概览

| 组件 | 型号 | 说明 |
|---|---|---|
| 机械臂 | 5-DOF 串联臂 | Yaw → Pitch1 → Pitch2 → Pitch3 → Roll |
| 电机 ×5 | 达妙（Damiao） | CAN/CANFD 通信，USB2CANFD_Dual 适配器 |
| 末端执行器 | 吸盘 | ESP32-C3 串口控制（`/dev/esp32_suction_c3`） |
| 手眼相机 | — | 安装在 Link_4，用于抓取目标检测 |
| 狗头相机 | — | 固定安装，用于放置/叠放目标检测 |
| 示教器 | FEETECH HLS3625 ×5 | 可选，用于拖拽示教和姿态录制/回放 |

---

## 软件架构

### 分层架构

```
┌──────────────────────────────────────────────────┐
│  task_node (arm2_task)                            │
│  · 交互菜单（15 种操作）                           │
│  · 导航集成（/arm/mission_event）                  │
│  · 三阶段抓取/放置管线（视觉对齐）                   │
│  · 吸盘控制、负载估计                              │
└────────┬──────────────────────────┬──────────────┘
         │ MoveJoint action         │ 感知服务调用
         ▼                          ▼
┌────────────────────┐    ┌──────────────────────┐
│  control_node      │    │  感知服务（neweyes）   │
│  · 逆动力学 + 摩擦  │    │  get_pick_pos        │
│  · 轨迹插值（五次）  │    │  get_place_pos       │
│  · PD 控制          │    │  get_stack_pos       │
│  · TF 广播（FK）    │    └──────────────────────┘
│  · 4 种控制模式     │
└────────┬───────────┘
         │ /arm2/_lowCmd/command
         ▼
┌──────────────────────────────────────────────────┐
│  dm_motor_sdk_ros                                 │
│  · CAN 帧收发 · 2 轴启动选圈                       │
│  · 关节状态发布 · 限位保护 · 跳变检测               │
└────────┬─────────────────────────────────────────┘
         │ USB2CANFD_Dual
         ▼
┌──────────────────────────────────────────────────┐
│  达妙电机 ×5（物理层）                             │
└──────────────────────────────────────────────────┘
```

### 两套控制栈

项目包含两套可选的底层控制方案，通过不同的 launch 文件切换：

| | Stack A（生产） | Stack B（实验/示教） |
|---|---|---|
| **启动文件** | `mini_launch.py` / `run_arm.sh` | `planner_inverse_task_launch.py` |
| **控制器** | `control_node` | `trajectory_planner_node` + `inverse_dynamics_node` |
| **控制模式** | idle, gravity_comp, moving, loaded（4 种） | 上述 4 种 + teach_pendant, teach_drag（6 种） |
| **特点** | 一体化，`run_arm.sh` 默认使用 | 支持拖拽示教、示教器跟随 |

### 辅助节点

| 节点 | 包 | 作用 |
|---|---|---|
| `suction_service_node` | `suction_serial_bridge` | 串口控制吸盘开关 |
| `bus_state_publisher` | `ftservo_hls3625_teach` | 示教舵机状态发布 |
| `debug_node` | `arm2_task` | 遥测监控 + 电机温度报警 |
| `gravity_comp_test_node` | `arm2_task` | 重力补偿测试（手持验证） |
| `inverse_dynamics_target_test_node` | `arm2_task` | 正弦波目标发生器（安全测试） |

---

## 项目结构

```
feet-arm2/
├── run_arm.sh                        # 一键启动脚本
├── README.md
├── LICENSE                           # MIT
├── .ros_domain_id.env                # ROS_DOMAIN_ID=91
├── .gitignore
├── scripts/
│   ├── install_esp32_suction_udev.sh # 吸盘串口 udev 规则
│   ├── run_suction_receiver.sh       # 吸盘接收端（分主机部署）
│   ├── tail_task_log.sh              # 查看 task_node 日志
│   └── debug/                        # 调试启动脚本（示教、吸盘、重力补偿）
├── tools/
│   └── remote_control_test/          # 远程控制测试工具（设计文档）
└── src/
    ├── robot_msgs/                   # 自定义消息/服务/动作接口
    ├── arm2_task/                    # 主控包（10 个可执行文件）
    │   ├── config/                   # 参数文件（3 套）
    │   ├── launch/                   # 启动文件（6 个）
    │   ├── include/arm2_task/        # 头文件（运动学/动力学/状态机）
    │   ├── src/                      # 源码
    │   └── urdf/                     # 机器人 URDF 模型
    ├── dm_motor_sdk_ros/             # 达妙电机 ROS2 驱动
    ├── suction_serial_bridge/        # 吸盘串口桥接
    ├── ftservo_hls3625_teach/        # 示教器包
    └── DM_DeviceSDK/                 # 达妙官方 SDK（第三方，需自行下载）
```

---

## 环境要求

| 依赖 | 说明 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble Hawksbill |
| 编译器 | GCC 支持 C++17 |
| Pinocchio | 刚体动力学库（`apt install ros-humble-pinocchio`） |
| navigation | 外部导航包（`find_package(navigation REQUIRED)`） |
| 达妙 SDK | 放入 `src/dm_motor_sdk_ros/third_party/` 和 `src/DM_DeviceSDK/` |

---

## 编译

```bash
cd feet-arm2
source /opt/ros/humble/setup.bash

# 可选：source 导航工作区
source ~/task/nav_ws/install/setup.bash

# 完整编译
colcon build --packages-select robot_msgs suction_serial_bridge dm_motor_sdk_ros arm2_task \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 启动

### 一键启动（推荐）

```bash
bash run_arm.sh              # 真机模式
bash run_arm.sh --sim        # 仿真模式（需先启动 MuJoCo 仿真器）
bash run_arm.sh --build      # 编译后启动
bash run_arm.sh --no-xterm   # SSH 场景（task_node 输出到当前终端）
bash run_arm.sh -h           # 查看所有选项
```

启动流程：安装 udev 规则 → 启动驱动 → 等待 `/robot_driver/ready` → 启动 `control_node` → 启动吸盘节点 → 启动 `task_node`

### 手动启动特定组件

```bash
# 仅逆动力学控制器
ros2 launch arm2_task inverse_dynamics_launch.py

# 示教器跟随
ros2 launch arm2_task teach_pendant_follow_launch.py

# 拖拽示教录制（仅启动节点，实际录制功能未实现）
ros2 launch arm2_task teach_drag_record_launch.py

# 正弦波测试目标
ros2 launch arm2_task inverse_dynamics_target_test_launch.py
```

---

## 控制模式

通过 `set_controller_mode` 服务切换，不同模式对应不同的 PD 增益：

| 模式 | 用途 | kp 范围 | kd 范围 |
|---|---|---|---|
| `idle` | 待机，允许手动推过，防止电机高频振动 | 0~2 | 0.1~0.5 |
| `gravity_comp` | 纯重力补偿，实现"透明"手感，零力示教 | 0.1~8.5 | 0.02~2.5 |
| `moving` | 标准运动（空载移动、瞭望、俯瞰） | 0.5~15 | 0.1~4 |
| `loaded` | 带负载运动（抓取箱子后），高刚性 | 0.8~20 | 0.2~8.5 |
| `teach_pendant` | 示教器跟随（仅 Stack B） | 0.5~15 | 0.1~4 |
| `teach_drag` | 零力拖拽示教（仅 Stack B），kp≈0 | 0~0.25 | 0.05~0.8 |

---

## 任务面板（交互菜单）

启动后进入交互菜单，共 16 个选项。**`task.manual_mode: true` 启用菜单，`false` 启用导航触发模式。**

```
====== Task Control Panel ======
1:  Reset              — 吸盘 OFF → 复位姿态 → idle
2:  Joint preset A     — 调试用：q = [0, 160, -130, 40, 0]°
3:  Joint preset B     — 调试用：q = [180, 90, -90, -90, 0]°
4:  Auto place         — 狗头相机感知 → 放置 → 吸盘 OFF → 后退
5:  Manual place       — 手动输入 x y z yaw → 放置
6:  Auto grasp         — 手眼相机感知 → 瞭望 → 抓取 → 吸盘 ON
7:  Manual grasp       — 手动输入 world x y z → 抓取
8:  Release            — 吸盘 OFF → moving 模式
9:  Carry reset        — 保持吸盘 ON → 持箱姿态 → loaded
10: Move to load       — 移到俯瞰姿态（相机朝下）
11: Estimate payload   — 负载质量估计
12: 3-Phase Grasp      — 三阶段精抓：扫描 → XY 对齐 → 下降抓取
13: 3-Phase Place      — 三阶段精放：扫描 → XY 对齐 → 下降放置
14: Auto Stack         — 狗头相机感知 → 叠放
15: Manual Stack       — 手动输入目标箱子上表面 x y z yaw → 叠放
0:  Exit
```

### 抓取流程详解（Case 6）

1. 切换到 `moving` 模式
2. 执行 `look_out`：joint_0 朝向目标方向，其余关节用 look_out 预设
3. 调用 `get_pick_pos` 感知服务（或使用 mock 坐标）
4. 3 次采样取中值（x/y/z 中值 + roll 中值）
5. 计算抓取位姿：末端 pitch = grasp_pitch + tool_pitch_offset，加工具偏移
6. 两段轨迹：pre-grasp（悬停）→ grasp（接触）
7. 吸盘 ON → 500ms 稳定 → 切换到 `loaded` 模式

### 放置/叠放流程详解（Case 4/5/14/15）

1. 调用感知服务获取目标位姿（或手动输入/mock）
2. TF 变换到 world 坐标系
3. 计算末端姿态（利用箱子 90° 对称性选择最优 roll）
4. 两段轨迹：hover（悬停）→ contact（接触）
5. 吸盘 OFF → 垂直后退 → 切换到 `moving` 模式

---

## 三阶段抓取/放置管线

比普通抓取多一个 **XY 闭环视觉对齐** 阶段，精度更高：

```
Phase 1 — 扫描（Scan）
  look_out → 感知/手动输入 → 获得粗定位目标

Phase 2 — 对齐（Align）
  移到 load 俯瞰姿态 → 固定 Z 高度
  → 每轮：感知当前物体 XY → 与末端 XY 比较
  → 调整 joint_0 → 重复直到误差 < 0.005m（最多 5 轮）

Phase 3 — 下降执行（Descend）
  joint_4 → -90° → 确认（真实/手动/中止）
  → 执行抓取或放置
```

参数：`visual_align.align_threshold: 0.005`（收敛阈值），`visual_align.max_iters: 5`（最大迭代）

---

## 导航集成

机械臂可与机器狗导航系统联动，通过服务触发抓取：

```
nav 到达任务点 → 调用 /arm/mission_event (Trigger) → 机械臂执行抓取
→ 发布 "grabbed" → 持箱收起 → 发布 "completed" → nav 继续移动
```

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/arm/mission_event` | `std_srvs/Trigger` | nav → 机械臂 | 触发抓取序列 |
| `/navigation/arm_event` | `navigation/StringCommand` | 机械臂 → nav | 回报 "grabbed" / "completed" |

**注意**：放置序列的 nav 触发尚未接入（代码中 HOLDING 分支被注释），机械臂抓取后不会自动放置。

切换到导航模式：`task.manual_mode: false`

---

## 示教功能

### 示教器（FEETECH HLS3625）

物理示教舵机，通过串口读取关节角度，映射到机械臂：

```bash
# 单机部署
bash scripts/debug/run_teach_pendant_real.sh

# 分机部署（示教器和机械臂在不同主机）
bash scripts/debug/run_teach_pendant_robot_side.sh   # 机械臂侧
bash scripts/debug/run_teach_pendant_pendant_side.sh  # 示教器侧
```

核心功能：
- 力矩开关：`set_torque` 释放舵机，手动摆姿势
- 姿态录制：`capture_pose` 记录当前舵机原始脉冲值
- 姿态回放：`play_pose` 回放录制的姿态
- 零点标定：`capture_zero_offsets` 自动生成标定配置文件

### 拖拽示教（Drag Teach）

切换到 `teach_drag` 模式后，逆动力学完全补偿重力和摩擦力，kp≈0，可以手拖机械臂运动。**轨迹录制节点尚未实现**（`teach_drag_record_node` 无源码），但 `teach_drag` 控制模式可用。

---

## ROS2 接口

### 自定义服务（robot_msgs/srv/）

| 服务 | 请求 | 响应 | 提供方 |
|---|---|---|---|
| `GetPickPos` | `object_name` | `success` + `pick_pose` (camera_link) | neweyes 感知节点 |
| `GetPlacePos` | `frame_name` | `success` + `place_pose` (dog_camera_link) | neweyes 感知节点 |
| `SetControllerMode` | `mode` | `success` + `message` | control_node / inverse_dynamics_node |
| `SetSuction` | `activate` | `success` | suction_serial_bridge |
| `SetPayloadState` | `has_load, mass, com` | `success` + `message` | inverse_dynamics_node |
| `GetPayloadEstimate` | — | `mass` + `success` + `message` | control_node |

### 核心话题

| 话题 | 类型 | QoS | 方向 |
|---|---|---|---|
| `/arm2/_lowState/joint` | `RobotState` | best_effort, KeepLast(1) | 驱动 → 上层 |
| `/arm2/_lowCmd/command` | `RobotCommand` | best_effort, KeepLast(1) | 上层 → 驱动 |
| `/joint_target_state` | `RobotState` | — | planner → inverse_dynamics（Stack B） |
| `/robot_driver/ready` | `Bool` | reliable, transient_local | 驱动 → 上层 |
| `/arm2/_lowState/temperature` | `Float32MultiArray` | — | 驱动 → debug_node |

### Action

| Action | 说明 |
|---|---|
| `move_joint` | 关节空间轨迹执行（支持多段 + 混合半径） |

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

## 配置文件

| 文件 | 用途 |
|---|---|
| `src/arm2_task/config/params.yaml` | **唯一参数文件**：外参、摩擦、6 组增益、负载、预设、几何、轨迹、任务、逆动力学 |
| `src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml` | 驱动参数：CAN 参数、电机 ID、零位、限位、跳变阈值 |

### 关键参数说明

```yaml
# 预设姿态（角度制）
presets:
  load:     [0, 90, -80, -90, 0]     # 俯瞰（相机朝下）
  look_out: [-90, 90, -145, 0, 0]    # 瞭望（相机斜下）
  reset:    [0, 175, -170, 10, 0]    # 收回/待机
  carry:    [0, 175, -140, 10, 0]    # 持箱姿态

# 关节限位（与驱动内部一致）
angle_window_lower: [-4.188, 0.0, -3.0, -2.0, -3.141]
angle_window_upper: [4.188, 3.5, 0.15, 1.57, 3.141]

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

# 抓取参数
task_step6:
  grasp_pitch: -1.57          # 抓取俯仰角 (rad)
  tool_pitch_offset: 0.25     # 吸盘前倾补偿 (rad)
  pre_grasp_offset: 0.10      # 预抓取悬停高度 (m)
  use_mock_target: false      # true 时用固定坐标调试（不需要相机）

# 放置参数
task_place_frame:
  hover_height: 0.10
  contact_offset: 0.45
  use_mock_target: false      # true 时用固定坐标调试

# 叠放参数
task_stack:
  hover_height: 0.05
  contact_offset: 0.25        # 箱子高度
  use_mock_target: false

# 摩擦力模型：tau_f = fc * tanh(alpha * dq) + fv * dq * GearRatio²
dynamics:
  friction:
    GearRatio: [10, 30, 30, 7, 7]
    alpha: 80.0
    fc: [0.25, 0.7, 0.25, 0.07, 0.05]
    fv: [0.00026634, 0.00013977, 0.00028228, 0.00012035, 0.00013322]
```

---

## 启动文件说明

| 文件 | 启动的节点 | 用途 |
|---|---|---|
| `mini_launch.py` | control_node + task_node | Stack A，`run_arm.sh` 实际使用 |
| `planner_inverse_task_launch.py` | trajectory_planner + inverse_dynamics + task_node | Stack B，支持示教和拖拽 |
| `inverse_dynamics_launch.py` | inverse_dynamics_node | 仅逆动力学控制器 |
| `teach_pendant_follow_launch.py` | teach_pendant_follow_node | 示教器跟随 |
| `teach_drag_record_launch.py` | teach_drag_record_node | ⚠️ 节点源码缺失 |
| `inverse_dynamics_target_test_launch.py` | inverse_dynamics_target_test_node | 正弦波安全测试 |

---

## 调试工具

| 工具 | 说明 |
|---|---|
| `debug_node` | 遥测监控 + 电机温度报警（70°C 警告 / 80°C 错误） |
| `gravity_comp_test_node` | 仅重力补偿，手持验证重力模型和摩擦力符号 |
| `inverse_dynamics_target_test_node` | 生成限幅正弦波目标，安全测试逆动力学 |
| `dm_motor_sdk_stress_test` | 原始 CAN 收发压力测试 |
| `suction_keyboard_client` | 手动吸盘开关（y/n/q） |
| `scripts/tail_task_log.sh` | 实时查看 task_node 日志，支持 grep 过滤 |
| `tools/remote_control_test/arm_controller.py` | 远程控制测试脚本（依赖的 `/arm/cmd` 接口未实现） |

### Mock 调试

所有视觉任务都支持 mock 模式，在 `params.yaml` 中设置 `use_mock_target: true` 即可用固定坐标替代相机感知，方便无相机调试：

```yaml
task_step6.use_mock_target: true       # Case 6 用 mock 坐标
task_place_frame.use_mock_target: true # Case 4 用 mock 坐标
task_stack.use_mock_target: true       # Case 14 用 mock 坐标
```

---

## 已知问题与经验

### 1. 段错误：GetPickPos.srv 不一致

两个 workspace（arm 和 neweyes）的 `robot_msgs/srv/GetPickPos.srv` 必须字段完全一致。不一致会导致 ROS2 序列化布局错位，症状是运行时**段错误**而非编译错误，极难排查。

> **修改任意一边的 `.srv` 后，必须同步另一边并重新编译。**

### 2. 2 轴启动选圈

2 轴掉电重上电后，底层硬件位置可能与 ROS 期望差一圈。驱动在启动时自动选圈（`bias = 0` 或 `+2π`），但启动姿态必须落在 `[0, 3.5]` 或 `[-2π, 3.5-2π]` 范围内，否则选圈失败（回退 bias=0）。

### 3. 相机外参是最大精度风险

`params.yaml` 中的外参 `pos/quat` 直接影响 world 坐标计算精度。如果外参未经实测标定，所有视觉引导操作都会有系统性误差。

### 4. 未实现的功能

- `teach_drag_record_node` — 启动文件存在但源码缺失
- `MoveToPose.action` — 接口已定义但无节点使用
- 导航放置路径 — `run_remote_control()` 中 HOLDING → 放置分支被注释

---