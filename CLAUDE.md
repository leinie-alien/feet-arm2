# CLAUDE.md — ~/data/robotics/arm 项目速查

> 写给下次进入这个项目的 Claude：读完这个文件你就不需要再完整扫一遍代码了。

---

## 1. 项目定位

5-DOF 机械臂 ROS2 (Humble) 控制系统，使用 **Damiao CAN 电机** (`dm_motor_sdk_ros`)。

**当前状态（2026-04）：已在实机验证可用**
- 机械臂控制链路（move_joint action / IK / TF）✓
- 相机订阅与识别（get_pick_pos 服务）✓
- 吸盘控制（set_suction 服务）✓
- 完整 3-Phase 抓取/放置流程（含 Phase-2 世界坐标闭环对齐）✓

**代码来源说明**：
- 底层接口（send_move_goal / wait_for_action_completion / TF / mode_switch / suction）来自本项目，均已实机验证
- 任务流程状态机（do_full_grasp / do_full_place / 3-Phase pipeline）从 `aarm` 项目迁移，使用本项目接口重写

---

## 2. 包结构

```
src/
├── arm2_task/          # 主控包（控制 + 任务）
├── robot_msgs/         # 自定义消息/服务/Action 定义
├── dm_motor_sdk_ros/   # Damiao CAN 电机驱动（底层）
└── ftservo_hls3625_teach/ # 舵机示教笔（可选，独立）
tools/
└── remote_control_test/
    └── arm_controller.py   # 远程控制测试脚本（独立，非 ROS2 包）
```

---

## 3. arm2_task 内部结构

### 节点

| 可执行文件 | 类名 | 职责 |
|---|---|---|
| `control_node` | `ControlNode` | 轨迹规划、PD+动力学控制、TF 发布、Action 服务端 |
| `task_node` | `TaskNode` | 菜单式任务调度（抓取/放置流程） |
| `trajectory_planner_node` | — | 独立轨迹规划节点（备用，通常不单独跑） |
| `inverse_dynamics_node` | — | 逆动力学测试 |
| `gravity_comp_test_node` | — | 重力补偿测试 |

### task_node.cpp 代码分区

| Section | 内容 | 修改频率 |
|---|---|---|
| Section 1 | Infrastructure（send_move_goal / TF / suction / mode_switch / get_object_yaw 等） | 不要动，已验证 |
| Section 2 | Task Building Blocks（do_reset / do_look_out / do_grasp_move 等） | 调参数时不需改代码 |
| Section 3 | 3-Phase Pipeline（phase1/2/3 + do_3phase_grasp/place） | 偶尔调整流程 |
| Section 4 | run_task_sequence（菜单 loop） | 加新 case 时改这里 |
| Section 5 | Remote Control（run_remote_control / do_grasp_sequence / do_place_sequence） | 远程控制流程调整时改这里 |

### 关键头文件

| 文件 | 内容 |
|---|---|
| `include/arm2_task/kinematics_engine.hpp` | `KinematicsEngine`: FK、IK、jacobian 速度映射、TF 坐标变换 |
| `include/arm2_task/dynamics_manager.hpp` | `DynamicsManager`: Pinocchio 重力补偿 + 摩擦力补偿 + 负载估计 |
| `include/arm2_task/common_units.hpp` | `RobotGeometry`, `JointState`, `CartesianPose`, `TaskState` 等基础结构 |

### IK 函数签名（`KinematicsEngine`，task_node 可用的）

```cpp
// 主用：目标位置 + 末端俯仰角 → 关节角（roll 默认为 0）
bool solveIK(const Eigen::Vector3d& target, double pitch, Eigen::VectorXd& q_out);

// 主用（含 roll）：目标位置 + 俯仰角 + 工具滚转角 → 关节角
// 内部调用上面的版本，再设 q_out[4] = roll
bool solveIK(const Eigen::Vector3d& target, double pitch, double roll, Eigen::VectorXd& q_out);

// 备用：带 r_offset / z_offset（偏移量 IK）
bool solveIK(const Eigen::Vector3d& target, Eigen::VectorXd& q_out, double r_offset, double z_offset);

// FK：关节角 → 末端 SE3 位姿（含平移和旋转）
pinocchio::SE3 forwardKinematics(const Eigen::VectorXd& q);
```

### tool_roll 计算（task_node 内部，Section 1/2）

```cpp
// get_object_yaw(target_world)：恒返回 0
//   object_yaw = atan2(target.y, target.x)
//   base_yaw   = object_yaw（IK 会把 joint_0 设成同样的值）
//   → 差值恒为 0，吸盘不额外旋转
//   用途：do_place_move（普通放置）和无朝向抓取

// get_box_edge_roll(target_world)：从视觉 orientation 提取箱子主边偏转角
//   视觉返回 R = [axis_u | axis_v | n_hat]（camera_link 系），经 TF 变换到 world 系
//   edge_yaw = atan2(R[1,0], R[0,0])      // 主边在 world XY 平面的方位角
//   base_yaw = atan2(target.y, target.x)   // ≈ joint_0
//   roll     = normalize(edge_yaw - base_yaw)，再归一化到 [-π/2, π/2]（矩形180°对称）
//   → joint_4 转到对齐主边的角度
//   用途：case 6 do_full_grasp_aligned

// get_frame_yaw(frame_world)：与 get_box_edge_roll 完全对称，用于放置框
//   从 orientation 提取 Z 轴 yaw，减去 joint0_ik，归一化到 [-π/2, π/2]
//   用途：case 4/5 do_place_move_with_orientation
```

`do_grasp_move(target, roll)` 和 `do_place_move_with_orientation` 均由调用方传入 roll，外部负责计算。

---

## 4. 话题 / 服务 / Action 命名

### 底层驱动 (dm_motor_sdk_ros)

| 方向 | 话题 | 类型 |
|---|---|---|
| 状态读取 | `/arm2/_lowState/joint` | `robot_msgs/msg/RobotState` |
| 指令下发 | `/arm2/_lowCmd/command` | `robot_msgs/msg/RobotCommand` |
| 驱动就绪 | `/robot_driver/ready` | `std_msgs/msg/Bool` (transient_local) |

位置模型约定：

- 编码器本体是单圈绝对值，但底层电机驱动器在每次上电后会自行记圈，所以 `/arm2/_lowState/joint` 中的 `q` 按连续位置理解。
- `joint_zero_offsets` 只做硬件零点到 ROS 零点的小量微调，不承担判圈职责。
- 1轴在驱动内部直接按 `±240°` 软件限位，主要用于防止过转扯线。
- 2轴机械范围按 `[0, 3.5] rad` 处理；如果启动首帧反馈落在 `[-2pi, 3.5-2pi]`，底层驱动会一次性补 `+2pi` 选圈，之后整次上电周期保持固定偏置，不做运行时动态判圈。
- 3/4/5轴只做零点微调和硬限位。

### control_node 提供

| 类型 | 名称 | 说明 |
|---|---|---|
| Action Server | `move_joint` | 轨迹跟踪（5阶多项式 + blending） |
| Service | `set_controller_mode` | 切换 PD 增益组：`idle/moving/loaded/gravity_comp/teach_pendant/teach_drag` |
| Service | `get_payload_estimate` | 触发负载质量估计 |
| TF (动态) | `world → Link_4` | 由 FK 实时计算发布 |
| TF (静态) | `Link_4 → camera_link` | 由 params.yaml `camera_extrinsics` 配置 |

### task_node 依赖的服务（可选）

| 服务 | 说明 |
|---|---|
| `get_pick_pos` | 手臂相机感知节点返回抓取目标位姿（`GetPickPos.srv`），frame_id = `camera_link` |
| `get_place_pos` | 狗头相机感知节点返回放置框位姿（`GetPlacePos.srv`），frame_id = `dog_camera_link` |
| `set_suction` | 吸盘开关（`SetSuction.srv`） |

### 远程控制话题（task.remote_mode: true 时启用）

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/arm/cmd` | 接收 | `std_msgs/String` | `"grasp"` / `"place"` |
| `/arm/status` | 发布 | `std_msgs/String` | `"grasped"` / `"stowed"` / `"placed"` / `"reset"` / `"error:xxx"` |

---

## 5. MoveJoint Action 结构

```
# Goal
float64 max_velocity
float64 max_acceleration
float64 blend_radius
int32 num_points
float64[] joint_targets   # 5*num_points 个值，按行展开
# Result / Feedback: 标准成功/失败
```

`task_node` 调用方式：`send_move_goal(vector<VectorXd>)` → `wait_for_action_completion()`

---

## 6. 预设位姿（params.yaml 里改角度，运行时自动转 rad）

| 名称 | 角度 [joint0..4]° | 用途 |
|---|---|---|
| `look_out` | [0, 120, -150, 0, 0] | 瞭望姿态，相机朝斜下方（视野广，Phase-1 用） |
| `load` | [0, 90, -90, -90, 0] | 俯瞰姿态，相机朝正下方（Phase-2 对齐用） |
| `reset` | [0, 175, -170, 10, 0] | 收回/待机姿态 |

**改法**：`src/arm2_task/config/params.yaml` → `presets:` 节点下直接改角度数字。

---

## 7. 任务菜单（task_node case 列表）

| Case | 名称 | 流程 |
|---|---|---|
| **1** | Reset | 吸盘 OFF → moving → 移到 reset 预设 → idle |
| **2** | Preset A（调试） | moving → 移到硬编码关节角 [0,160,-130,40,0]° |
| **3** | Preset B（调试） | moving → 移到硬编码关节角 [180,90,-90,-90,0]° |
| **4** | Auto Place（感知坐标） | 调用狗头相机感知服务获取方框位姿 → do_place_move_with_orientation（无 look_out，无手动输入） |
| **5** | Manual Place | 手动输入 world x y z yaw → do_place_move_with_orientation |
| **6** | Auto Grasp | mock/感知 → look_out → pre-grasp → grasp（含 tool_roll 对齐）→ suction ON |
| **7** | Manual Grasp | 输入 world x y z → look_out → pre-grasp → grasp（含 tool_roll）→ suction ON |
| **8** | Release | suction OFF → moving 模式 |
| **9** | Carry（携带复位） | 保持吸盘 ON，moving → carry 预设 → loaded |
| **10** | Move to Load | moving → 移到 load 预设（俯瞰位） |
| **11** | Estimate Payload | 调用 get_payload_estimate 服务，打印估计质量 |
| **12** | 3-Phase Grasp | Phase1(粗定位) → Phase2(XY闭环对齐) → Phase3(joint4 -90°→抓取) |
| **13** | 3-Phase Place | Phase1(粗定位) → Phase2(XY闭环对齐) → Phase3(joint4 -90°→放置) |
| **0** | Exit | 退出 |

---

## 8. 3-Phase Pipeline 详解（case 12/13）

```
Phase 1（粗定位）
  → 移到 look_out（joint_0 朝正前方）
  → 选择输入模式：
      (r)eal   → 调用 get_pick_pos 服务，TF 变换到 world 坐标
      (m)anual → 用户直接输入 world 坐标 x y z
      (a)bort  → 中止

Phase 2（世界坐标 XY 闭环对齐）
  → 移到 load 预设（joint_0 朝向粗定位目标），记录当前末端 Z（全程固定）
  → 循环最多 align_max_iters 次：
      1. 调用 get_pick_pos 服务，获取物体 world (tx, ty)
      2. FK(q_current) 获取当前末端 (ex, ey)
      3. error = sqrt((tx-ex)² + (ty-ey)²)
      4. error < align_threshold → 收敛退出
      5. 否则：只更新 joint_0 = atan2(ty, tx)，其余关节保持不动（避免完整 IK 引起姿态跳变）
  → 未收敛时用最后估计值继续（不中止）

Phase 3 Grasp（抓取下降）
  → joint_4 → -90°（吸盘朝下，与相机无关）
  → 询问确认 (r/m/a)
  → do_grasp_move：pre-grasp IK + grasp IK，双段轨迹，roll 由 get_object_yaw() 计算
  → suction ON → 等 400ms → loaded 模式

Phase 3 Place（放置下降，对称）
  → joint_4 → -90°
  → 询问确认 (r/m/a)
  → do_place_move：pre-place IK + place IK + 垂直后退，roll 由 get_object_yaw() 计算
  → suction OFF → moving 模式
```

---

## 9. 参数速查表（全部在 params.yaml 里改）

文件路径：`src/arm2_task/config/params.yaml`

### 运动参数

| 参数键 | 当前值 | 说明 | 改了影响 |
|---|---|---|---|
| `trajectory_planner.max_velocity` | 0.8 | 最大关节速度 (rad/s) | 所有运动快慢 |
| `trajectory_planner.max_acceleration` | 1.2 | 最大关节加速度 (rad/s²) | 所有运动加速度 |
| `trajectory_planner.dist_threshold` | 0.05 | 轨迹 blend 半径 | 多段轨迹平滑度 |

### 预设姿态（角度制）

| 参数键 | 当前值 | 改了影响 |
|---|---|---|
| `presets.look_out` | [0,120,-150,0,0] | Phase-1 瞭望姿态 / case 6 感知姿态 |
| `presets.load` | [0,90,-90,-90,0] | Phase-2 俯视对齐姿态 / case 10 |
| `presets.reset` | [0,175,-170,10,0] | case 1/9 复位姿态 |

### 抓取/放置参数

| 参数键 | 当前值 | 说明 |
|---|---|---|
| `task_step6.grasp_pitch` | -1.57 | 末端俯仰角 (rad)，-1.57 = 竖直朝下 |
| `task_step6.object_height` | 0.005 | 物体高度偏移 (m)，直接加到目标 Z 上（末端落点 = target.z + object_height + tool_offset_z） |
| `task_step6.pre_grasp_offset` | 0.10 | 预抓取高度偏移 (m)，在末端落点上方悬停的高度 |
| `task_step6.tool_offset_x/y` | 0.0 | 工具末端相对物体中心的 XY 偏移 (m) |
| `task_step6.tool_offset_z` | -0.17 | 工具末端 Z 偏移 (m)，**补偿 MuJoCo base_link Z=0.38m 偏置** |
| `task_step6.pick_object_name` | "box" | 传给 get_pick_pos 服务的物体名 |
| `task_step6.use_mock_target` | false | true = case 6 用 mock 坐标（调试用） |
| `task_step6.mock_x/y/z` | 0.35/0/0.12 | mock 目标坐标 (m) |
| `task_place.pre_place_offset` | 0.12 | 预放置悬停高度偏移 (m) |
| `task_place.retreat_offset` | 0.15 | 放置后垂直后退距离 (m) |

### Phase-2 对齐参数

| 参数键 | 当前值 | 说明 |
|---|---|---|
| `visual_align.align_threshold` | 0.005 | 收敛阈值 (m)，末端 XY 与目标的距离 |
| `visual_align.max_iters` | 5 | 最大对齐迭代次数 |

### 服务依赖开关

| 参数键 | 当前值 | 说明 |
|---|---|---|
| `task.require_payload_service` | false | true = 没有 payload 服务时启动失败 |
| `task.require_suction_service` | false | true = 没有 suction 服务时启动失败 |
| `task.remote_mode` | false | true = 通过 `/arm/cmd` topic 控制；false = 菜单交互模式 |

### 相机外参

| 参数键 | 当前值 | 说明 |
|---|---|---|
| `camera_extrinsics.pos` | [0, 0, 0] | camera_link 相对 Link_4 的位置 (m) |
| `camera_extrinsics.quat` | [0, 0, 0, 1] | camera_link 相对 Link_4 的旋转（恒等，无旋转） |

> **为什么是 identity**：坐标变换在 `mujoco_runner/main.py` 的 `get_pick_pos_callback` 内已完成。MuJoCo `framepos` sensor 输出目标在 `Link_4_sensor_site` 局部坐标系下的位置（该 site 相对 Link_4 body 有 `pos=[0,-0.07,0]` + `euler=[0,1.5708,0]` 即 Ry(90°)）。Python 代码将其转换为 Link_4 body frame：
> ```
> p_link4.x = -p_site.z
> p_link4.y =  p_site.y - 0.07
> p_link4.z =  p_site.x
> ```
> 返回的 `frame_id = "Link_4"`，task_node 直接通过 `world→Link_4` 动态 TF 转换即可，无需额外旋转。
>
> 如用错误的 pos=[0,-0.07,0] / quat=[0,0.707,0,0.707]，TF camera_link X 轴会指向 world -Y（而非 world -Z），导致感知坐标 y 值严重偏大（如 y=-0.803）。

---

## 10. Z 坐标计算说明

末端落点 Z 计算链（`do_grasp_move` / `do_place_move`）：

```
ee_target.z = target.position.z + object_height + tool_offset_z
```

- `target.position.z`：来自感知服务，经 TF 变换后的 world 系 Z
- `object_height`（0.005）：从地面算起的物体顶面高度，让末端接触物体而非穿过地面
- `tool_offset_z`（-0.17）：**关键补偿值**。MuJoCo 中 `base_link` 位于 `z=0.38m`，而 IK 假设基座在 `z=0`，导致 FK/IK 坐标系与 world 坐标系有 0.38m 偏差。通过 `-0.17` 抵消这部分误差（结合 `object_height` 共同调节）。实机不需要此补偿时置为 `0.0`。

**调末端高度的正确顺序**：
1. 先确认感知坐标正确（看日志 `get_pick_pos -> world=(x,y,z)` 是否合理）
2. 再改 `tool_offset_z`（整体抬高/降低末端）
3. 最后改 `object_height`（微调）

---

## 11. 相机几何

```
相机安装在 Link_4 上，TF: world → Link_4 (动态FK) → camera_link (静态外参)
```

`get_pick_pos` 服务返回 `frame_id = "camera_link"` 的坐标，`call_pick_service_sync()` 内部通过 TF 转换为 world 系再返回。

**Phase 3 joint_4=-90° 的作用**：仅为让**吸盘**朝下，与相机视角无关。相机在 Phase-2 俯视对齐时不受 joint_4 影响。

---

## 12. 编译与启动

```bash
# 编译（在项目根目录）
cd ~/data/robotics/arm
colcon build --packages-select arm2_task

# 一键启动（推荐）— 真机模式，task_node 弹 xterm 窗口
bash run_arm.sh

# 仿真模式
bash run_arm.sh --sim

# SSH 无 xterm
bash run_arm.sh --sim --no-xterm

# 启动前自动编译
bash run_arm.sh --build
```

**`run_arm.sh` 关键设计**：
- `/robot_driver/ready` 用 `transient_local` QoS，**会缓存上次会话的旧消息**。
  脚本在收到 ready topic 后，会立即二次确认驱动进程还活着，防止误把旧缓存当成就绪信号。

---

## 13. 常见调试场景

### 对齐太慢/太快
改 `visual_align.max_iters`（次数）。

### 抓不到（末端位置偏高/偏低）
按顺序排查：
1. 看日志确认 `get_pick_pos` 返回的 world Z 是否合理
2. 改 `task_step6.tool_offset_z`（整体抬高/降低，主要补偿量）
3. 改 `task_step6.object_height`（微调，物体顶面高度）

### 预抓取碰到物体
增大 `task_step6.pre_grasp_offset`。

### 工具方向偏转（roll 对不上物体）
`get_object_yaw()` 计算的是 `atan2(obj.y, obj.x) - joint_0`。如果物体方向与预期不符，检查感知服务返回的 world XY 坐标是否正确。

### Phase-2 不收敛
1. 先检查 `get_pick_pos` 服务输出的坐标是否稳定
2. 放宽 `visual_align.align_threshold`（增大收敛阈值）
3. 或增大 `visual_align.max_iters`（允许更多次迭代）

### 快速验证流程（不接真机）
1. `task.require_suction_service: false`（跳过吸盘）
2. `task_step6.use_mock_target: true`（case 6 用固定坐标）
3. Phase-1 选 `(m)anual` 手动输入坐标

### task_node 卡在 "Waiting for first robot joint state..."
检查 `robot_msgs/msg/MotorState.msg` 是否包含 `bool valid` 字段，以及驱动发布的消息里 `valid=true`。仿真时确认 `mujoco_runner/main.py` 中 `ms.valid = True`。

---

## 14. 远程控制模式

`params.yaml` 设 `task.remote_mode: true` 后，task_node 启动时自动执行 reset，之后通过 topic 接受外部命令，不弹菜单。

### 时序

```
启动 → do_reset → 广播 "reset" → 等命令

收到 "grasp":
  → look_out + perception（get_pick_pos）
  → do_full_grasp_aligned（grasp + suction ON）
  → sleep 0.5s → 广播 "grasped"
  → moving → carry 预设 → loaded
  → 广播 "stowed"

收到 "place":
  → call_place_service_sync（get_place_pos）
  → do_place_move_with_orientation（place + suction OFF + retreat）
  → 广播 "placed"
  → do_reset
  → 广播 "reset"

执行中收到新命令 → 广播 "error:busy"
```

### 测试脚本

```bash
# source 环境
source /opt/ros/humble/setup.bash
source ~/task/arm/feet-arm2-main/install/setup.bash

# 交互模式（g=grasp / p=place / q=quit）
python3 tools/remote_control_test/arm_controller.py

# 单次命令
python3 tools/remote_control_test/arm_controller.py --once grasp
python3 tools/remote_control_test/arm_controller.py --once place
```

### roll 对齐逻辑（2026-06 修正）

抓取和放置的 joint_4 roll 两套完全对称，互相独立：

- **抓取**：`get_box_edge_roll` 从抓取目标 orientation 提取主边方向，归一化到 `[-π/2, π/2]`
- **放置**：`get_frame_yaw` 从放置框 orientation 提取主边方向，同样归一化到 `[-π/2, π/2]`

视觉服务（手臂相机 `/camera`、狗头相机 `/head_camera`）均返回 `R = [axis_u | axis_v | n_hat]`，第一列 axis_u 为物体主边方向。经 TF 变换到 world 系后，`atan2(R[1,0], R[0,0]) - joint0_ik` 即为 joint_4 需要转的量。
