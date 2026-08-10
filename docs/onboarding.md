# 机械臂基础与控制 — 新队员入门指南

> 本文档面向新加入的队员，梳理本项目（5-DOF 机械臂）涉及的前备知识体系和实现考量。
> 每个章节先讲理论要点，再说明本项目是怎么做的、为什么这样做。

---

## 目录

- [Part 1 — 前备知识](#part-1--前备知识)
  - [1.1 刚体变换与矩阵](#11-刚体变换与矩阵)
  - [1.2 SolidWorks → URDF](#12-solidworks--urdf)
  - [1.3 正向运动学 (FK)](#13-正向运动学-fk)
  - [1.4 逆向运动学 (IK)](#14-逆向运动学-ik)
  - [1.5 视觉引导感知](#15-视觉引导感知)
  - [1.6 轨迹规划](#16-轨迹规划)
  - [1.7 控制基础](#17-控制基础)
  - [1.8 电机驱动](#18-电机驱动)
- [Part 2 — 项目实现](#part-2--项目实现)
  - [2.1 架构总览](#21-架构总览)
  - [2.2 运动学引擎](#22-运动学引擎)
  - [2.3 视觉感知管线](#23-视觉感知管线)
  - [2.4 控制模式设计](#24-控制模式设计)
  - [2.5 任务编排](#25-任务编排)

---

## Part 1 — 前备知识

### 1.1 刚体变换与矩阵

#### 理论要点

机械臂的每一根连杆都是一个**刚体**。描述刚体在空间中的位置和姿态，用的是**齐次变换矩阵**（Homogeneous Transformation Matrix）：

```
T = ┌        ┐
    │ R   p  │
    │ 0   1  │
    └        ┘
```

其中 R 是 3×3 旋转矩阵，p 是 3×1 平移向量。

**需要掌握的概念：**

- 旋转矩阵（SO(3)）—— 绕 X/Y/Z 轴的旋转
- 齐次坐标 —— 为什么加一维
- 变换的复合 —— 矩阵乘法，从右到左
- **DH 参数**（Denavit-Hartenberg）—— 用 4 个参数描述相邻关节之间的变换，是机械臂建模的标准方法

**推荐资源：**

- [3Blue1Brown — Linear Algebra 系列](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab)：直观理解线性变换
- [Modern Robotics 第 3 章](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)：免费的在线教材，Rigid-Body Motions 讲得非常好
- [Peter Corke — Robotics Toolbox 文档](https://petercorke.com/toolboxes/robotics-toolbox/)：MATLAB/Python 机器人工具箱，有大量示例

#### 本项目的情况

本项目 5 个关节全是旋转关节，DH 参数体现为 URDF 文件中的 `<joint>` 标签。实际计算用的是 Pinocchio 库，不需要手动写 DH 矩阵，但理解 DH 参数才能看懂 URDF 里每个关节的 `origin` 标签（`xyz` 是位置，`rpy` 是姿态）。

代码位置：`src/arm2_task/urdf/arm2.urdf`

---

### 1.2 SolidWorks → URDF

#### 理论要点

URDF（Unified Robot Description Format）是 ROS 中描述机器人结构的 XML 格式。从 CAD 到 URDF 的典型流程：

```
SolidWorks 装配体
  → 导出每个连杆为 STL（网格文件）
  → 写 URDF XML（定义连杆、关节、惯量）
  → 在 ROS 中加载
```

**URDF 核心标签：**

```xml
<link name="Link_1">
  <visual>
    <geometry><mesh filename="..." /></geometry>  <!-- 用于显示 -->
  </visual>
  <collision>
    <geometry><mesh filename="..." /></geometry>  <!-- 用于碰撞检测 -->
  </collision>
  <inertial>
    <mass value="..." />                           <!-- 质量 (kg) -->
    <inertia ixx="..." iyy="..." izz="..." />      <!-- 转动惯量 -->
  </inertial>
</link>

<joint name="Joint_1" type="revolute">            <!-- 旋转关节 -->
  <parent link="Link_0" />
  <child link="Link_1" />
  <origin xyz="0 0 0.0845" rpy="0 0 0" />        <!-- 相对父连杆的位姿 -->
  <axis xyz="0 0 1" />                             <!-- 旋转轴 -->
  <limit lower="-4.188" upper="4.188" effort="..." velocity="..." />
</joint>
```

**关键注意事项：**

- `origin` 的 `rpy` 是 Roll-Pitch-Yaw，顺序是绕 X → 绕 Y → 绕 Z（固定轴）
- 惯量矩阵（inertia）必须正定，否则 Pinocchio 等库会报错。SolidWorks 可以自动计算
- STL 文件路径是相对于 URDF 文件所在目录的

**推荐资源：**

- [ROS2 URDF 官方教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [SolidWorks to URDF 插件](http://wiki.ros.org/sw_urdf_exporter)（ROS1 时代，但概念通用）

#### 本项目的情况

本项目的 URDF 在 `src/arm2_task/urdf/arm2.urdf`，STL 网格文件在 `src/arm2_task/meshes/`。几何参数（连杆长度）与 `params.yaml` 中的 `robot_geometry` 严格一致，修改任何一个都要同步更新另一个。

---

### 1.3 正向运动学 (FK)

#### 理论要点

**正向运动学**：给定每个关节的角度，计算末端执行器在世界坐标系中的位置和姿态。

对于串联机械臂，FK 就是沿着运动链把所有 DH 变换矩阵乘起来：

```
T_world_to_tip = T_0→1 · T_1→2 · T_2→3 · T_3→4 · T_4→5
```

**推荐资源：**

- [Modern Robotics 第 4 章 Forward Kinematics](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- 任何机器人学教材的 FK 章节

#### 本项目的情况

本项目使用 **Pinocchio** 库做 FK，不需要手写矩阵乘法。Pinocchio 从 URDF 加载模型后，调用 `forwardKinematics(model, data, q)` 即可得到所有连杆的位姿。

代码位置：`src/arm2_task/src/kinematics_engine.cpp` 中的 `forwardKinematics()`

返回的是 **Link_4** 的位姿（不是末端 Link_5），因为相机安装在 Link_4 上。`control_node` 拿到 FK 结果后，发布 `world → Link_4` 的动态 TF。

---

### 1.4 逆向运动学 (IK)

#### 理论要点

**逆向运动学**：给定末端执行器的目标位姿，反算每个关节的角度。

IK 比 FK 难得多，因为：
- 存在多解（肘关节可以向上或向下）
- 可能无解（目标超出工作空间）
- 可能有无穷多解（冗余自由度）

**常见方法：**

| 方法 | 原理 | 优缺点 |
|---|---|---|
| **解析法** | 几何三角推导，直接算出每个关节角 | 快、准，但只适用于特定构型 |
| **数值法（雅可比迭代）** | 用雅可比矩阵伪逆迭代逼近 | 通用，但慢、可能不收敛 |
| **优化法** | 用非线性优化求解 | 灵活，但计算量大 |

**推荐资源：**

- [Modern Robotics 第 6 章 Inverse Kinematics](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- 解析法示例：搜索 "3-DOF planar arm inverse kinematics geometry"

#### 本项目的情况

本项目用的是**解析法**，因为前 3 个关节（Yaw + Pitch1 + Pitch2）构成一个平面三连杆，可以用几何法直接求解：

```
已知：目标 world 坐标 (x, y, z)，去掉 L1 基座高度后得到腕部坐标 (x_w, y_w, z_w)

1. q0 = atan2(y_w, x_w)                        ← 基座旋转朝向目标
2. 在 XZ 平面用余弦定理求 q1、q2（肘向下分支）    ← 大臂 + 小臂
3. q3 = φ - q1 - q2                             ← 腕部调整姿态
4. q4 = 0（初始值）                              ← 末端 roll，后续用视觉对齐
```

代码位置：`src/arm2_task/src/kinematics_engine.cpp` 中的 `solveIK()` 和 `solvePlanar3Link()`

**为什么选解析法？** 因为我们 5 个关节中有 3 个是平面连杆，几何结构简单，解析法又快又不会发散。数值法反而可能因为初始猜测不好而收敛到错误解。

---

### 1.5 视觉引导感知

#### 理论要点

视觉引导机械臂的完整链路：

```
相机图像 → 物体检测 → 物体在相机坐标系中的位姿
  → TF 变换到世界坐标系 → 目标位姿 (world)
  → IK 求解 → 关节角 → 执行运动
```

**需要掌握的核心概念：**

**a) 相机模型与内参**

针孔相机模型：3D 世界点 → 2D 像素点。内参矩阵 K 描述焦距、光心、畸变。

**b) 手眼标定（Eye-in-Hand Calibration）**

相机装在机械臂上（"眼在手"），需要知道相机相对机械臂末端的位姿 `T(Link_4 → camera_link)`。这就是 `params.yaml` 里的 `camera_extrinsics`。

经典方法：机械臂移动到多个不同姿态，每拍一张棋盘格，求解 AX = XB 问题。

**c) 物体姿态估计**

从 RGB-D 图像中检测物体并估计其 6D 位姿。本项目用的是 RANSAC 平面拟合 + 角点检测。

**d) TF 坐标变换**

拿到物体在相机坐标系中的位姿后，利用 TF 树变换到世界坐标系：

```
T(world → object) = T(world → Link_4) · T(Link_4 → camera_link) · T(camera_link → object)
                     ↑ 动态 FK              ↑ 静态外参                  ↑ 感知输出
```

**推荐资源：**

- [OpenCV 相机标定教程](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [手眼标定综述](https://campar.in.tum.de/Chair/HandEyeCalibration) — TUM 的经典综述
- [ROS2 TF2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- `tf2_ros` 和 `tf2_geometry_msgs` 是 ROS 中做坐标变换的标准工具

#### 本项目的情况

**外参：** 手眼相机外参已经标定好，填在 `params.yaml` 的 `camera_extrinsics` 中。`control_node` 启动时自动广播这段静态 TF。狗头相机外参同理。

**感知服务：** 感知节点在独立的 "neweyes" workspace 中运行，提供 ROS2 服务：
- `get_pick_pos` — 返回箱子上表面在 `camera_link` 中的位姿
- `get_place_pos` — 返回方框在 `dog_camera_link` 中的位姿

**TF 变换：** `task_node` 收到感知结果后，用 TF2 做 `lookupTransform("world", frame_id)` + `doTransform()`，把位姿转到 world 坐标系，然后喂给 IK。代码在 `task_node.cpp` 的 `call_pick_service_sync()` 中。

**⚠️ 关键教训：** `GetPickPos.srv` 在 arm 和 neweyes 两个 workspace 中各有一份，必须字段完全一致，否则 ROS2 序列化布局错位 → 运行时 Segmentation Fault。修改任意一边的 `.srv` 后必须同步另一边。

**三阶段管线**（详见 [2.3 节](#23-视觉感知管线)）是我们提高精度的核心设计。

---

### 1.6 轨迹规划

#### 理论要点

机械臂不能瞬间从一个角度跳到另一个角度——需要规划一条平滑的轨迹，受速度和加速度限制。

**常见轨迹类型：**

| 类型 | 特点 |
|---|---|
| 梯形速度 | 加速 → 匀速 → 减速，简单但加速度不连续 |
| **五次多项式** | 位置、速度、加速度都连续，平滑 |
| S 曲线 | 加加速度也连续，更平滑但更复杂 |
| 样条插值 | 通过多个中间点 |

**轨迹参数：** 最大速度 `v_max`、最大加速度 `a_max`、轨迹时长 `T`

**多段混合（Blend）：** 当连续执行多个 waypoint 时，不用停到前一个终点再出发，而是在接近终点时就开始混合下一段，使运动更连续。

**推荐资源：**

- [Modern Robotics 第 9 章 Trajectory Generation](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- Search: "quintic polynomial trajectory generation"

#### 本项目的情况

本项目使用**五次多项式**插值（不是梯形！`plan_trapezoid()` 名字有误导）。位置曲线的形状是 `s(t) = 10t³ - 15t⁴ + 6t⁵`（归一化时间），保证起点和终点的速度、加速度都为 0。

**多段混合：** `control_node` 支持 blend radius —— 当前段未结束时就开始混入下一段，避免停顿。

**参数：** `params.yaml` 中 `trajectory_planner.max_velocity: 0.5`、`max_acceleration: 1.0`、`min_segment_duration: 0.3`

代码位置：`control_node.cpp` 中的 `plan_trapezoid()` 和 `compute_segment_state()`

---

### 1.7 控制基础

#### 理论要点

机械臂控制的核心问题：给定目标关节角 `q_des`，计算电机应该输出的力矩 `τ`。

**a) PD 控制（反馈）**

最基本的反馈控制：
```
τ = Kp · (q_des - q_actual) + Kd · (0 - dq_actual)
```
- `Kp`（比例增益）：位置误差越大，力矩越大。过大会振荡，过小会有静差
- `Kd`（微分增益）：阻尼项，抑制振荡。过大会发热，过小会超调

**b) 前馈控制**

PD 只能"等出了误差再修正"。前馈提前算好需要的力矩：
```
τ = τ_gravity + τ_friction + τ_PD
```
- `τ_gravity`（重力补偿）：用动力学模型算出让机械臂不掉下来需要的力矩
- `τ_friction`（摩擦力补偿）：`fc · tanh(α · dq) + fv · dq`

**c) 逆动力学（RNEA）**

RNEA（Recursive Newton-Euler Algorithm）是计算前馈力矩的标准算法。给定 `(q, dq, ddq)`，算出每个关节需要的力矩。项目使用 Pinocchio 库实现。

**d) 控制模式切换**

不同场景需要不同的增益：
- 手持示教：kp 极低，让人能推动
- 空载运动：标准增益
- 带负载：更高增益，提高刚性

**推荐资源：**

- [Brian Douglas — PID 控制](https://www.youtube.com/watch?v=UR0hOmjaHp0)：非常直观的 PID 讲解
- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)：RNEA 和 ABA 的使用
- [Modern Robotics 第 8 章 Dynamics](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)

#### 本项目的情况

本项目实现了 6 种控制模式（见 [2.4 节](#24-控制模式设计)），前馈用的是 `RNEA(desired) + friction(actual)`。

摩擦力模型：`tau_f = fc · tanh(α · dq) + fv · dq · GearRatio²`
- `tanh` 函数让库伦摩擦在零速附近平滑过渡，避免力矩跳变
- `GearRatio²` 是因为摩擦力在电机侧，需要折算到关节侧

代码位置：
- `control_node.cpp` — Stack A 的 PD + 前馈控制
- `inverse_dynamics_node.cpp` — Stack B 的逆动力学控制
- `dynamics_manager.cpp` — RNEA 和摩擦力计算

---

### 1.8 电机驱动

> *（待填充）本章节将介绍达妙电机的 CAN/CANFD 通信协议、MIT 控制模式、以及 USB2CANFD_Dual 适配器的使用。*

---

## Part 2 — 项目实现

### 2.1 架构总览

```
┌─────────────────────────────────────────────────────────┐
│  task_node                                              │
│  "大脑" — 任务编排、交互菜单、导航集成、视觉对齐         │
│  输入：感知结果 (world 坐标)、用户命令、nav 触发         │
│  输出：MoveJoint action (关节目标)                       │
└──────────────┬──────────────────────────────────────────┘
               │ MoveJoint action
               ▼
┌─────────────────────────────────────────────────────────┐
│  control_node / inverse_dynamics_node                   │
│  "小脑" — 轨迹插值 + FK/IK + 前馈控制 + PD              │
│  输入：MoveJoint target                                 │
│  输出：/arm2/_lowCmd/command (关节力矩)                   │
└──────────────┬──────────────────────────────────────────┘
               │ MIT 控制命令
               ▼
┌─────────────────────────────────────────────────────────┐
│  dm_motor_sdk_ros                                       │
│  "脊髓" — CAN 帧收发、2 轴选圈、限位、跳变检测          │
│  输入：力矩命令                                          │
│  输出：关节状态 /arm2/_lowState/joint                    │
└──────────────────────────────────────────────────────────┘
```

**数据流：** 感知位姿 (world) → IK → 关节角 target → 轨迹插值 → 前馈 + PD → 力矩命令 → 电机

**两套控制栈：**

| | Stack A（生产） | Stack B（实验/示教） |
|---|---|---|
| 控制器 | `control_node` | `inverse_dynamics_node` |
| 控制模式 | 4 种 | 6 种（多 teach_pendant、teach_drag） |
| 启动 | `run_arm.sh` 默认 | `planner_inverse_task_launch.py` |

---

### 2.2 运动学引擎

**设计决策：为什么用解析法 IK？**

本项目的 5-DOF 构型中，前 3 个关节（Yaw + Pitch1 + Pitch2）形成平面三连杆，正好可以用余弦定理直接求解。解析法在这个场景下：
- 计算快（几个三角函数，不需要迭代）
- 不会发散（数值法需要好的初始猜测）
- 解唯一（肘向下分支，固定取 `q2 = -acos(cos_q2)` 即负值）

**IK 计算流程：**

```
1. 分离基座旋转：q0 = atan2(y_w, x_w)
2. 在 XZ 平面投影，用余弦定理：
   d² = x_w² + z_w²（去掉 L1 后）
   cos(q2) = (d² - l2² - l3²) / (2·l2·l3)
   q2 = -acos(cos_q2)        ← 固定肘向下
   q1 = atan2(z_w, x_w) - atan2(l3·sin(q2), l2 + l3·cos(q2))
3. 腕部：q3 = target_pitch - q1 - q2
4. q4 = 0（初始，后续用视觉对齐调整）
```

**阻尼最小二乘（Damped Least Squares）：** 当需要速度级 IK 时（如视觉对齐），使用 `(JJᵀ + λ²I)⁻¹` 避免奇异点附近的数值爆炸。

代码位置：`src/arm2_task/src/kinematics_engine.cpp`

---

### 2.3 视觉感知管线

**为什么设计三阶段？**

单次感知 → 直接抓取的问题：
- 瞭望时相机离目标远，位姿估计有误差
- 一次感知没有机会纠正

三阶段管线把精度问题拆解：

```
Phase 1 — 扫描（Scan）
  瞭望姿态 → 粗定位目标
  目的：知道目标大概在哪，误差 5-10cm 都可以接受

Phase 2 — 对齐（Align）
  切换到俯瞰姿态 → 固定 Z 高度
  → 每轮：感知当前物体 XY → 与末端 XY 比较误差
  → 只调整 joint_0 → 重复直到误差 < 5mm（最多 5 轮）
  目的：XY 闭环对齐，消除感知 → 运动链的累积误差

Phase 3 — 下降执行（Descend）
  joint_4 旋转 -90° → 确认目标 → 执行抓取/放置
  目的：在确认对齐后执行，避免误抓
```

**统计对齐（Case 6 的增强）：** 3 次感知采样，取 x/y/z/roll 各自的中值，减少单次感知的随机误差。

**关键参数：** `visual_align.align_threshold: 0.005`（5mm 收敛阈值）、`visual_align.max_iters: 5`

---

### 2.4 控制模式设计

6 种模式对应 6 组 PD 增益，设计思路：

| 模式 | 设计目标 | 为什么这样设 |
|---|---|---|
| `idle` | 待机省电，允许手动推 | kp 极低，不会跟人"较劲" |
| `gravity_comp` | 零力示教，手拖机械臂 | 前馈抵消重力，kp≈0 让人能推动 |
| `moving` | 空载轨迹跟踪 | 中等 kp/kd，兼顾响应和稳定 |
| `loaded` | 带箱子运动 | 更高 kp，克服箱子惯量，减少静差 |
| `teach_pendant` | 实时跟随示教器 | 响应快，但不要振荡 |
| `teach_drag` | 手拖录制轨迹 | kp≈0，完全靠前馈 |

**为什么有 6 组而不是调一组？** 不同场景的物理条件差异太大——空载和带负载需要的刚性完全不同，一组参数没法同时满足。

**摩擦力模型为什么用 `tanh`？** 库伦摩擦在零速时有符号跳变，直接用 `sign(dq)` 会让力矩在零速附近剧烈振荡。`tanh(α·dq)` 在零速附近平滑过渡，消除了这个不稳定因素。

---

### 2.5 任务编排

**抓取为什么分两段（pre-grasp → grasp）？**

```
pre-grasp（悬停）：末端在目标上方 pre_grasp_offset (0.10m) 处
  → 确认位置正确，没有碰撞风险
grasp（接触）：末端垂直下降到 object_height + tool_offset_z
  → 吸盘接触箱子表面
```

分段的好处：如果 pre-grasp 位置有问题，可以在碰到箱子之前停下来，不会撞到东西。

**为什么用 `get_box_edge_roll` 而不是直接用感知的朝向？**

感知给出的箱子朝向可能有 90° 的歧义（箱子是正方形顶面）。`get_box_edge_roll` 利用这个对称性：把感知角度映射到 `[-π/2, 0]` 区间，选择最近的边作为抓取方向，始终保证吸盘与箱子边缘对齐。

**mode 切换时机：**

```
idle → moving（开始运动）→ loaded（抓取后，高刚性持箱）→ moving（放置后）→ idle（任务结束）
```

每次切换都有明确的物理意义：低刚性等待 → 标准运动 → 负载运动 → 恢复。

---

## 附录：快速上手路线

建议新队员按以下顺序学习：

1. **先跑起来**：`bash run_arm.sh --sim`，体验交互菜单的 15 种操作
2. **读 Part 1.1-1.4**：理解 FK/IK/URDF，对照 `arm2.urdf` 和 `kinematics_engine.cpp`
3. **读 Part 1.7**：理解 PD + 前馈，对照 `control_node.cpp`
4. **读 Part 2.3**：理解三阶段管线，在仿真中跑 Case 12 看 Phase 2 对齐过程
5. **读 Part 1.5**：理解视觉引导，对照 `task_node.cpp` 的感知调用和 TF 变换
6. **读 Part 2.5**：理解任务编排，在真机上调试抓取参数

遇到不懂的概念，回到本文档对应的 Part 1 章节查阅推荐资源。