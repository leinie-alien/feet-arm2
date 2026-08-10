# 机械臂远程控制接口说明

> 面向外部控制节点开发者（如机器狗控制系统集成方）。
> 本文档描述机械臂与导航系统之间的实际通信接口。

---

## 1. 运行环境

| 项目 | 说明 |
|---|---|
| ROS2 版本 | Humble |
| 节点名 | `task_manager_node` |
| 启用方式 | `params.yaml` 中设置 `task.manual_mode: false`，然后正常启动机械臂 |

---

## 2. 对外接口

### 2.1 导航 → 机械臂：`/arm/mission_event`

| 字段 | 值 |
|---|---|
| 类型 | `std_srvs/srv/Trigger`（服务） |
| 方向 | 导航 → 机械臂 |

导航到达任务点后调用此服务，机械臂立即返回 `success: true`，然后异步执行抓取序列。

```bash
# 手动触发（测试用）
ros2 service call /arm/mission_event std_srvs/srv/Trigger
```

### 2.2 机械臂 → 导航：`/navigation/arm_event`

| 字段 | 值 |
|---|---|
| 类型 | `navigation/srv/StringCommand`（服务） |
| 方向 | 机械臂 → 导航 |

机械臂在关键节点向导航回报事件：

| 事件 | 含义 | 触发时机 |
|---|---|---|
| `"grabbed"` | 箱子已抓取 | 吸盘 ON + 0.5s 稳定后 |
| `"completed"` | 本次任务完成 | 持箱收起（carry 姿态 + loaded 模式）后 |

---

## 3. 完整交互时序

### 3.1 启动时序

```
机械臂启动
  └─ 等待底层驱动就绪（/robot_driver/ready）
  └─ 执行复位动作（reset 预设姿态）
  └─ 进入 IDLE，等待 /arm/mission_event
```

### 3.2 抓取时序

```
导航到达任务点 → 调用 /arm/mission_event
  └─ 机械臂：moving 模式
  └─ 机械臂：look_out 姿态（朝向 +X 方向）
  └─ 机械臂：调用手眼相机感知（get_pick_pos）
  └─ 机械臂：pre-grasp → grasp（双段轨迹）
  └─ 机械臂：吸盘 ON
  └─ 等待 0.5s（吸附稳定）
  └─ 发送 /navigation/arm_event "grabbed"  ← 导航可认为箱子已抓起
  └─ 机械臂：moving → carry 预设 → loaded 模式
  └─ 发送 /navigation/arm_event "completed" ← 导航可继续移动
```

### 3.3 放置时序

> ⚠️ 放置路径的导航触发尚未接入（代码中 HOLDING → 放置分支已注释）。
> 当前放置操作通过交互菜单手动触发（Case 4/5）。

---

## 4. 状态机（外部控制节点视角）

```
[初始] 等待 arm 就绪
  ↓ 到达抓取点
[调用 /arm/mission_event]
  ↓ 等待 "grabbed"
[箱子已抓取，可继续移动]
  ↓ 等待 "completed"
[机械臂已收起，任务完成]
  ↓ 到达放置点
[手动触发放置（Case 4）或通过交互菜单]
```

---

## 5. 依赖的第三方服务（机械臂内部调用，外部无需处理）

| 服务名 | 提供方 | 说明 |
|---|---|---|
| `get_pick_pos` | 手臂相机节点（neweyes workspace） | 返回箱子在 `camera_link` 系的位姿 |
| `get_place_pos` | 狗头相机节点（neweyes workspace） | 返回放置框在 `dog_camera_link` 系的位姿 |
| `set_suction` | 吸盘控制节点 | 吸盘开关 |

> 这些服务由机械臂节点内部调用，外部控制节点**不需要**直接对接。
> 但需要确保这些节点在机械臂执行任务前已启动。

---

## 6. 调试监控命令

```bash
# source 环境
source /opt/ros/humble/setup.bash
source feet-arm2/install/setup.bash

# 手动触发抓取
ros2 service call /arm/mission_event std_srvs/srv/Trigger

# 查看所有活跃话题
ros2 topic list -v

# 查看机械臂状态
ros2 topic echo /arm2/_lowState/joint
```

---

## 7. 测试脚本

位置：`tools/remote_control_test/arm_controller.py`

> ⚠️ 此脚本基于旧版话题接口（`/arm/cmd` + `/arm/status`）编写，当前版本的接口已改为服务调用（`/arm/mission_event`），脚本待更新。

---

## 8. 注意事项

1. **启动顺序**：相机节点、吸盘节点需在机械臂之前或同时启动，否则感知服务超时。
2. **busy 保护**：机械臂单次只处理一个任务，执行中新触发会被忽略。
3. **模式切换**：`task.manual_mode: false` 启用导航模式，`true` 启用交互菜单模式。
4. **放置路径**：当前导航模式下只实现了抓取，放置需手动触发。