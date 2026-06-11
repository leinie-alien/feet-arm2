# 机械臂远程控制接口说明

> 面向外部控制节点开发者（如机器狗控制系统集成方）。
> 本文档描述机械臂 ROS2 节点对外暴露的通信接口，以及完整的交互时序。

---

## 1. 运行环境

| 项目 | 说明 |
|---|---|
| ROS2 版本 | Humble |
| 节点名 | `task_manager_node` |
| 启用方式 | `params.yaml` 中设置 `task.remote_mode: true`，然后正常启动机械臂 |

---

## 2. 对外话题接口

### 2.1 命令话题（外部 → 机械臂）

| 字段 | 值 |
|---|---|
| 话题名 | `/arm/cmd` |
| 消息类型 | `std_msgs/msg/String` |
| QoS | 默认（reliable, volatile, depth=10） |

**支持的命令字符串：**

| 命令 | 触发动作 |
|---|---|
| `"grasp"` | 执行完整抓取序列（感知 → 抓取 → 收起） |
| `"place"` | 执行完整放置序列（感知 → 放置 → 复位） |

> **注意**：命令在执行期间发送新命令会被拒绝，机械臂返回 `"error:busy"`。

---

### 2.2 状态话题（机械臂 → 外部）

| 字段 | 值 |
|---|---|
| 话题名 | `/arm/status` |
| 消息类型 | `std_msgs/msg/String` |
| QoS | 默认（reliable, volatile, depth=10） |

**状态字符串定义：**

| 状态 | 含义 | 触发时机 |
|---|---|---|
| `"reset"` | 机械臂已复位到待机姿态 | 启动完成后 / place 序列结束后 |
| `"grasped"` | 吸盘已吸取箱子 | suction ON 后 0.5s |
| `"stowed"` | 机械臂已收起（携带箱子） | carry 预设到位后 |
| `"placed"` | 箱子已放下，机械臂已离开放置点 | suction OFF + 后退完成后 |
| `"error:busy"` | 收到命令时机械臂正在执行任务 | 执行中收到新命令时 |
| `"error:perception_failed"` | 抓取感知失败 | get_pick_pos 服务调用失败 |
| `"error:grasp_failed"` | 抓取动作失败（IK 或运动失败） | do_grasp_move 返回 false |
| `"error:carry_failed"` | 收起动作失败 | carry 轨迹执行失败 |
| `"error:place_perception_failed"` | 放置感知失败 | get_place_pos 服务调用失败 |
| `"error:place_failed"` | 放置动作失败 | do_place_move 返回 false |
| `"error:unknown_cmd:<cmd>"` | 收到未知命令 | 命令不是 grasp/place 时 |

---

## 3. 完整交互时序

### 3.1 启动时序

```
机械臂启动
  └─ 等待底层驱动就绪（/robot_driver/ready）
  └─ 执行复位动作（reset 预设姿态）
  └─ 发布 "reset"
  └─ 进入 IDLE，等待命令
```

### 3.2 抓取时序（发送 "grasp"）

```
外部节点发送 "grasp"
  └─ 机械臂：look_out 姿态（朝向目标方向）
  └─ 机械臂：调用手臂相机感知（get_pick_pos）
  └─ 机械臂：pre-grasp → grasp（双段轨迹）
  └─ 机械臂：吸盘 ON
  └─ 等待 0.5s（吸附稳定）
  └─ 发布 "grasped"          ← 外部节点可在此时认为箱子已被抓起
  └─ 机械臂：moving → carry 预设 → loaded 模式
  └─ 发布 "stowed"            ← 外部节点可在此时开始移动机器狗
```

### 3.3 放置时序（发送 "place"）

```
外部节点发送 "place"
  └─ 机械臂：调用狗头相机感知（get_place_pos）
  └─ 机械臂：moving 模式
  └─ 机械臂：pre-place → place（双段轨迹）
  └─ 机械臂：吸盘 OFF
  └─ 机械臂：垂直后退（离开放置点上方）
  └─ 发布 "placed"            ← 外部节点可在此时认为箱子已安全放下
  └─ 机械臂：复位到 reset 预设姿态
  └─ 发布 "reset"             ← 外部节点可在此时认为机械臂完全空闲
```

### 3.4 错误处理

```
任意步骤失败
  └─ 发布 "error:<原因>"
  └─ 机械臂进入 IDLE（不继续后续步骤，不自动复位）
  └─ 外部节点需决定是否重试或发送其他命令
```

---

## 4. 典型状态机（外部控制节点视角）

```
[初始] 等待 "reset"
  ↓ 收到 "reset"
[IDLE] 等待任务指令
  ↓ 决定抓取 → 发送 "grasp"
[WAITING_GRASP] 等待 "grasped"
  ↓ 收到 "grasped"（箱子已在吸盘上）
[WAITING_STOW] 等待 "stowed"
  ↓ 收到 "stowed"（机械臂已收起，可移动）
[MOVING] 机器狗移动到放置目标位置
  ↓ 到位 → 发送 "place"
[WAITING_PLACE] 等待 "placed"
  ↓ 收到 "placed"（箱子已放下）
[WAITING_RESET] 等待 "reset"
  ↓ 收到 "reset"
[IDLE] 任务完成
```

---

## 5. 依赖的第三方服务（机械臂内部调用，外部无需处理）

| 服务名 | 提供方 | 说明 |
|---|---|---|
| `get_pick_pos` | 手臂相机节点（`/home/zyy/task/arm/camera`） | 返回箱子在 `camera_link` 系的位姿 |
| `get_place_pos` | 狗头相机节点（`/home/zyy/task/arm/head_camera`） | 返回放置框在 `dog_camera_link` 系的位姿 |
| `set_suction` | 吸盘控制节点 | 吸盘开关 |

> 这些服务由机械臂节点内部调用，外部控制节点**不需要**直接对接。
> 但需要确保这些节点在机械臂执行任务前已启动。

---

## 6. 调试监控命令

```bash
# source 环境（每个终端都需要）
source /opt/ros/humble/setup.bash
source ~/task/arm/feet-arm2-main/install/setup.bash

# 实时监听机械臂状态广播
ros2 topic echo /arm/status

# 实时监听发给机械臂的命令
ros2 topic echo /arm/cmd

# 查看所有活跃话题
ros2 topic list -v

# 手动发送命令（测试用）
ros2 topic pub --once /arm/cmd std_msgs/msg/String "{data: 'grasp'}"
ros2 topic pub --once /arm/cmd std_msgs/msg/String "{data: 'place'}"
```

---

## 7. 测试脚本

位置：`tools/remote_control_test/arm_controller.py`

```bash
# 交互模式
python3 tools/remote_control_test/arm_controller.py

# 单次执行后退出
python3 tools/remote_control_test/arm_controller.py --once grasp
python3 tools/remote_control_test/arm_controller.py --once place
```

脚本会等待每个阶段的状态广播（带超时），适合验证完整流程是否正常。

---

## 8. 注意事项

1. **启动顺序**：手臂相机节点、狗头相机节点、吸盘节点需在机械臂节点之前或同时启动，否则感知服务调用超时会报 error。
2. **重试策略**：收到 `error:*` 后机械臂处于 IDLE，外部节点可直接重发命令重试；但建议先确认错误原因。
3. **busy 保护**：机械臂单次只处理一个命令，并发命令会被拒绝，外部节点需等收到终态（`stowed`/`reset`/`error:*`）后再发下一条命令。
4. **place 前提**：放置命令假设机械臂当前持有箱子（吸盘 ON，carry 姿态）。顺序必须是 grasp → stowed → place，否则放置动作没有意义。
