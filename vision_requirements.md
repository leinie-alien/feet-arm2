# 视觉感知节点需求文档

> 对应 arm2_task 各任务 case 所需的感知节点规格说明。
> 感知节点只负责检测和缓存，不做 TF 变换，坐标系转换由 task_node 完成。

---

## 总览

| 任务 | Case | 服务名 | 检测目标 | 相机 |
|---|---|---|---|---|
| 自动抓取 | 6 | `get_pick_pos` | 被抓箱子上表面 | 手眼相机（Link_4） |
| 自动放置到方框 | 4 | `get_place_pos` | 地面目标方框 | 狗头相机 |
| 自动叠放箱子 | 14 | `get_stack_pos` | 目标箱子上表面 | 狗头相机 |

---

## 1. 抓取感知节点（get_pick_pos）

**已实现**：`/home/leine/data/robotics/260601/camera/src/my_pick_pipeline/`

### 服务定义

```
# robot_msgs/srv/GetPickPos.srv
string object_name
---
bool success
geometry_msgs/PoseStamped pick_pose
```

### 输出

| 字段 | 内容 |
|---|---|
| `pick_pose.header.frame_id` | `"camera_link"` |
| `pick_pose.pose.position` | 箱子上表面中心点（手眼相机坐标系） |
| `pick_pose.pose.orientation` | R = [axis_u, axis_v, n_hat] 构建的 quaternion |

### TF 链路

```
camera_link → Link_4 → world
```

- `camera_link → Link_4`：静态，由 `params.yaml camera_extrinsics` 配置，`control_node` 启动时广播
- `Link_4 → world`：动态，由 `control_node` FK 实时计算广播

---

## 2. 放置感知节点（get_place_pos）

**待实现**，可从 camera 项目复制框架

### 服务定义

```
# robot_msgs/srv/GetPlacePos.srv
string frame_name
---
bool success
geometry_msgs/PoseStamped place_pose
```

### 输出

| 字段 | 内容 |
|---|---|
| `place_pose.header.frame_id` | `"dog_camera_link"` |
| `place_pose.pose.position` | 地面方框中心点（狗头相机坐标系） |
| `place_pose.pose.orientation` | R = [axis_u, axis_v, n_hat] 构建的 quaternion |

### TF 链路

```
dog_camera_link → world
```

**只有一段变换**。机械臂固定安装在机器狗上，`arm_base = world` 原点不变，
因此只需标定**狗头相机相对机械臂基座（= world）的位置和旋转**，不依赖任何狗体 TF。

`control_node` 启动时从 `params.yaml dog_camera_extrinsics` 读取并广播这段静态 TF。

### 外参（需标定后填入 params.yaml）

```yaml
# src/arm2_task/config/params.yaml
dog_camera_extrinsics:
  parent_frame: world           # arm_base = world，直接用
  child_frame:  dog_camera_link
  pos:  [0.0, 0.0, 0.0]        # ← 待标定：相机光心相对 arm_base 的位置 (m)
  quat: [0.0, 0.0, 0.0, 1.0]   # ← 待标定：相机相对 arm_base 的旋转 [qx,qy,qz,qw]
```

**已在代码中实现**：`control_node.cpp::publish_static_dog_camera_tf()` 在启动时自动读取并广播，无需额外脚本。

---

## 3. 叠放感知节点（get_stack_pos）

**待实现**，直接复用 camera 项目，改 3 行即可

### 服务定义

复用 `GetPlacePos.srv`（字段含义相同，服务名不同）：

```
# robot_msgs/srv/GetPlacePos.srv（复用）
string frame_name
---
bool success
geometry_msgs/PoseStamped place_pose   # 此处为目标箱子上表面位姿
```

### 输出

| 字段 | 内容 |
|---|---|
| `place_pose.header.frame_id` | `"dog_camera_link"` |
| `place_pose.pose.position` | 目标箱子上表面中心点（狗头相机坐标系） |
| `place_pose.pose.orientation` | R = [axis_u, axis_v, n_hat] 构建的 quaternion |

### TF 链路

与 get_place_pos 完全相同：`dog_camera_link → world`（共用同一组外参标定值）。

### 从 camera 项目复制的最小改动

```python
# realtime_inference.py 中只改这 3 处：

# 1. frame_id
pose_msg.header.frame_id = "dog_camera_link"   # 原为 "camera_link"

# 2. 服务名和 srv 类型
from robot_msgs.srv import GetPlacePos          # 原为 GetPickPos
srv = node.create_service(GetPlacePos, 'get_stack_pos', handle_get_pick_pose)
# 原为 GetPickPos / 'get_pick_pos'
```

其余全部复用：RANSAC 平面拟合、角点检测、UV 轴估计、quaternion 构建、缓存机制、服务回调结构。

---

## 4. 狗头相机外参标定说明

> 当前代码中唯一的未知量。其余链路全部已实现。

### 需要标定的变换

```
dog_camera_link → world（= arm_base_link）
```

即：**狗头相机光心相对机械臂基座的位置和旋转**。

机械臂安装在狗身上不会移动，这个变换是固定的，只需标定一次。

### 标定方法（最简单）

1. 把机械臂末端移到已知 world 坐标处（例如用 FK 读出末端位置）
2. 在该位置放一个视觉标志物（彩色标记或 ArUco）
3. 让狗头相机检测该标志物，得到其在 `dog_camera_link` 下的坐标 `p_cam`
4. 已知 world 坐标 `p_world`，求解 `T(dog_camera_link → world)` 使得 `T * p_cam = p_world`
5. 多点标定取平均，分别得到 `pos` 和 `quat`

### 填入位置

```yaml
# src/arm2_task/config/params.yaml
dog_camera_extrinsics:
  parent_frame: world
  child_frame:  dog_camera_link
  pos:  [x, y, z]           # 填标定结果 (m)
  quat: [qx, qy, qz, qw]   # 填标定结果
```

重新 `colcon build --packages-select arm2_task` 后生效（`control_node` 启动时自动广播）。

### 误差对任务的影响

| 误差类型 | 影响 | 能否补偿 |
|---|---|---|
| pos 平移误差 | 放置/叠放 XY 整体偏移 | 否，必须标定准确 |
| quat 旋转误差 | 目标越远偏差越大 | 否，必须标定准确 |
| z 误差 | 高度偏移 | 可通过 `contact_offset` 微调 |

---

## 5. 完整 TF 树

```
world（= arm_base_link）
├── dog_camera_link          ← 静态（control_node 广播，dog_camera_extrinsics 配置）
│   用于：get_place_pos / get_stack_pos（感知坐标直接转 world）
│
└── Link_1 ... Link_4        ← 动态（control_node FK 实时广播）
    └── camera_link          ← 静态（control_node 广播，camera_extrinsics 配置）
        用于：get_pick_pos
```

**设计要点**：`dog_camera_link` 和 `camera_link` 都挂在 `world` 下（直接或间接），
`task_node` 侧的 TF lookup 统一为 `lookupTransform("world", frame_id, ...)`，不需要区分。

---

## 6. 调试顺序建议

```
Step 1  用 case 5（manual place）验证 roll_sign
        输入：0.3 0 0 0   → 期望 joint_4 ≈ 0
        输入：0.3 0 0 0.5 → 期望 joint_4 ≈ ±0.5（确认符号）

Step 2  用 case 15（manual stack）验证叠放高度
        输入目标箱子上表面坐标，调 task_stack.contact_offset

Step 3  标定狗头相机外参，填入 params.yaml，重新编译

Step 4  实现 get_place_pos 节点，用 case 4 测试完整放置流程

Step 5  实现 get_stack_pos 节点（复制 get_place_pos 改 3 行），用 case 14 测试叠放
```

---

## 7. 参数对照表

| 参数 | 放置（task_place_frame） | 叠放（task_stack） |
|---|---|---|
| hover_height | 0.25m | 0.05m |
| contact_offset | 0.0m | 0.25m（箱子高度） |
| roll_sign | 1.0（待验证） | 1.0（共用） |
| 感知服务 | get_place_pos | get_stack_pos |
| 目标 frame_id | dog_camera_link | dog_camera_link |
| 外参 | 共用同一组标定值 | 共用同一组标定值 |
