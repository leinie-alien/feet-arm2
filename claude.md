# Claude 协作笔记

## 项目结构

| workspace | 路径 | 作用 |
|---|---|---|
| arm | `/home/leine/data/robotics/arm` | 控制栈（task_node / control_node / dm_motor_sdk_ros） |
| neweyes | `/home/leine/data/robotics/neweyes` | 视觉感知栈（get_pick_pos 服务端，realtime_inference.py） |

启动命令：`bash run_arm.sh`（真机），`bash run_arm.sh --sim`（仿真）

---

## Bug 记录

### [2025] 段错误：`ros2 run` task_node 在 cmd:6 Auto Grasp 时崩溃

**现象**
```
[INFO] [task_manager_node]: >>> Auto Grasp
[INFO] [task_manager_node]: [Mode] Controller switched to: moving
[INFO] [task_manager_node]: [look_out] yaw toward (1.000, 0.000)
[ros2run]: Segmentation fault
```

**根本原因**

两个 workspace 的 `robot_msgs/srv/GetPickPos.srv` 不一致：

- `arm` workspace（task_node 消费方）：response = `bool success` + `geometry_msgs/PoseStamped pick_pose`
- `neweyes` workspace（感知服务提供方）：response = `geometry_msgs/PoseStamped pick_pose`（旧版，无 success 字段）

ROS2 消息序列化按字段顺序排列内存，两边布局错位，task_node 解析 response 时访问非法地址 → 段错误。

**修复**

1. `neweyes/src/robot_msgs/srv/GetPickPos.srv` — 加 `bool success` 字段与 arm 同步
2. `neweyes/.../realtime_inference.py` — 服务回调填充 `response.success = True/False`
3. `arm/src/arm2_task/src/task_node.cpp` `call_pick_service_sync()` — 加 null 和 `!success` 防护

**教训**

> 两个 workspace 共用同一份 `robot_msgs` 的 `.srv`/`.msg` 时，任意一边修改后**必须同步另一边并重新编译**，否则序列化布局错位，症状是运行时段错误而非编译错误，极难排查。

---

## 关键文件速查

| 文件 | 说明 |
|---|---|
| `src/arm2_task/src/task_node.cpp` | 主任务节点，含抓取/放置/视觉对齐逻辑 |
| `src/arm2_task/config/params.yaml` | 所有可调参数（tool offset、pitch、preset 角度等） |
| `src/dm_motor_sdk_ros/src/dm_motor_robot_driver_node.cpp` | 达妙电机 CAN 驱动节点 |
| `src/robot_msgs/srv/GetPickPos.srv` | 感知服务接口（**与 neweyes workspace 必须保持同步**） |
| `neweyes/src/my_pick_pipeline/my_pick_pipeline/realtime_inference.py` | 视觉感知服务端 |
