# Handeye ChArUco 标定说明

这套代码分成两部分：

- `tools/` 负责离线求解和导出相机内参
- `src/arm2_task/src/handeye_charuco_capture_node.cpp` 负责在线采集标定数据

所以，**只发 `tools/` 不够**。如果对方只想重新计算外参，`tools/` 加上已有的 `recordings/handeye_charuco_capture/dataset.yml` 就够；如果要重新采集数据，还需要把采集节点、参数文件和启动脚本一起发过去。

## 最小需要发送的文件

只做离线求解外参：

- `tools/solve_handeye_charuco.py`
- `tools/export_realsense_intrinsics.py`
- `recordings/handeye_charuco_capture/dataset.yml`
- `recordings/handeye_charuco_capture/*.png` 或 `images/` 目录

重新采集 + 求解：

- 上面这些文件
- `src/arm2_task/src/handeye_charuco_capture_node.cpp`
- `src/arm2_task/config/params.yaml`
- `run_handeye_charuco_capture_real.sh`

## 流程

1. 用 `export_realsense_intrinsics.py` 导出相机内参。
2. 运行 `handeye_charuco_capture_node` 采集多组 ChArUco 图像和机器人位姿。
3. 用 `solve_handeye_charuco.py` 读取 `dataset.yml`，求解 `Link_4 -> camera_link` 外参。
4. 把脚本输出的 `camera_extrinsics` 写回 `params.yaml`。

## 用法

导出内参：

```bash
python3 tools/export_realsense_intrinsics.py --output recordings/handeye_charuco_capture/device_intrinsics.yml
```

求解外参：

```bash
python3 tools/solve_handeye_charuco.py \
  --dataset recordings/handeye_charuco_capture/dataset.yml \
  --output recordings/handeye_charuco_capture/handeye_result.yml
```

## 输出结果

脚本会输出：

- `Link_4 -> camera_link` 平移 `pos`
- `Link_4 -> camera_link` 四元数 `quat`

可以直接写到：

```yaml
camera_extrinsics:
  parent_frame: Link_4
  child_frame: camera_link
  pos: [...]
  quat: [...]
```

## 依赖

- ROS 2 Humble
- OpenCV + `aruco`
- Python 3
- `numpy`
- `pyrealsense2`（只在导出 RealSense 内参时需要）
