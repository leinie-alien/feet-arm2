#!/usr/bin/env bash
# run_neweyes.sh — neweyes 视觉节点一键启动
# 用法：
#   bash run_neweyes.sh          # 正常启动
#   bash run_neweyes.sh --build  # 先编译再启动
#   bash run_neweyes.sh -h       # 查看帮助
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
AUTO_BUILD="${AUTO_BUILD:-false}"

# ===== 帮助 =====
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "用法: bash run_neweyes.sh [--build]"
  echo "  --build   启动前先执行 colcon build"
  echo "  环境变量 WS_DIR 可覆盖工作空间路径（默认脚本所在目录）"
  exit 0
fi

[[ "${1:-}" == "--build" ]] && AUTO_BUILD=true

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# ===== 检查 ROS 2 =====
[[ -f "$ROS_SETUP" ]] || { echo "[neweyes] ERROR: 找不到 ROS 2 Humble ($ROS_SETUP)" >&2; exit 1; }
source_setup "$ROS_SETUP"

# ===== 可选编译 =====
if [[ "$AUTO_BUILD" == "true" ]]; then
  echo "[neweyes] 编译中..."
  (cd "$WS_DIR" && colcon build --symlink-install 2>&1)
  echo "[neweyes] 编译完成"
fi

# ===== source arm workspace 的 robot_msgs（两个 workspace 共用同一份消息定义，避免 DDS 类型不匹配）=====
ARM_WS="${ARM_WS:-$HOME/data/robotics/arm}"
ARM_SETUP="$ARM_WS/install/setup.bash"
if [[ -f "$ARM_SETUP" ]]; then
  source_setup "$ARM_SETUP"
else
  echo "[neweyes] WARN: arm workspace ($ARM_SETUP) 未找到，将使用本地 robot_msgs（可能导致服务通信失败）" >&2
fi

# ===== source 工作空间 =====
[[ -f "$WS_SETUP" ]] \
  || { echo "[neweyes] ERROR: install/setup.bash 不存在，请先编译（--build 或 colcon build）" >&2; exit 1; }
source_setup "$WS_SETUP"

# ===== 激活 venv 中的第三方库（pyrealsense2、ultralytics、opencv 等）=====
VENV_SITE="$WS_DIR/.venv/lib/python3.10/site-packages"
[[ -d "$VENV_SITE" ]] || { echo "[neweyes] ERROR: 找不到 venv，请先运行: python3 -m venv $WS_DIR/.venv --system-site-packages && $WS_DIR/.venv/bin/pip install pyrealsense2 ultralytics opencv-python" >&2; exit 1; }
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"

echo "[neweyes] 启动视觉节点 pose_estimator ..."
echo "[neweyes] 服务名: get_pick_pos  帧: camera_link"
echo "[neweyes] 按 q 退出可视化窗口"
echo ""

ros2 run my_pick_pipeline pose_estimator
