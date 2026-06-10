#!/usr/bin/env bash
# run_arm.sh — 机械臂控制栈一键启动
# 用法：
#   bash run_arm.sh               # 真机模式，task_node 在 xterm 中运行
#   bash run_arm.sh --sim         # 仿真模式（先另开终端跑 sim_arm.sh）
#   bash run_arm.sh --no-xterm    # task_node 输出到当前终端（SSH 场景）
#   bash run_arm.sh --build       # 启动前先编译
#   bash run_arm.sh -h            # 查看帮助
set -euo pipefail

# ===== 可改默认项（也可通过同名环境变量覆盖） =====
SIM_MODE="${SIM_MODE:-false}"           # true=仿真, false=真机
TASK_IN_XTERM="${TASK_IN_XTERM:-true}"  # true=task_node 弹 xterm 窗口
AUTO_BUILD="${AUTO_BUILD:-false}"        # true=启动前自动编译

# ===== 路径 =====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
PARAMS_FILE="${PARAMS_FILE:-$WS_DIR/src/arm2_task/config/params.yaml}"
DRIVER_PARAMS_FILE="${DRIVER_PARAMS_FILE:-$WS_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml}"
SIM_WS="${SIM_WS:-$HOME/data/robotics/arm_mujuco_ws}"
SUCTION_WS="${SUCTION_WS:-$WS_DIR}"
SUCTION_PORT="${SUCTION_PORT:-/dev/esp32_suction_c3}"

# udev（Damiao USB CAN 适配器）
DM_UDEV_RULE_SRC="$WS_DIR/src/dm_motor_sdk_ros/udev/99-dm-usb2canfd.rules"
DM_UDEV_RULE_DST="/etc/udev/rules.d/99-dm-usb2canfd.rules"
DM_USB_VENDOR_ID="34b7"

READY_TIMEOUT=15
DRIVER_PID=""
CONTROL_PID=""
SUCTION_PID=""
TASK_PID=""
CLEANUP_RUNNING=0
ROS2_DAEMON_PATTERN='[r]os2cli\.daemon\.daemonize.*--name ros2-daemon'
declare -a ROS2_DAEMON_PIDS_BEFORE=()

# ------------------------------------------------------------------ #
#  辅助函数
# ------------------------------------------------------------------ #

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

一键启动机械臂控制栈（真机 / 仿真两种模式）。

真机模式（默认）：
  1. 安装 Damiao USB udev 规则（首次）
  2. 启动 dm_motor_sdk_ros 驱动，等待 /robot_driver/ready
  3. 启动 control_node + task_node

仿真模式（--sim）：
  1. 提示确认仿真器已在另一个终端运行（sim_arm.sh）
  2. 等待 /robot_driver/ready（由 mujoco_runner 发出）
  3. 启动 control_node + task_node

Options:
  --sim                仿真模式（不启动硬件驱动）
  --sim-ws <dir>       指定仿真工作区目录（其中应包含 sim_arm.sh）
  --no-xterm           task_node 输出到当前终端，不弹 xterm（SSH 场景）
  --build              启动前先编译 dm_motor_sdk_ros 和 arm2_task
  --params <file>      指定 arm2_task params.yaml 路径
  --driver-params <f>  指定驱动 yaml 路径（真机模式有效）
  --ready-timeout <s>  等待驱动就绪的超时秒数（默认 $READY_TIMEOUT）
  -h, --help           显示此帮助

环境变量快捷覆盖（等同于对应选项）：
  SIM_MODE=true        等同于 --sim
  TASK_IN_XTERM=false  等同于 --no-xterm
  AUTO_BUILD=true      等同于 --build
  PARAMS_FILE=<path>   等同于 --params
  SIM_WS=<path>        等同于 --sim-ws
  SUCTION_WS=<path>    吸盘工作区（默认当前工作区）
  SUCTION_PORT=<path>  吸盘串口设备（默认 /dev/esp32_suction_c3）
USAGE
}

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

run_with_privilege() {
  if [[ "$(id -u)" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

# 在独立进程组中后台运行，将 pgid 写入变量
launch_in_group() {
  local __var="$1"; shift
  setsid "$@" &
  local pid=$!
  printf -v "$__var" '%s' "$pid"
}

stop_process_group() {
  local pid="$1" name="$2"
  [[ -n "$pid" ]] || return 0
  echo "[run_arm] stopping $name (pgid=$pid)..."
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 -- "-$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
    sleep 0.1
  done
  echo "[run_arm] WARN: $name did not stop after SIGTERM; sending SIGKILL"
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

record_existing_ros2_daemons() {
  mapfile -t ROS2_DAEMON_PIDS_BEFORE < <(pgrep -f "$ROS2_DAEMON_PATTERN" || true)
}

stop_new_ros2_daemons() {
  local -a current=()
  mapfile -t current < <(pgrep -f "$ROS2_DAEMON_PATTERN" || true)
  for pid in "${current[@]}"; do
    local seen=0
    for old in "${ROS2_DAEMON_PIDS_BEFORE[@]}"; do [[ "$pid" == "$old" ]] && { seen=1; break; }; done
    (( seen )) && continue
    echo "[run_arm] stopping ros2-daemon (pid=$pid)"
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break; sleep 0.1
    done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  done
}

cleanup() {
  (( CLEANUP_RUNNING )) && return
  CLEANUP_RUNNING=1
  echo "[run_arm] shutting down..."
  stop_process_group "$TASK_PID"    "task_node"
  stop_process_group "$SUCTION_PID" "suction_service_node"
  stop_process_group "$CONTROL_PID" "control_node"
  stop_process_group "$DRIVER_PID"  "dm_motor_sdk_ros driver"
  stop_new_ros2_daemons
  echo "[run_arm] done."
}

wait_for_ready() {
  local timeout_sec="$1"
  local deadline=$(( SECONDS + timeout_sec ))
  echo "[run_arm] waiting for /robot_driver/ready (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    # 真机模式：先确认驱动进程还活着，再检查 topic
    # （transient_local 会缓存上次会话的旧消息，必须先排除驱动已死的情况）
    if [[ -n "$DRIVER_PID" ]] && ! kill -0 "$DRIVER_PID" 2>/dev/null; then
      echo "[run_arm] ERROR: driver exited — no hardware connected?" >&2
      return 1
    fi
    if ros2 topic echo /robot_driver/ready std_msgs/msg/Bool \
        --once --qos-reliability reliable --qos-durability transient_local \
        2>/dev/null | grep -q "data: true"; then
      # 收到消息后再确认一次驱动还活着（防止收到 stale 缓存消息后才崩）
      if [[ -n "$DRIVER_PID" ]] && ! kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "[run_arm] ERROR: received stale /robot_driver/ready but driver has already exited." >&2
        echo "[run_arm]        (上次仿真/驱动遗留了 transient_local 缓存消息)" >&2
        return 1
      fi
      echo "[run_arm] /robot_driver/ready received."
      return 0
    fi
    sleep 0.3
  done
  echo "[run_arm] ERROR: timed out waiting for /robot_driver/ready" >&2
  return 1
}

ensure_dm_usb_permissions() {
  [[ -f "$DM_UDEV_RULE_SRC" ]] || { echo "[run_arm] WARN: udev rule source missing, skipping."; return 0; }
  command -v udevadm &>/dev/null || { echo "[run_arm] WARN: udevadm not found, skipping USB permission setup."; return 0; }
  if [[ ! -f "$DM_UDEV_RULE_DST" ]] || ! cmp -s "$DM_UDEV_RULE_SRC" "$DM_UDEV_RULE_DST"; then
    echo "[run_arm] installing Damiao USB udev rule..."
    run_with_privilege install -m 0644 "$DM_UDEV_RULE_SRC" "$DM_UDEV_RULE_DST"
    run_with_privilege udevadm control --reload-rules
    run_with_privilege udevadm trigger --subsystem-match=usb
  fi
  command -v lsusb &>/dev/null || return 0
  while IFS= read -r dev_node; do
    [[ -e "$dev_node" ]] || continue
    echo "[run_arm] granting access to $dev_node (Damiao USB CAN)"
    run_with_privilege chmod 0666 "$dev_node"
  done < <(lsusb -d "${DM_USB_VENDOR_ID}:" 2>/dev/null \
    | awk '{printf "/dev/bus/usb/%s/%s\n", $2, substr($4,1,3)}')
}

# ------------------------------------------------------------------ #
#  参数解析
# ------------------------------------------------------------------ #

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim)            SIM_MODE=true;           shift ;;
    --sim-ws)         SIM_WS="$2";             shift 2 ;;
    --no-xterm)       TASK_IN_XTERM=false;     shift ;;
    --build)          AUTO_BUILD=true;         shift ;;
    --params)         PARAMS_FILE="$2";        shift 2 ;;
    --driver-params)  DRIVER_PARAMS_FILE="$2"; shift 2 ;;
    --ready-timeout)  READY_TIMEOUT="$2";      shift 2 ;;
    -h|--help)        usage; exit 0 ;;
    *) echo "[run_arm] ERROR: unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

# ------------------------------------------------------------------ #
#  启动前校验
# ------------------------------------------------------------------ #

[[ -f "$ROS_SETUP" ]]    || { echo "[run_arm] ERROR: ROS setup not found: $ROS_SETUP" >&2; exit 1; }
[[ -f "$PARAMS_FILE" ]]  || { echo "[run_arm] ERROR: params not found: $PARAMS_FILE" >&2; exit 1; }
if [[ "$SIM_MODE" == "false" ]]; then
  [[ -f "$DRIVER_PARAMS_FILE" ]] \
    || { echo "[run_arm] ERROR: driver params not found: $DRIVER_PARAMS_FILE" >&2; exit 1; }
fi
if [[ "$TASK_IN_XTERM" == "true" ]] && ! command -v xterm &>/dev/null; then
  echo "[run_arm] WARN: xterm not found, falling back to inline output (--no-xterm)."
  TASK_IN_XTERM=false
fi

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║         run_arm.sh — 5-DOF 机械臂        ║"
echo "╚══════════════════════════════════════════╝"
echo "  workspace    : $WS_DIR"
echo "  params       : $PARAMS_FILE"
echo "  mode         : $([ "$SIM_MODE" == "true" ] && echo "仿真（MuJoCo）" || echo "真机（Damiao CAN）")"
echo "  task_in_xterm: $TASK_IN_XTERM"
echo ""

# ------------------------------------------------------------------ #
#  编译（可选）
# ------------------------------------------------------------------ #

source_setup "$ROS_SETUP"

if [[ "$AUTO_BUILD" == "true" ]]; then
  echo "[run_arm] building packages..."
  (cd "$WS_DIR" && colcon build \
    --packages-select robot_msgs suction_serial_bridge dm_motor_sdk_ros arm2_task \
    --cmake-args -DCMAKE_BUILD_TYPE=Release)
fi

[[ -f "$WS_SETUP" ]] \
  || { echo "[run_arm] ERROR: install/setup.bash not found — build first (--build or colcon build)" >&2; exit 1; }
source_setup "$WS_SETUP"
if [[ -f "$SUCTION_WS/install/setup.bash" ]]; then
  source_setup "$SUCTION_WS/install/setup.bash"
fi
record_existing_ros2_daemons
trap cleanup EXIT INT TERM
cd "$SCRIPT_DIR"

# ------------------------------------------------------------------ #
#  驱动层
# ------------------------------------------------------------------ #

if [[ "$SIM_MODE" == "true" ]]; then
  echo "[run_arm] 仿真模式：跳过硬件驱动。"
  if [[ -f "$SIM_WS/sim_arm.sh" ]]; then
    echo "[run_arm] 确保仿真器已在另一个终端运行："
    echo "            bash $SIM_WS/sim_arm.sh"
  else
    echo "[run_arm] WARN: 未找到仿真启动脚本: $SIM_WS/sim_arm.sh"
    echo "[run_arm]       当前 SIM_WS=$SIM_WS"
    echo "[run_arm]       如果你的仿真工作区在别处，可这样启动："
    echo "[run_arm]         SIM_WS=/actual/sim/workspace bash run_arm.sh --sim"
    echo "[run_arm]       或：bash run_arm.sh --sim --sim-ws /actual/sim/workspace"
  fi
  echo ""
  # 等 mujoco_runner 发布 ready
  if ! wait_for_ready "$READY_TIMEOUT"; then
    echo "[run_arm] 提示：请先启动仿真器，再运行本脚本。" >&2
    exit 1
  fi
else
  ensure_dm_usb_permissions
  echo "[run_arm] launching dm_motor_sdk_ros driver..."
  launch_in_group DRIVER_PID \
    ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py \
      params_path:="$DRIVER_PARAMS_FILE"
  if ! wait_for_ready "$READY_TIMEOUT"; then
    echo "[run_arm] ERROR: driver failed to become ready." >&2
    exit 1
  fi
fi

# ------------------------------------------------------------------ #
#  控制层：control_node
# ------------------------------------------------------------------ #

echo "[run_arm] launching control_node..."
launch_in_group CONTROL_PID \
  ros2 run arm2_task control_node \
    --ros-args --params-file "$PARAMS_FILE"

# control_node 需要订阅到 ready 信号后才开始工作，给它一秒初始化
sleep 1

# ------------------------------------------------------------------ #
#  吸盘层：suction_service_node（可选，仿真模式或设备不存在时跳过）
# ------------------------------------------------------------------ #

if [[ "$SIM_MODE" == "false" ]] && [[ -e "$SUCTION_PORT" ]]; then
  if [[ -f "$SUCTION_WS/install/setup.bash" ]]; then
    echo "[run_arm] launching suction_service_node on $SUCTION_PORT..."
    launch_in_group SUCTION_PID \
      ros2 launch suction_serial_bridge suction_service.launch.py
  else
    echo "[run_arm] WARN: suction workspace not built ($SUCTION_WS/install/setup.bash missing), skipping."
  fi
elif [[ "$SIM_MODE" == "false" ]]; then
  echo "[run_arm] WARN: suction device not found ($SUCTION_PORT), skipping suction node."
fi

# ------------------------------------------------------------------ #
#  任务层：task_node（可选 xterm）
# ------------------------------------------------------------------ #

TASK_LOG="/tmp/task_node_debug.log"
echo "[run_arm] launching task_node... (log: $TASK_LOG)"
if [[ "$TASK_IN_XTERM" == "true" ]]; then
  launch_in_group TASK_PID \
    xterm -hold -u8 -T "Arm — Task Control Panel" \
      -fn "-misc-fixed-medium-r-normal--18-120-100-100-c-90-iso10646-1" \
      -e bash -c "ros2 run arm2_task task_node \
           --ros-args --params-file '$PARAMS_FILE' 2>&1 | tee '$TASK_LOG'"
else
  launch_in_group TASK_PID \
    ros2 run arm2_task task_node \
      --ros-args --params-file "$PARAMS_FILE" 2>&1 | tee "$TASK_LOG"
fi

echo ""
echo "[run_arm] 所有节点已启动。按 Ctrl+C 全部停止。"
echo "  control_node pid: $CONTROL_PID"
echo "  task_node    pid: $TASK_PID"
[[ -n "$SUCTION_PID" ]] && echo "  suction_node  pid: $SUCTION_PID"
[[ -n "$DRIVER_PID" ]] && echo "  driver       pid: $DRIVER_PID"
echo ""

# 任意一个核心进程（control/task）退出就结束整个栈。
# 驱动进程不放进 wait -n，改用后台 watchdog 监视，
# 避免驱动意外崩溃后 wait -n 立刻返回却没有给用户任何提示。
if [[ -n "$DRIVER_PID" ]]; then
  ( while kill -0 "$DRIVER_PID" 2>/dev/null; do sleep 1; done
    echo ""
    echo "[run_arm] WARN: driver process exited unexpectedly — stopping arm stack."
    kill -TERM $$ 2>/dev/null || true
  ) &
fi
wait -n "$CONTROL_PID" "$TASK_PID"
