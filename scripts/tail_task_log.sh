#!/bin/bash
# tail task_node 的实际运行日志（/tmp/task_node_debug.log）
# 用法：
#   ./scripts/tail_task_log.sh              # tail 全部输出
#   ./scripts/tail_task_log.sh box_edge_roll  # 只看 roll 相关
#   ./scripts/tail_task_log.sh grasp          # 只看抓取相关

FILTER="${1:-}"
LOG="/tmp/task_node_debug.log"

if [ ! -f "$LOG" ]; then
  echo "Log not found: $LOG (run_arm.sh 还没启动？)"
  exit 1
fi

echo "==> $LOG"
echo ""

if [ -n "$FILTER" ]; then
  tail -f "$LOG" | grep --line-buffered "$FILTER"
else
  tail -f "$LOG"
fi
