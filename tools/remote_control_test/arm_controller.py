#!/usr/bin/env python3
"""
arm_controller.py — 机械臂远程控制测试节点

用法：
  # 1. 先把 params.yaml 里 task.remote_mode 改为 true 再启动机械臂
  # 2. source ROS2 环境后运行本脚本
  #    python3 arm_controller.py
  #    python3 arm_controller.py --once grasp   # 发一条命令后退出
  #    python3 arm_controller.py --once place

话题：
  /arm/cmd    (std_msgs/String) 发送命令  → "grasp" / "place"
  /arm/status (std_msgs/String) 接收状态  ← "grasped" / "stowed" / "placed" / "reset" / "error:xxx"
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 终端颜色
_GREEN  = "\033[92m"
_YELLOW = "\033[93m"
_RED    = "\033[91m"
_CYAN   = "\033[96m"
_RESET  = "\033[0m"


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller_test")

        self._pub = self.create_publisher(String, "/arm/cmd", 10)
        self._sub = self.create_subscription(
            String, "/arm/status", self._on_status, 10
        )
        self._last_status = None
        self._status_event = threading.Event()

    # ── 接收状态回调 ──────────────────────────────────────────────────────────
    def _on_status(self, msg: String):
        s = msg.data
        self._last_status = s
        self._status_event.set()

        if s.startswith("error"):
            color = _RED
        elif s in ("grasped", "placed"):
            color = _GREEN
        elif s in ("stowed", "reset"):
            color = _CYAN
        else:
            color = _YELLOW

        print(f"  {color}[ARM STATUS] {s}{_RESET}")

    # ── 发送命令 ─────────────────────────────────────────────────────────────
    def send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        print(f"{_YELLOW}[SEND] → {cmd}{_RESET}")

    # ── 等待特定状态（超时返回 False）────────────────────────────────────────
    def wait_for_status(self, expected: str, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._status_event.clear()
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            self._status_event.wait(timeout=min(0.5, remaining))
            if self._last_status == expected:
                return True
            if self._last_status and self._last_status.startswith("error"):
                print(f"{_RED}[ERROR] Arm reported: {self._last_status}{_RESET}")
                return False
        print(f"{_RED}[TIMEOUT] Expected '{expected}' but got '{self._last_status}'{_RESET}")
        return False

    # ── 完整抓取流程（等到 stowed）────────────────────────────────────────────
    def run_grasp(self) -> bool:
        print(f"\n{_CYAN}=== Grasp sequence ==={_RESET}")
        self._last_status = None
        self.send_cmd("grasp")

        print("  Waiting for grasped...")
        if not self.wait_for_status("grasped", timeout=40.0):
            return False

        print("  Waiting for stowed...")
        if not self.wait_for_status("stowed", timeout=30.0):
            return False

        print(f"{_GREEN}=== Grasp complete ==={_RESET}")
        return True

    # ── 完整放置流程（等到 reset）─────────────────────────────────────────────
    def run_place(self) -> bool:
        print(f"\n{_CYAN}=== Place sequence ==={_RESET}")
        self._last_status = None
        self.send_cmd("place")

        print("  Waiting for placed...")
        if not self.wait_for_status("placed", timeout=40.0):
            return False

        print("  Waiting for reset...")
        if not self.wait_for_status("reset", timeout=30.0):
            return False

        print(f"{_GREEN}=== Place complete ==={_RESET}")
        return True

    # ── 交互菜单 ─────────────────────────────────────────────────────────────
    def run_interactive(self):
        print(f"\n{_CYAN}Arm Remote Controller — interactive mode{_RESET}")
        print("  g / grasp  → execute grasp sequence")
        print("  p / place  → execute place sequence")
        print("  s          → send raw status query (just sends 'status')")
        print("  q / quit   → exit\n")

        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        # 等机械臂启动完成的 "reset" 信号（最多 60s）
        print("Waiting for arm to be ready (reset signal)...")
        if not self.wait_for_status("reset", timeout=60.0):
            print(f"{_YELLOW}No reset signal received, proceeding anyway.{_RESET}")

        while rclpy.ok():
            try:
                cmd = input(f"\n{_CYAN}cmd> {_RESET}").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("g", "grasp"):
                self.run_grasp()
            elif cmd in ("p", "place"):
                self.run_place()
            elif cmd == "s":
                self.send_cmd("status")
            elif cmd == "":
                continue
            else:
                print(f"Unknown command: {cmd}")


def main():
    parser = argparse.ArgumentParser(description="Arm remote control test node")
    parser.add_argument(
        "--once",
        choices=["grasp", "place"],
        help="Send a single command and exit when sequence completes",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ArmController()

    if args.once:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        if args.once == "grasp":
            ok = node.run_grasp()
        else:
            ok = node.run_place()

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if ok else 1)
    else:
        node.run_interactive()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
