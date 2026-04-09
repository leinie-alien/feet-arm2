#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

extern "C" {
#include "bsp_can.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
}

namespace {

// 从 SDK 适配层的接收队列中取出所有待处理帧，
// 再交给现有协议层回调做解析。
void pump_rx()
{
  while (canx_pending(&hcan1) > 0U) {
    can1_rx_callback();
  }
}

// 打印每个电机当前最新一次解析到的反馈快照。
void print_feedback(int motor_count)
{
  for (int i = 0; i < motor_count; ++i) {
    std::printf(
        "motor=%u pos=%.4f vel=%.4f tor=%.4f state=%d\n",
        motor[i].id,
        motor[i].para.pos,
        motor[i].para.vel,
        motor[i].para.tor,
        motor[i].para.state);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  // 命令行参数：
  //   argv[1] -> 通道号
  //   argv[2] -> 运行时长（秒）
  //   argv[3] -> 控制循环频率（Hz）
  const uint8_t channel = argc > 1 ? static_cast<uint8_t>(std::atoi(argv[1])) : 1;
  const int duration_sec = argc > 2 ? std::atoi(argv[2]) : 5;
  const double loop_hz = argc > 3 ? std::atof(argv[3]) : 200.0;
  const int motor_count = 3;

  // 打开指定通道上的 USB2CANFD SDK 通信链路。
  if (!canx_open(&hcan1, channel, 1000000, 5000000)) {
    std::fprintf(stderr, "dm_motor_sdk_example: canx_open failed\n");
    return 1;
  }

  // 初始化协议层电机状态，并使能前 3 个电机。
  dm_motor_init();
  for (int i = 0; i < motor_count; ++i) {
    dm_motor_enable(&hcan1, &motor[i]);
  }

  const auto period = std::chrono::duration<double>(1.0 / std::max(loop_hz, 1.0));
  const auto start = std::chrono::steady_clock::now();
  auto last_print = start;

  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(duration_sec)) {
    // 发送下一批控制命令前，先把待处理反馈全部取出。
    pump_rx();

    // 给每个电机一个小幅正弦位置目标，
    // 让这个 example 同时覆盖发送链路和反馈链路。
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    for (int i = 0; i < motor_count; ++i) {
      motor[i].ctrl.pos_set = static_cast<float>(0.5 * std::sin(t + i * 0.3));
      motor[i].ctrl.vel_set = 2.0f;
      dm_motor_ctrl_send(&hcan1, &motor[i]);
    }

    // 以适合人工阅读的频率打印反馈，而不是每个控制周期都打印。
    const auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::milliseconds(500)) {
      print_feedback(motor_count);
      last_print = now;
    }

    std::this_thread::sleep_for(period);
  }

  // 退出前再处理一遍残留反馈，并输出最终一次状态快照。
  pump_rx();
  print_feedback(motor_count);

  // 让电机以干净状态退出，并关闭 SDK 支撑的 CAN 通道。
  for (int i = 0; i < motor_count; ++i) {
    dm_motor_disable(&hcan1, &motor[i]);
  }
  canx_close(&hcan1);
  return 0;
}
