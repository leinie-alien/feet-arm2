#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

extern "C" {
#include "bsp_can.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
}

namespace {

constexpr int kDefaultMotorCount = 5;

struct StressStats {
  uint64_t sent = 0;
  uint64_t received = 0;
  std::vector<uint64_t> received_per_motor;
};

void pump_rx(StressStats* stats, int motor_count)
{
  while (canx_pending(&hcan1) > 0U) {
    uint16_t rec_id = 0;
    uint8_t rx_data[8] = {0};
    canx_receive(&hcan1, &rec_id, rx_data);

    if (rec_id >= 0x21 && rec_id < static_cast<uint16_t>(0x21 + motor_count)) {
      const uint8_t feedback_motor_id = static_cast<uint8_t>(rec_id - 0x20);
      motor_t* target_motor = &motor[feedback_motor_id - 1];
      dm_motor_fbdata(target_motor, rx_data);
      ++stats->received;
      ++stats->received_per_motor[feedback_motor_id - 1];
      continue;
    }

    if (rx_data[2] == 0x33) {
      const uint16_t motor_id =
          static_cast<uint16_t>(rx_data[0]) |
          static_cast<uint16_t>((rx_data[1] & 0x0F) << 4);
      if (motor_id >= 1 && motor_id <= static_cast<uint16_t>(motor_count)) {
        receive_motor_data(&motor[motor_id - 1], rx_data);
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv)
{
  const uint8_t channel = argc > 1 ? static_cast<uint8_t>(std::atoi(argv[1])) : 1;
  const int rounds = argc > 2 ? std::atoi(argv[2]) : 10000;
  const int interval_us = argc > 3 ? std::atoi(argv[3]) : 1000;
  const int requested_motor_count = argc > 4 ? std::atoi(argv[4]) : kDefaultMotorCount;
  const int motor_count = std::max(1, std::min(requested_motor_count, static_cast<int>(num)));

  if (!canx_open(&hcan1, channel, 1000000, 5000000)) {
    std::fprintf(stderr, "dm_motor_sdk_stress_test: canx_open failed\n");
    return 1;
  }

  dm_motor_init();
  for (int i = 0; i < motor_count; ++i) {
    dm_motor_enable(&hcan1, &motor[i]);
    motor[i].ctrl.pos_set = 0.0f;
    motor[i].ctrl.vel_set = 2.0f;
  }

  StressStats stats{};
  stats.received_per_motor.assign(static_cast<size_t>(motor_count), 0);
  const auto start = std::chrono::steady_clock::now();

  for (int round = 0; round < rounds; ++round) {
    pump_rx(&stats, motor_count);
    for (int i = 0; i < motor_count; ++i) {
      dm_motor_ctrl_send(&hcan1, &motor[i]);
      ++stats.sent;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(interval_us));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pump_rx(&stats, motor_count);
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  std::printf("channel=%u\n", channel);
  std::printf("rounds=%d\n", rounds);
  std::printf("interval_us=%d\n", interval_us);
  std::printf("motor_count=%d\n", motor_count);
  std::printf("sent_total=%llu\n", static_cast<unsigned long long>(stats.sent));
  std::printf("received_total=%llu\n", static_cast<unsigned long long>(stats.received));
  for (int i = 0; i < motor_count; ++i) {
    std::printf(
        "received_motor_%d=%llu\n",
        i + 1,
        static_cast<unsigned long long>(stats.received_per_motor[static_cast<size_t>(i)]));
  }
  std::printf("received_ratio=%.6f\n", stats.sent == 0 ? 0.0 : static_cast<double>(stats.received) / static_cast<double>(stats.sent));
  std::printf("elapsed_ms=%lld\n", static_cast<long long>(elapsed_ms));

  for (int i = 0; i < motor_count; ++i) {
    dm_motor_disable(&hcan1, &motor[i]);
  }
  canx_close(&hcan1);
  return 0;
}
