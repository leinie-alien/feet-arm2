#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "robot_msgs/msg/motor_command.hpp"
#include "robot_msgs/msg/motor_state.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

extern "C" {
#include "bsp_can.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
}

namespace
{

constexpr size_t kDefaultMotorCount = 5;
constexpr double kFullTurnRad = 2.0 * M_PI;
constexpr double kJoint1LowerLimit = -4.0 * M_PI / 3.0;
constexpr double kJoint1UpperLimit = 4.0 * M_PI / 3.0;
constexpr double kJoint2LowerLimit = 0.0;
constexpr double kJoint2UpperLimit = 3.5;
constexpr double kJoint2WrappedLowerLimit = kJoint2LowerLimit - kFullTurnRad;
constexpr double kJoint2WrappedUpperLimit = kJoint2UpperLimit - kFullTurnRad;
constexpr double kJoint3LowerLimit = -3.0;
constexpr double kJoint3UpperLimit = 0.15;
constexpr double kJoint4LowerLimit = -2.0;
constexpr double kJoint4UpperLimit = 1.57;
constexpr double kJoint5LowerLimit = -M_PI;
constexpr double kJoint5UpperLimit = M_PI;
constexpr double kDefaultQJumpThresholdRad = 0.03;
constexpr double kDefaultTorqueLogPeriodSec = 1.0;
constexpr int kEnableRetryLimit = 10;
constexpr int kEnableRetryIntervalMs = 50;

std::string join_double_vector(const std::vector<double> & values)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

double clamp_to_interval(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace

class DmMotorRobotDriverNode : public rclcpp::Node 
{
public:
  DmMotorRobotDriverNode()
  : Node("dm_motor_driver_node")
  {
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    ready_publisher_ = create_publisher<std_msgs::msg::Bool>("/robot_driver/ready", ready_qos);
    state_publisher_ = create_publisher<robot_msgs::msg::RobotState>("/arm2/_lowState/joint", state_qos);
    publish_ready_state(false);

    channel_ = declare_parameter<int>("channel", 1);
    can_baud_ = declare_parameter<int>("can_baud", 1000000);
    canfd_baud_ = declare_parameter<int>("canfd_baud", 5000000);
    loop_hz_ = declare_parameter<double>("loop_hz", 1000.0);

    auto motor_ids = declare_parameter<std::vector<int64_t>>(
      "motor_ids", std::vector<int64_t>{1, 2, 3, 4, 5});
    auto feedback_ids = declare_parameter<std::vector<int64_t>>(
      "feedback_ids", std::vector<int64_t>{});
    auto inverted_ids = declare_parameter<std::vector<int64_t>>(
      "inverted_motor_ids", std::vector<int64_t>{});
    auto feedback_timeout_ms = declare_parameter<int64_t>("feedback_timeout_ms", 50);
    auto command_timeout_ms = declare_parameter<int64_t>("command_timeout_ms", 2000);
    auto motor_pmax = declare_parameter<std::vector<double>>(
      "motor_pmax", std::vector<double>{12.5, 12.5, 12.5, 12.5, 12.566});
    auto motor_vmax = declare_parameter<std::vector<double>>(
      "motor_vmax", std::vector<double>{30.0, 10.0, 10.0, 30.0, 50.0});
    auto motor_tmax = declare_parameter<std::vector<double>>(
      "motor_tmax", std::vector<double>{10.0, 28.0, 28.0, 10.0, 5.0});
    auto joint_zero_offsets = declare_parameter<std::vector<double>>(
      "joint_zero_offsets", std::vector<double>{-0.02, 0.14, 0.1, 0.361, 0.0});
    q_jump_threshold_rad_ = declare_parameter<double>("q_jump_threshold_rad", kDefaultQJumpThresholdRad);
    torque_log_period_sec_ =
      declare_parameter<double>("torque_log_period_sec", kDefaultTorqueLogPeriodSec);
    temperature_topic_ =
      declare_parameter<std::string>("temperature_topic", "/arm2/_lowState/temperature");
    temperature_publish_hz_ = declare_parameter<double>("temperature_publish_hz", 1.0);

    normalize_motor_ids(motor_ids);
    normalize_feedback_ids(feedback_ids);

    if (loop_hz_ < 1.0) {
      RCLCPP_WARN(get_logger(), "参数 loop_hz=%.3f 过小，已提升到 1.0Hz。", loop_hz_);
      loop_hz_ = 1.0;
    }
    if (feedback_timeout_ms < 10) {
      RCLCPP_WARN(get_logger(), "参数 feedback_timeout_ms=%ld 过小，已提升到 10ms。", feedback_timeout_ms);
      feedback_timeout_ms = 10;
    }
    if (command_timeout_ms < 10) {
      RCLCPP_WARN(get_logger(), "参数 command_timeout_ms=%ld 过小，已提升到 10ms。", command_timeout_ms);
      command_timeout_ms = 10;
    }
    if (temperature_publish_hz_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "参数 temperature_publish_hz=%.3f 非法，已关闭温度话题发布。",
        temperature_publish_hz_);
      temperature_publish_hz_ = 0.0;
    }

    normalize_vector_param(motor_pmax, "motor_pmax", 12.5);
    normalize_vector_param(motor_vmax, "motor_vmax", 30.0);
    normalize_vector_param(motor_tmax, "motor_tmax", 10.0);
    normalize_vector_param(joint_zero_offsets, "joint_zero_offsets", 0.0);

    feedback_timeout_ns_ = static_cast<uint64_t>(feedback_timeout_ms) * 1000000ull;
    command_timeout_ns_ = static_cast<uint64_t>(command_timeout_ms) * 1000000ull;
    joint_zero_offsets_ = joint_zero_offsets;
    if (temperature_publish_hz_ > 0.0) {
      temperature_publish_period_ns_ = static_cast<uint64_t>(1000000000.0 / temperature_publish_hz_);
      if (temperature_publish_period_ns_ == 0) {
        temperature_publish_period_ns_ = 1;
      }
    }

    for (const auto raw_id : inverted_ids) {
      if (raw_id < 1 || raw_id > static_cast<int64_t>(num)) {
        RCLCPP_WARN(
          get_logger(), "忽略越界的反向电机 ID: %ld，允许范围为 [1, %d]", raw_id, num);
        continue;
      }
      inverted_motor_ids_.insert(static_cast<uint16_t>(raw_id));
    }

    if (!canx_open(&hcan1, static_cast<uint8_t>(channel_), can_baud_, canfd_baud_)) {
      throw std::runtime_error(
              "canx_open failed. Check USB2CANFD_Dual connection, permissions, and channel settings.");
    }
    interface_ready_ = true;

    dm_motor_init();
    cached_commands_.resize(hardware_ids_.size());
    q_jump_reject_counts_.assign(hardware_ids_.size(), 0);
    feedback_position_biases_.assign(hardware_ids_.size(), 0.0);
    last_torque_log_time_ = std::chrono::steady_clock::now();

    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      robot_msgs::msg::MotorState init_state;
      init_state.q = 0.0f;
      init_state.dq = 0.0f;
      init_state.ddq = 0.0f;
      init_state.tau_est = 0.0f;
      init_state.cur = 0.0f;
      init_state.valid = false;
      last_valid_states_[hardware_ids_[i]] = init_state;

      motor[i].id = hardware_ids_[i];
      motor[i].ctrl.mode = mit_mode;
      motor[i].tmp.PMAX = static_cast<float>(motor_pmax[i]);
      motor[i].tmp.VMAX = static_cast<float>(motor_vmax[i]);
      motor[i].tmp.TMAX = static_cast<float>(motor_tmax[i]);
      if (!feedback_ids_.empty()) {
        motor[i].mst_id = feedback_ids_[i];
      }
      cached_commands_[i].q = 0.0f;
      cached_commands_[i].dq = 0.0f;
      cached_commands_[i].tau = 0.0f;
      cached_commands_[i].kp = 0.0f;
      cached_commands_[i].kd = 0.0f;
    }
    enable_motors_and_wait_for_feedback();
    initial_enable_completed_ = true;
    publish_ready_state(true);

    subscription_ = create_subscription<robot_msgs::msg::RobotCommand>(
      "/arm2/_lowCmd/command", state_qos,
      std::bind(&DmMotorRobotDriverNode::command_callback, this, std::placeholders::_1));
    if (!temperature_topic_.empty() && temperature_publish_hz_ > 0.0) {
      temperature_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        temperature_topic_, state_qos);
    }

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DmMotorRobotDriverNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "dm_motor_robot_driver_node started. channel=%d can=%d canfd=%d loop_hz=%.1f motors=%zu",
      channel_, can_baud_, canfd_baud_, loop_hz_, hardware_ids_.size());
    if (feedback_ids_.empty()) {
      RCLCPP_INFO(get_logger(), "feedback_ids 未显式配置，沿用底层 dm_motor_init() 的 mst_id 配置");
    } else {
      RCLCPP_INFO(get_logger(), "feedback_ids 已显式配置，将覆盖底层 mst_id");
    }
    RCLCPP_INFO(
      get_logger(),
      "1号电机 ROS 限位[rad]: [%.3f, %.3f]",
      kJoint1LowerLimit,
      kJoint1UpperLimit);
    RCLCPP_INFO(
      get_logger(), "关节零点偏置[rad]: [%s]", join_double_vector(joint_zero_offsets_).c_str());
    RCLCPP_INFO(
      get_logger(),
      "2号电机 ROS 限位[rad]: [%.3f, %.3f], 启动判圈窗口[rad]: [%.3f, %.3f]",
      kJoint2LowerLimit,
      kJoint2UpperLimit,
      kJoint2WrappedLowerLimit,
      kJoint2WrappedUpperLimit);
    if (temperature_publisher_) {
      RCLCPP_INFO(
        get_logger(), "温度话题已启用: topic=%s rate=%.3fHz layout=[motor, sensor(Tmos,Tcoil)]",
        temperature_topic_.c_str(), temperature_publish_hz_);
    } else {
      RCLCPP_INFO(get_logger(), "温度话题发布已关闭。");
    }
  }

  ~DmMotorRobotDriverNode() override
  {
    publish_ready_state(false);
    if (!interface_ready_) {
      return;
    }

    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      dm_motor_disable(&hcan1, &motor[i]);
    }
    canx_close(&hcan1);
  }

private:
  void normalize_motor_ids(const std::vector<int64_t> & motor_ids)
  {
    std::unordered_set<uint16_t> seen_ids;

    hardware_ids_.clear();
    for (const auto raw_id : motor_ids) {
      if (raw_id < 1 || raw_id > static_cast<int64_t>(num)) {
        RCLCPP_WARN(get_logger(), "忽略越界电机 ID: %ld，允许范围为 [1, %d]", raw_id, num);
        continue;
      }

      const auto id = static_cast<uint16_t>(raw_id);
      if (!seen_ids.insert(id).second) {
        RCLCPP_WARN(get_logger(), "忽略重复电机 ID: %u", static_cast<unsigned int>(id));
        continue;
      }
      hardware_ids_.push_back(id);
    }

    if (hardware_ids_.empty()) {
      RCLCPP_WARN(get_logger(), "未配置有效 motor_ids，回退到默认 5 轴映射。");
      for (size_t i = 0; i < kDefaultMotorCount; ++i) {
        hardware_ids_.push_back(static_cast<uint16_t>(i + 1));
      }
      return;
    }

    if (hardware_ids_.size() > static_cast<size_t>(num)) {
      RCLCPP_WARN(
        get_logger(), "motor_ids 数量为 %zu，超过底层支持上限 %d，已截断。",
        hardware_ids_.size(), num);
      hardware_ids_.resize(num);
    }
  }

  void normalize_vector_param(std::vector<double> & values, const char * name, double fallback)
  {
    if (values.size() == hardware_ids_.size()) {
      return;
    }

    RCLCPP_WARN(
      get_logger(), "参数 %s 长度为 %zu，期望为 %zu。将使用默认值 %.3f。",
      name, values.size(), hardware_ids_.size(), fallback);
    values.assign(hardware_ids_.size(), fallback);
  }

  void normalize_feedback_ids(const std::vector<int64_t> & feedback_ids)
  {
    feedback_ids_.clear();
    if (feedback_ids.empty()) {
      return;
    }

    if (feedback_ids.size() != hardware_ids_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "参数 feedback_ids 长度为 %zu，期望为 %zu。将忽略该参数并沿用底层 mst_id 配置。",
        feedback_ids.size(), hardware_ids_.size());
      return;
    }

    for (const auto raw_id : feedback_ids) {
      if (raw_id < 1 || raw_id > 0x7FF) {
        RCLCPP_WARN(
          get_logger(),
          "参数 feedback_ids 中存在越界值 %ld。将忽略整个参数并沿用底层 mst_id 配置。",
          raw_id);
        feedback_ids_.clear();
        return;
      }
      feedback_ids_.push_back(static_cast<uint16_t>(raw_id));
    }
  }

  double clamp_ros_joint_angle(uint16_t motor_id, double q) const
  {
    if (motor_id == 1) {
      return clamp_to_interval(q, kJoint1LowerLimit, kJoint1UpperLimit);
    }
    if (motor_id == 2) {
      return clamp_to_interval(q, kJoint2LowerLimit, kJoint2UpperLimit);
    }
    if (motor_id == 3) {
      return clamp_to_interval(q, kJoint3LowerLimit, kJoint3UpperLimit);
    }
    if (motor_id == 4) {
      return clamp_to_interval(q, kJoint4LowerLimit, kJoint4UpperLimit);
    }
    if (motor_id == 5) {
      return clamp_to_interval(q, kJoint5LowerLimit, kJoint5UpperLimit);
    }
    return q;
  }

  void initialize_joint2_turn_bias()
  {
    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      if (hardware_ids_[i] != 2) {
        continue;
      }

      motor_fbpara_t feedback{};
      uint64_t feedback_ns = 0;
      const bool has_feedback =
        dm_motor_get_feedback_snapshot(hardware_ids_[i], &feedback, &feedback_ns) && feedback_ns != 0;
      if (!has_feedback || !std::isfinite(feedback.pos)) {
        RCLCPP_WARN(
          get_logger(),
          "2号电机启动时未拿到有效位置反馈，启动判圈回退为 0 圈偏置。");
        feedback_position_biases_[i] = 0.0;
        return;
      }

      const double direction = should_invert(hardware_ids_[i]) ? -1.0 : 1.0;
      const double q_base = direction * static_cast<double>(feedback.pos) - joint_zero_offsets_[i];
      if (q_base >= kJoint2LowerLimit && q_base <= kJoint2UpperLimit) {
        feedback_position_biases_[i] = 0.0;
      } else if (q_base >= kJoint2WrappedLowerLimit && q_base <= kJoint2WrappedUpperLimit) {
        feedback_position_biases_[i] = kFullTurnRad;
      } else {
        feedback_position_biases_[i] = 0.0;
        RCLCPP_WARN(
          get_logger(),
          "2号电机启动位置 %.4f rad 不在期望区间 [%.4f, %.4f] 或 [%.4f, %.4f] 内，回退为 0 圈偏置。",
          q_base,
          kJoint2LowerLimit,
          kJoint2UpperLimit,
          kJoint2WrappedLowerLimit,
          kJoint2WrappedUpperLimit);
      }

      RCLCPP_INFO(
        get_logger(),
        "2号电机启动判圈: q_base=%.4f rad, bias=%.4f rad, q_ros=%.4f rad",
        q_base,
        feedback_position_biases_[i],
        clamp_ros_joint_angle(hardware_ids_[i], q_base + feedback_position_biases_[i]));
      return;
    }
  }

  double command_q_to_motor_pos_set(size_t index, double command_q)
  {
    const uint16_t motor_id = hardware_ids_[index];
    const double direction = should_invert(motor_id) ? -1.0 : 1.0;
    const double limited_q = clamp_ros_joint_angle(motor_id, command_q);
    return direction * (limited_q - feedback_position_biases_[index] + joint_zero_offsets_[index]);
  }

  bool should_invert(uint16_t motor_id) const
  {
    return inverted_motor_ids_.count(motor_id) > 0;
  }

  uint64_t monotonic_now_ns() const
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void publish_ready_state(bool ready)
  {
    if (ready_state_initialized_ && ready == last_published_ready_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_publisher_->publish(msg);
    ready_state_initialized_ = true;
    last_published_ready_ = ready;
  }

  void apply_cached_command_locked(size_t index)
  {
    const uint16_t motor_id = hardware_ids_[index];
    const double direction = should_invert(motor_id) ? -1.0 : 1.0;

    motor[index].ctrl.pos_set = static_cast<float>(command_q_to_motor_pos_set(index, cached_commands_[index].q));
    motor[index].ctrl.vel_set = static_cast<float>(direction * cached_commands_[index].dq);
    motor[index].ctrl.tor_set = static_cast<float>(direction * cached_commands_[index].tau);
    motor[index].ctrl.kp_set = cached_commands_[index].kp;
    motor[index].ctrl.kd_set = cached_commands_[index].kd;
  }

  void apply_feedback_probe_command_locked(size_t index)
  {
    motor[index].ctrl.pos_set = 0.0f;
    motor[index].ctrl.vel_set = 0.0f;
    motor[index].ctrl.tor_set = 0.0f;
    motor[index].ctrl.kp_set = 0.0f;
    motor[index].ctrl.kd_set = 0.0f;
  }

  void pump_feedback()
  {
    while (canx_pending(&hcan1) > 0U) {
      can1_rx_callback();
    }
  }

  void enable_motors_and_wait_for_feedback()
  {
    std::vector<bool> feedback_received(hardware_ids_.size(), false);

    for (int attempt = 1; attempt <= kEnableRetryLimit; ++attempt) {
      bool all_feedback_received = true;

      for (size_t i = 0; i < hardware_ids_.size(); ++i) {
        if (feedback_received[i]) {
          continue;
        }
        dm_motor_enable(&hcan1, &motor[i]);
      }

      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kEnableRetryIntervalMs);
      while (std::chrono::steady_clock::now() < deadline) {
        pump_feedback();
        for (size_t i = 0; i < hardware_ids_.size(); ++i) {
          if (feedback_received[i]) {
            continue;
          }

          motor_fbpara_t feedback{};
          uint64_t feedback_ns = 0;
          if (dm_motor_get_feedback_snapshot(hardware_ids_[i], &feedback, &feedback_ns) && feedback_ns != 0) {
            feedback_received[i] = true;
          }
        }

        all_feedback_received = std::all_of(
          feedback_received.begin(), feedback_received.end(),
          [](bool received) { return received; });
        if (all_feedback_received) {
          break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }

      all_feedback_received = std::all_of(
        feedback_received.begin(), feedback_received.end(),
        [](bool received) { return received; });
      if (all_feedback_received) {
        if (attempt > 1) {
          RCLCPP_WARN(
            get_logger(),
            "电机使能在第 %d 轮补发后全部收到首帧反馈。",
            attempt);
        }
        initialize_joint2_turn_bias();
        return;
      }

      std::ostringstream pending_ids;
      bool first = true;
      for (size_t i = 0; i < hardware_ids_.size(); ++i) {
        if (feedback_received[i]) {
          continue;
        }
        if (!first) {
          pending_ids << ", ";
        }
        first = false;
        pending_ids << hardware_ids_[i];
      }

      RCLCPP_WARN(
        get_logger(),
        "第 %d/%d 轮使能后仍未收到这些电机的首帧反馈: [%s]，将继续补发使能。",
        attempt,
        kEnableRetryLimit,
        pending_ids.str().c_str());
    }

    std::ostringstream missing_ids;
    bool first = true;
    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      motor_fbpara_t feedback{};
      uint64_t feedback_ns = 0;
      const bool has_feedback =
        dm_motor_get_feedback_snapshot(hardware_ids_[i], &feedback, &feedback_ns) && feedback_ns != 0;
      if (has_feedback) {
        continue;
      }

      if (!first) {
        missing_ids << ", ";
      }
      first = false;
      missing_ids << hardware_ids_[i];
    }

    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      if (feedback_received[i]) {
        dm_motor_disable(&hcan1, &motor[i]);
      }
    }

    throw std::runtime_error(
      "Failed to receive initial motor feedback after enable retries. Missing motor IDs: [" +
      missing_ids.str() + "]");
  }

  void command_callback(const robot_msgs::msg::RobotCommand::SharedPtr msg)
  {
    if (!msg || !interface_ready_) {
      return;
    }

    const size_t cmd_size = msg->motor_command.size();
    if (cmd_size != hardware_ids_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到控制数组长度为 %zu，期望长度为 %zu。整包拒绝，继续保持上一帧有效控制命令。",
        cmd_size, hardware_ids_.size());
      return;
    }

    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      const auto & cmd = msg->motor_command[i];
      if (
        !std::isfinite(cmd.q) || !std::isfinite(cmd.dq) || !std::isfinite(cmd.tau) ||
        !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd))
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "拦截: 收到电机 [ID:%u] 的非法控制指令(NaN/Inf)，继续保持上一帧有效控制命令。",
          static_cast<unsigned int>(hardware_ids_[i]));
        return;
      }
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    cached_commands_ = msg->motor_command;
    last_command_ns_ = monotonic_now_ns();
    has_valid_command_ = true;
  }

  void timer_callback()
  {
    if (!interface_ready_) {
      publish_ready_state(false);
      return;
    }

    pump_feedback();

    const uint64_t now_ns = monotonic_now_ns();
    bool has_valid_command = false;
    bool command_timed_out;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      has_valid_command = has_valid_command_;
      command_timed_out =
        has_valid_command_ && (last_command_ns_ != 0) && ((now_ns - last_command_ns_) > command_timeout_ns_);
      if (!has_valid_command) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "尚未收到首条有效控制指令，驱动持续发送零增益控制帧以拉起电机反馈。");
      }
      if (command_timed_out) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "控制命令超时，继续保持上一帧有效控制命令，不切换为零力矩输出。");
      }

      for (size_t i = 0; i < hardware_ids_.size(); ++i) {
        if (has_valid_command) {
          apply_cached_command_locked(i);
        } else {
          apply_feedback_probe_command_locked(i);
        }
        dm_motor_ctrl_send(&hcan1, &motor[i]);
      }
    }

    robot_msgs::msg::RobotState state_msg;
    state_msg.motor_state.resize(hardware_ids_.size());
    bool all_feedback_fresh = true;

    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      const uint16_t motor_id = hardware_ids_[i];
      motor_fbpara_t feedback{};
      uint64_t feedback_ns = 0;
      const bool has_feedback = dm_motor_get_feedback_snapshot(motor_id, &feedback, &feedback_ns);
      const bool fresh_feedback =
        has_feedback && feedback_ns <= now_ns && (now_ns - feedback_ns) <= feedback_timeout_ns_;
      const auto last_valid_it = last_valid_states_.find(motor_id);
      const bool had_valid_feedback =
        last_valid_it != last_valid_states_.end() && last_valid_it->second.valid;

      if (!fresh_feedback) {
        all_feedback_fresh = false;
        if (had_valid_feedback) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "电机 [ID:%u] 反馈超时，继续发布上一帧有效状态。",
            static_cast<unsigned int>(motor_id));
        } else {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "电机 [ID:%u] 尚未收到首帧反馈，继续发布上一帧有效状态。",
            static_cast<unsigned int>(motor_id));
        }
        state_msg.motor_state[i] = last_valid_states_[motor_id];
        state_msg.motor_state[i].valid = false;
        continue;
      }

      double q = feedback.pos;
      double dq = feedback.vel;
      double tau = feedback.tor;
      const double direction = should_invert(motor_id) ? -1.0 : 1.0;

      if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau)) {
        all_feedback_fresh = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "电机 [ID:%u] 反馈异常(NaN/Inf)，回退到上一帧有效状态。",
          static_cast<unsigned int>(motor_id));
        state_msg.motor_state[i] = last_valid_states_[motor_id];
        state_msg.motor_state[i].valid = false;
        continue;
      }

      q = direction * q - joint_zero_offsets_[i];
      q += feedback_position_biases_[i];
      q = clamp_ros_joint_angle(motor_id, q);
      dq = direction * dq;
      tau = direction * tau;

      if (had_valid_feedback) {
        const double q_delta = std::abs(static_cast<double>(last_valid_it->second.q) - q);
        if (q_delta > q_jump_threshold_rad_) {
          all_feedback_fresh = false;
          ++q_jump_reject_counts_[i];
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "电机 [ID:%u] 位置反馈跳变过大(Δq=%.4f rad, 阈值=%.4f rad)，回退到上一帧有效状态。",
            static_cast<unsigned int>(motor_id), q_delta, q_jump_threshold_rad_);
          state_msg.motor_state[i] = last_valid_it->second;
          state_msg.motor_state[i].valid = false;
          continue;
        }
      }

      state_msg.motor_state[i].q = static_cast<float>(q);
      state_msg.motor_state[i].dq = static_cast<float>(dq);
      state_msg.motor_state[i].ddq = 0.0f;
      state_msg.motor_state[i].tau_est = static_cast<float>(tau);
      state_msg.motor_state[i].cur = 0.0f;
      state_msg.motor_state[i].valid = true;
      last_valid_states_[motor_id] = state_msg.motor_state[i];
    }

    feedback_healthy_ = all_feedback_fresh;
    publish_ready_state(interface_ready_ && initial_enable_completed_);
    state_publisher_->publish(state_msg);
    publish_temperature_if_needed(now_ns);
    log_torque_diagnostics_if_needed(state_msg);
  }

  void log_torque_diagnostics_if_needed(const robot_msgs::msg::RobotState & state_msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - last_torque_log_time_).count();
    if (elapsed_sec < torque_log_period_sec_) {
      return;
    }

    std::vector<double> received_ff_tau;
    received_ff_tau.reserve(cached_commands_.size());
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      for (const auto & cmd : cached_commands_) {
        received_ff_tau.push_back(static_cast<double>(cmd.tau));
      }
    }

    std::vector<double> feedback_tau_est;
    feedback_tau_est.reserve(state_msg.motor_state.size());
    std::string valid_mask;
    valid_mask.reserve(state_msg.motor_state.size() * 2);
    for (size_t i = 0; i < state_msg.motor_state.size(); ++i) {
      feedback_tau_est.push_back(static_cast<double>(state_msg.motor_state[i].tau_est));
      valid_mask += state_msg.motor_state[i].valid ? '1' : '0';
      if (i + 1 != state_msg.motor_state.size()) {
        valid_mask += ',';
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Torque diag: ff_tau=%s fb_tau=%s valid_mask=[%s]",
      join_double_vector(received_ff_tau).c_str(),
      join_double_vector(feedback_tau_est).c_str(),
      valid_mask.c_str());

    last_torque_log_time_ = now;
  }

  void publish_temperature_if_needed(uint64_t now_ns)
  {
    if (!temperature_publisher_ || temperature_publish_period_ns_ == 0) {
      return;
    }
    if (
      last_temperature_publish_ns_ != 0 &&
      now_ns >= last_temperature_publish_ns_ &&
      (now_ns - last_temperature_publish_ns_) < temperature_publish_period_ns_)
    {
      return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "motor";
    msg.layout.dim[0].size = hardware_ids_.size();
    msg.layout.dim[0].stride = hardware_ids_.size() * 2;
    msg.layout.dim[1].label = "sensor[Tmos,Tcoil]";
    msg.layout.dim[1].size = 2;
    msg.layout.dim[1].stride = 2;
    msg.layout.data_offset = 0;
    msg.data.reserve(hardware_ids_.size() * 2);

    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      motor_fbpara_t feedback{};
      uint64_t feedback_ns = 0;
      const bool has_feedback =
        dm_motor_get_feedback_snapshot(hardware_ids_[i], &feedback, &feedback_ns);
      const bool fresh_feedback =
        has_feedback && feedback_ns <= now_ns && (now_ns - feedback_ns) <= feedback_timeout_ns_;
      if (!fresh_feedback || !std::isfinite(feedback.Tmos) || !std::isfinite(feedback.Tcoil)) {
        msg.data.push_back(nan_value);
        msg.data.push_back(nan_value);
        continue;
      }

      msg.data.push_back(feedback.Tmos);
      msg.data.push_back(feedback.Tcoil);
    }

    temperature_publisher_->publish(msg);
    last_temperature_publish_ns_ = now_ns;
  }

  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr subscription_;
  rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temperature_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<uint16_t> hardware_ids_;
  std::vector<uint16_t> feedback_ids_;
  std::vector<double> joint_zero_offsets_;
  std::unordered_set<uint16_t> inverted_motor_ids_;
  std::map<uint16_t, robot_msgs::msg::MotorState> last_valid_states_;
  std::vector<robot_msgs::msg::MotorCommand> cached_commands_;
  std::mutex command_mutex_;

  int channel_{1};
  int can_baud_{1000000};
  int canfd_baud_{5000000};
  double loop_hz_{100.0};
  std::string temperature_topic_;
  double temperature_publish_hz_{1.0};

  bool interface_ready_{false};
  bool feedback_healthy_{false};
  bool initial_enable_completed_{false};
  bool ready_state_initialized_{false};
  bool last_published_ready_{false};
  uint64_t feedback_timeout_ns_{50000000ull};
  uint64_t command_timeout_ns_{100000000ull};
  uint64_t last_command_ns_{0};
  bool has_valid_command_{false};
  double q_jump_threshold_rad_{kDefaultQJumpThresholdRad};
  double torque_log_period_sec_{kDefaultTorqueLogPeriodSec};
  std::vector<uint64_t> q_jump_reject_counts_;
  std::vector<double> feedback_position_biases_;
  std::chrono::steady_clock::time_point last_torque_log_time_{};
  uint64_t temperature_publish_period_ns_{1000000000ull};
  uint64_t last_temperature_publish_ns_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DmMotorRobotDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "dm_motor_robot_driver_node error: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
