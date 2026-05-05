#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
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
constexpr double kAngleWrapLimit = M_PI;
constexpr double kFullTurnRad = 2.0 * M_PI;
constexpr double kDefaultQJumpThresholdRad = 0.03;
constexpr double kDefaultTorqueLogPeriodSec = 1.0;

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

double normalize_angle_to_two_pi(double angle)
{
  const double period = kFullTurnRad;
  if (!std::isfinite(angle) || period <= 0.0) {
    return angle;
  }

  angle = std::fmod(angle + kAngleWrapLimit, period);
  if (angle < 0.0) {
    angle += period;
  }
  return angle - kAngleWrapLimit;
}

double shortest_angular_distance(double from, double to)
{
  return normalize_angle_to_two_pi(to - from);
}

double distance_to_interval(double value, double lower, double upper)
{
  if (value < lower) {
    return lower - value;
  }
  if (value > upper) {
    return value - upper;
  }
  return 0.0;
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
      "motor_vmax", std::vector<double>{30.0, 10.0, 30.0, 30.0, 50.0});
    auto motor_tmax = declare_parameter<std::vector<double>>(
      "motor_tmax", std::vector<double>{10.0, 28.0, 10.0, 10.0, 5.0});
    auto joint_zero_offsets = declare_parameter<std::vector<double>>(
      "joint_zero_offsets", std::vector<double>{-0.02, 0.14, 0.1, 0.361, 0.0});
    auto joint2_publish_window = declare_parameter<std::vector<double>>(
      "joint2_publish_window", std::vector<double>{-0.5, 3.5});
    q_jump_threshold_rad_ = declare_parameter<double>("q_jump_threshold_rad", kDefaultQJumpThresholdRad);
    torque_log_period_sec_ =
      declare_parameter<double>("torque_log_period_sec", kDefaultTorqueLogPeriodSec);

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

    normalize_vector_param(motor_pmax, "motor_pmax", 12.5);
    normalize_vector_param(motor_vmax, "motor_vmax", 30.0);
    normalize_vector_param(motor_tmax, "motor_tmax", 10.0);
    normalize_vector_param(joint_zero_offsets, "joint_zero_offsets", 0.0);
    normalize_joint2_publish_window(joint2_publish_window);

    feedback_timeout_ns_ = static_cast<uint64_t>(feedback_timeout_ms) * 1000000ull;
    command_timeout_ns_ = static_cast<uint64_t>(command_timeout_ms) * 1000000ull;
    joint_zero_offsets_ = joint_zero_offsets;

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
    latest_feedback_unwrapped_q_.assign(hardware_ids_.size(), 0.0);
    latest_feedback_unwrapped_valid_.assign(hardware_ids_.size(), false);
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

      dm_motor_clear_err(&hcan1, &motor[i]);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      dm_motor_enable(&hcan1, &motor[i]);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    subscription_ = create_subscription<robot_msgs::msg::RobotCommand>(
      "/arm2/_lowCmd/command", state_qos,
      std::bind(&DmMotorRobotDriverNode::command_callback, this, std::placeholders::_1));

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
      "2号电机定制发布区间: enabled=%s window=[%.3f, %.3f]",
      joint2_publish_window_enabled_ ? "true" : "false",
      joint2_publish_window_min_,
      joint2_publish_window_max_);
    RCLCPP_INFO(
      get_logger(), "关节零点偏置[rad]: [%s]", join_double_vector(joint_zero_offsets_).c_str());
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

  void normalize_joint2_publish_window(const std::vector<double> & window)
  {
    joint2_publish_window_enabled_ = false;
    joint2_publish_window_min_ = -0.5;
    joint2_publish_window_max_ = 3.5;

    if (window.size() != 2) {
      RCLCPP_WARN(
        get_logger(),
        "参数 joint2_publish_window 长度为 %zu，期望为 2。将使用默认区间 [-0.5, 3.5]。",
        window.size());
      joint2_publish_window_enabled_ = true;
      return;
    }

    if (!std::isfinite(window[0]) || !std::isfinite(window[1]) || window[0] >= window[1]) {
      RCLCPP_WARN(
        get_logger(),
        "参数 joint2_publish_window 非法([%.3f, %.3f])。将使用默认区间 [-0.5, 3.5]。",
        window[0], window[1]);
      joint2_publish_window_enabled_ = true;
      return;
    }

    joint2_publish_window_min_ = window[0];
    joint2_publish_window_max_ = window[1];
    joint2_publish_window_enabled_ = true;
  }

  double normalize_published_angle(uint16_t motor_id, double q)
  {
    if (motor_id == 2 && joint2_publish_window_enabled_) {
      return normalize_joint2_published_angle(q);
    }
    return normalize_angle_to_two_pi(q);
  }

  double normalize_joint2_published_angle(double q)
  {
    const std::array<double, 3> candidates{q - kFullTurnRad, q, q + kFullTurnRad};
    const auto last_it = last_valid_states_.find(2);
    const bool has_last_valid_q = last_it != last_valid_states_.end() && last_it->second.valid;
    const double last_q = has_last_valid_q ? static_cast<double>(last_it->second.q) : 0.0;
    const double center = 0.5 * (joint2_publish_window_min_ + joint2_publish_window_max_);

    const auto choose_better_candidate = [&](double lhs, double rhs) {
      const double lhs_interval_distance =
        distance_to_interval(lhs, joint2_publish_window_min_, joint2_publish_window_max_);
      const double rhs_interval_distance =
        distance_to_interval(rhs, joint2_publish_window_min_, joint2_publish_window_max_);
      if (lhs_interval_distance != rhs_interval_distance) {
        return lhs_interval_distance < rhs_interval_distance;
      }

      if (has_last_valid_q) {
        const double lhs_continuity = std::abs(lhs - last_q);
        const double rhs_continuity = std::abs(rhs - last_q);
        if (lhs_continuity != rhs_continuity) {
          return lhs_continuity < rhs_continuity;
        }
      }

      const double lhs_center_distance = std::abs(lhs - center);
      const double rhs_center_distance = std::abs(rhs - center);
      if (lhs_center_distance != rhs_center_distance) {
        return lhs_center_distance < rhs_center_distance;
      }

      return lhs < rhs;
    };

    const auto best_it = std::min_element(
      candidates.begin(), candidates.end(),
      [&](double lhs, double rhs) {
        return choose_better_candidate(lhs, rhs);
      });

    const double best_candidate = *best_it;
    const double best_distance_to_window =
      distance_to_interval(best_candidate, joint2_publish_window_min_, joint2_publish_window_max_);
    if (best_distance_to_window > 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "2号电机角度 %.4f rad 的 ±2pi 等效值均不在发布区间 [%.4f, %.4f] 内，选择最接近区间的候选 %.4f rad (distance=%.4f).",
        q,
        joint2_publish_window_min_,
        joint2_publish_window_max_,
        best_candidate,
        best_distance_to_window);
    }

    return best_candidate;
  }

  double reconstruct_command_angle(size_t index, double command_q)
  {
    const uint16_t motor_id = hardware_ids_[index];
    if (motor_id != 2 || !joint2_publish_window_enabled_) {
      return command_q;
    }
    return reconstruct_joint2_command_angle(index, command_q);
  }

  double reconstruct_joint2_command_angle(size_t index, double command_q)
  {
    const std::array<double, 3> candidates{
      command_q - kFullTurnRad, command_q, command_q + kFullTurnRad};

    if (index < latest_feedback_unwrapped_valid_.size() && latest_feedback_unwrapped_valid_[index]) {
      const double reference = latest_feedback_unwrapped_q_[index];
      return *std::min_element(
        candidates.begin(), candidates.end(),
        [reference](double lhs, double rhs) {
          return std::abs(lhs - reference) < std::abs(rhs - reference);
        });
    }

    const auto last_it = last_valid_states_.find(2);
    if (last_it != last_valid_states_.end() && last_it->second.valid) {
      const double reference = static_cast<double>(last_it->second.q);
      return *std::min_element(
        candidates.begin(), candidates.end(),
        [reference](double lhs, double rhs) {
          return std::abs(lhs - reference) < std::abs(rhs - reference);
        });
    }

    return normalize_joint2_published_angle(command_q);
  }

  double command_q_to_motor_pos_set(size_t index, double command_q)
  {
    const uint16_t motor_id = hardware_ids_[index];
    const double direction = should_invert(motor_id) ? -1.0 : 1.0;
    const double reconstructed_q = reconstruct_command_angle(index, command_q);
    return direction * (reconstructed_q + joint_zero_offsets_[index]);
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

  void prepare_safe_command_locked(size_t index)
  {
    const uint16_t motor_id = hardware_ids_[index];
    double hold_q = 0.0;

    const auto it = last_valid_states_.find(motor_id);
    if (it != last_valid_states_.end() && it->second.valid) {
      hold_q = static_cast<double>(it->second.q);
    }

    if (motor_id == 2 &&
        index < latest_feedback_unwrapped_valid_.size() &&
        latest_feedback_unwrapped_valid_[index]) {
      const double direction = should_invert(motor_id) ? -1.0 : 1.0;
      motor[index].ctrl.pos_set = static_cast<float>(
        direction * (latest_feedback_unwrapped_q_[index] + joint_zero_offsets_[index]));
    } else {
      motor[index].ctrl.pos_set = static_cast<float>(command_q_to_motor_pos_set(index, hold_q));
    }
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

  void command_callback(const robot_msgs::msg::RobotCommand::SharedPtr msg)
  {
    if (!msg || !interface_ready_) {
      return;
    }

    const size_t cmd_size = msg->motor_command.size();
    if (cmd_size != hardware_ids_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到控制数组长度为 %zu，期望长度为 %zu。整包拒绝，保持安全输出。",
        cmd_size, hardware_ids_.size());
      last_command_ns_ = 0;
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    for (size_t i = 0; i < hardware_ids_.size(); ++i) {
      const auto & cmd = msg->motor_command[i];
      if (
        !std::isfinite(cmd.q) || !std::isfinite(cmd.dq) || !std::isfinite(cmd.tau) ||
        !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd))
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "拦截: 收到电机 [ID:%u] 的非法控制指令(NaN/Inf)",
          static_cast<unsigned int>(hardware_ids_[i]));
        last_command_ns_ = 0;
        return;
      }

      cached_commands_[i] = cmd;
    }
    last_command_ns_ = monotonic_now_ns();
  }

  void timer_callback()
  {
    if (!interface_ready_) {
      publish_ready_state(false);
      return;
    }

    pump_feedback();

    const uint64_t now_ns = monotonic_now_ns();
    bool command_timed_out;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_timed_out = (last_command_ns_ == 0) || ((now_ns - last_command_ns_) > command_timeout_ns_);
      if (command_timed_out) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "控制命令超时，驱动切换到安全输出。");
      }

      for (size_t i = 0; i < hardware_ids_.size(); ++i) {
        if (command_timed_out) {
          prepare_safe_command_locked(i);
        } else {
          const uint16_t motor_id = hardware_ids_[i];
          const double direction = should_invert(motor_id) ? -1.0 : 1.0;
          motor[i].ctrl.pos_set = static_cast<float>(command_q_to_motor_pos_set(i, cached_commands_[i].q));
          motor[i].ctrl.vel_set = static_cast<float>(direction * cached_commands_[i].dq);
          motor[i].ctrl.tor_set = static_cast<float>(direction * cached_commands_[i].tau);
          motor[i].ctrl.kp_set = cached_commands_[i].kp;
          motor[i].ctrl.kd_set = cached_commands_[i].kd;
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

      if (!fresh_feedback) {
        all_feedback_fresh = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "电机 [ID:%u] 反馈超时或尚未收到首帧，继续发布上一帧有效状态。",
          static_cast<unsigned int>(motor_id));
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
      latest_feedback_unwrapped_q_[i] = q;
      latest_feedback_unwrapped_valid_[i] = true;
      q = normalize_published_angle(motor_id, q);
      dq = direction * dq;
      tau = direction * tau;

      const auto last_it = last_valid_states_.find(motor_id);
      if (last_it != last_valid_states_.end() && last_it->second.valid) {
        const double q_delta =
          std::abs(shortest_angular_distance(static_cast<double>(last_it->second.q), q));
        if (q_delta > q_jump_threshold_rad_) {
          all_feedback_fresh = false;
          ++q_jump_reject_counts_[i];
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "电机 [ID:%u] 位置反馈跳变过大(Δq=%.4f rad, 阈值=%.4f rad)，回退到上一帧有效状态。",
            static_cast<unsigned int>(motor_id), q_delta, q_jump_threshold_rad_);
          state_msg.motor_state[i] = last_it->second;
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
    publish_ready_state(interface_ready_ && feedback_healthy_);
    state_publisher_->publish(state_msg);
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

  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr subscription_;
  rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
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

  bool interface_ready_{false};
  bool feedback_healthy_{false};
  bool ready_state_initialized_{false};
  bool last_published_ready_{false};
  uint64_t feedback_timeout_ns_{50000000ull};
  uint64_t command_timeout_ns_{100000000ull};
  uint64_t last_command_ns_{0};
  double q_jump_threshold_rad_{kDefaultQJumpThresholdRad};
  double torque_log_period_sec_{kDefaultTorqueLogPeriodSec};
  bool joint2_publish_window_enabled_{true};
  double joint2_publish_window_min_{-0.5};
  double joint2_publish_window_max_{3.5};
  std::vector<uint64_t> q_jump_reject_counts_;
  std::vector<double> latest_feedback_unwrapped_q_;
  std::vector<bool> latest_feedback_unwrapped_valid_;
  std::chrono::steady_clock::time_point last_torque_log_time_{};
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
