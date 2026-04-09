#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_msgs/msg/motor_command.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

namespace
{

constexpr size_t kDefaultMotorCount = 5;

}  // namespace

class DmMotorRobotStressNode : public rclcpp::Node
{
public:
  DmMotorRobotStressNode()
  : Node("dm_motor_robot_stress_node")
  {
    motor_count_ = static_cast<size_t>(declare_parameter<int>("motor_count", static_cast<int>(kDefaultMotorCount)));
    if (motor_count_ == 0) {
      motor_count_ = kDefaultMotorCount;
    }

    publish_hz_ = declare_parameter<double>("publish_hz", 100.0);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 2.0);
    sine_amplitude_ = declare_parameter<double>("sine_amplitude", 0.2);
    sine_frequency_hz_ = declare_parameter<double>("sine_frequency_hz", 0.2);
    phase_step_rad_ = declare_parameter<double>("phase_step_rad", 0.25);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 1.0);
    require_ready_ = declare_parameter<bool>("require_ready", false);
    auto default_kp = declare_parameter<std::vector<double>>(
      "default_kp", std::vector<double>(motor_count_, 0.0));
    auto default_kd = declare_parameter<std::vector<double>>(
      "default_kd", std::vector<double>(motor_count_, 0.0));
    auto default_tau = declare_parameter<std::vector<double>>(
      "default_tau", std::vector<double>(motor_count_, 0.0));

    normalize_vector_param(default_kp, "default_kp", 0.0);
    normalize_vector_param(default_kd, "default_kd", 0.0);
    normalize_vector_param(default_tau, "default_tau", 0.0);
    default_kp_ = default_kp;
    default_kd_ = default_kd;
    default_tau_ = default_tau;

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    command_publisher_ = create_publisher<robot_msgs::msg::RobotCommand>("/arm2/_lowCmd/command", state_qos);
    state_subscription_ = create_subscription<robot_msgs::msg::RobotState>(
      "/arm2/_lowState/joint", state_qos,
      std::bind(&DmMotorRobotStressNode::on_state, this, std::placeholders::_1));
    ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_driver/ready", ready_qos,
      std::bind(&DmMotorRobotStressNode::on_ready, this, std::placeholders::_1));

    last_valid_per_motor_.assign(motor_count_, false);

    const auto publish_period = std::chrono::duration<double>(1.0 / std::max(publish_hz_, 1.0));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&DmMotorRobotStressNode::publish_command, this));

    const auto report_period = std::chrono::duration<double>(std::max(report_period_sec_, 0.2));
    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(report_period),
      std::bind(&DmMotorRobotStressNode::report_stats, this));

    start_time_ = now_steady();
    last_report_time_ = start_time_;

    RCLCPP_INFO(
      get_logger(),
      "stress node started. motor_count=%zu publish_hz=%.1f require_ready=%s kp/kd/tau default all zero=%s",
      motor_count_, publish_hz_, require_ready_ ? "true" : "false",
      command_profile_is_passive() ? "true" : "false");
  }

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  static SteadyTimePoint now_steady()
  {
    return std::chrono::steady_clock::now();
  }

  void normalize_vector_param(std::vector<double> & values, const char * name, double fallback)
  {
    if (values.size() == motor_count_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(), "参数 %s 长度为 %zu，期望为 %zu。将使用默认值 %.3f。",
      name, values.size(), motor_count_, fallback);
    values.assign(motor_count_, fallback);
  }

  void on_ready(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    ready_received_ = true;
    ready_state_ = msg->data;
    ++ready_updates_;
  }

  void on_state(const robot_msgs::msg::RobotState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    ++received_state_count_;
    last_state_time_ = now_steady();

    const size_t expected = motor_count_;
    if (msg->motor_state.size() != expected) {
      ++size_mismatch_count_;
      return;
    }

    size_t valid_count = 0;
    for (size_t i = 0; i < expected; ++i) {
      last_valid_per_motor_[i] = msg->motor_state[i].valid;
      if (msg->motor_state[i].valid) {
        ++valid_count;
      }
    }

    if (valid_count == expected) {
      ++all_valid_state_count_;
    } else {
      ++partial_invalid_state_count_;
    }
  }

  void publish_command()
  {
    const auto now = now_steady();
    if (require_ready_ && (!ready_received_ || !ready_state_)) {
      return;
    }

    robot_msgs::msg::RobotCommand cmd_msg;
    cmd_msg.motor_command.resize(motor_count_);

    const double elapsed_sec = std::chrono::duration<double>(now - start_time_).count();
    for (size_t i = 0; i < motor_count_; ++i) {
      auto & cmd = cmd_msg.motor_command[i];
      const double phase = elapsed_sec * 2.0 * M_PI * sine_frequency_hz_ + phase_step_rad_ * static_cast<double>(i);
      const double q = sine_amplitude_ * std::sin(phase);
      const double dq = sine_amplitude_ * 2.0 * M_PI * sine_frequency_hz_ * std::cos(phase);

      cmd.q = static_cast<float>(q);
      cmd.dq = static_cast<float>(dq);
      cmd.tau = static_cast<float>(default_tau_[i]);
      cmd.kp = static_cast<float>(default_kp_[i]);
      cmd.kd = static_cast<float>(default_kd_[i]);
    }

    command_publisher_->publish(cmd_msg);
    ++sent_command_count_;
    last_command_time_ = now;
  }

  void report_stats()
  {
    const auto now = now_steady();
    const double report_dt = std::chrono::duration<double>(now - last_report_time_).count();
    const double total_dt = std::chrono::duration<double>(now - start_time_).count();
    const double tx_hz = report_dt > 0.0 ? static_cast<double>(sent_command_count_ - last_report_sent_) / report_dt : 0.0;
    const double rx_hz = report_dt > 0.0 ? static_cast<double>(received_state_count_ - last_report_received_) / report_dt : 0.0;
    const bool command_stale =
      (sent_command_count_ == 0) ||
      (std::chrono::duration<double>(now - last_command_time_).count() > command_timeout_sec_);
    const bool state_stale =
      (received_state_count_ == 0) ||
      (std::chrono::duration<double>(now - last_state_time_).count() > command_timeout_sec_);
    const auto command_summary = summarize_command_profile();

    std::string valid_text;
    for (size_t i = 0; i < last_valid_per_motor_.size(); ++i) {
      valid_text += (last_valid_per_motor_[i] ? '1' : '0');
      if (i + 1 != last_valid_per_motor_.size()) {
        valid_text += ',';
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "stress total=%.1fs tx=%llu rx=%llu tx_hz=%.1f rx_hz=%.1f ready=%s ready_updates=%llu all_valid=%llu partial_invalid=%llu size_mismatch=%llu cmd_stale=%s state_stale=%s valid_mask=[%s]",
      total_dt,
      static_cast<unsigned long long>(sent_command_count_),
      static_cast<unsigned long long>(received_state_count_),
      tx_hz,
      rx_hz,
      ready_received_ ? (ready_state_ ? "true" : "false") : "unknown",
      static_cast<unsigned long long>(ready_updates_),
      static_cast<unsigned long long>(all_valid_state_count_),
      static_cast<unsigned long long>(partial_invalid_state_count_),
      static_cast<unsigned long long>(size_mismatch_count_),
      command_stale ? "true" : "false",
      state_stale ? "true" : "false",
      valid_text.c_str());
    RCLCPP_INFO(get_logger(), "cmd_profile %s", command_summary.c_str());

    last_report_time_ = now;
    last_report_sent_ = sent_command_count_;
    last_report_received_ = received_state_count_;
  }

  bool command_profile_is_passive() const
  {
    return std::all_of(
      default_kp_.begin(), default_kp_.end(), [](double value) { return value == 0.0; }) &&
           std::all_of(
             default_kd_.begin(), default_kd_.end(), [](double value) { return value == 0.0; }) &&
           std::all_of(
             default_tau_.begin(), default_tau_.end(), [](double value) { return value == 0.0; });
  }

  std::string summarize_command_profile() const
  {
    const auto max_abs = [](const std::vector<double> & values) {
      double max_value = 0.0;
      for (const double value : values) {
        max_value = std::max(max_value, std::abs(value));
      }
      return max_value;
    };

    char buffer[256];
    std::snprintf(
      buffer, sizeof(buffer),
      "q_sine_amp=%.3f dq_sine_peak=%.3f max|kp|=%.3f max|kd|=%.3f max|tau|=%.3f passive=%s",
      sine_amplitude_,
      sine_amplitude_ * 2.0 * M_PI * sine_frequency_hz_,
      max_abs(default_kp_),
      max_abs(default_kd_),
      max_abs(default_tau_),
      command_profile_is_passive() ? "true" : "false");
    return std::string(buffer);
  }

  rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  size_t motor_count_{kDefaultMotorCount};
  double publish_hz_{100.0};
  double report_period_sec_{2.0};
  double sine_amplitude_{0.2};
  double sine_frequency_hz_{0.2};
  double phase_step_rad_{0.25};
  double command_timeout_sec_{1.0};
  bool require_ready_{false};

  std::vector<double> default_kp_;
  std::vector<double> default_kd_;
  std::vector<double> default_tau_;
  std::vector<bool> last_valid_per_motor_;

  bool ready_received_{false};
  bool ready_state_{false};
  uint64_t ready_updates_{0};
  uint64_t sent_command_count_{0};
  uint64_t received_state_count_{0};
  uint64_t all_valid_state_count_{0};
  uint64_t partial_invalid_state_count_{0};
  uint64_t size_mismatch_count_{0};
  uint64_t last_report_sent_{0};
  uint64_t last_report_received_{0};

  SteadyTimePoint start_time_{};
  SteadyTimePoint last_report_time_{};
  SteadyTimePoint last_command_time_{};
  SteadyTimePoint last_state_time_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DmMotorRobotStressNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
