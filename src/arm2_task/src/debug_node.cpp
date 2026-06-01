#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class DebugNode : public rclcpp::Node
{
public:
  DebugNode()
  : Node("debug_node")
  {
    command_topic_ =
      this->declare_parameter("debug.command_topic", std::string("/arm2/_lowCmd/command"));
    state_topic_ =
      this->declare_parameter("debug.state_topic", std::string("/arm2/_lowState/joint"));
    temperature_topic_ =
      this->declare_parameter("debug.temperature_topic", std::string("/arm2/_lowState/temperature"));
    expected_motor_count_ = this->declare_parameter("debug.expected_motor_count", 5);
    temperature_monitor_axis_index_ = this->declare_parameter("debug.temperature_monitor_axis_index", 2);
    temperature_warning_c_ = this->declare_parameter("debug.temperature_warning_c", 70.0);
    temperature_error_c_ = this->declare_parameter("debug.temperature_error_c", 80.0);
    summary_hz_ = this->declare_parameter("debug.summary_hz", 1.0);
    command_qos_depth_ = this->declare_parameter("debug.command_qos_depth", 1);
    state_qos_depth_ = this->declare_parameter("debug.state_qos_depth", 1);
    temperature_qos_depth_ = this->declare_parameter("debug.temperature_qos_depth", 1);

    if (expected_motor_count_ < 1) {
      expected_motor_count_ = 5;
    }
    if (temperature_monitor_axis_index_ < 0) {
      temperature_monitor_axis_index_ = 0;
    }
    if (temperature_monitor_axis_index_ >= expected_motor_count_) {
      temperature_monitor_axis_index_ = expected_motor_count_ - 1;
    }
    if (!std::isfinite(temperature_warning_c_)) {
      temperature_warning_c_ = 70.0;
    }
    if (!std::isfinite(temperature_error_c_)) {
      temperature_error_c_ = 80.0;
    }
    if (temperature_error_c_ < temperature_warning_c_) {
      temperature_error_c_ = temperature_warning_c_;
    }
    if (summary_hz_ < 0.1) {
      summary_hz_ = 0.1;
    }
    if (command_qos_depth_ < 1) {
      command_qos_depth_ = 1;
    }
    if (state_qos_depth_ < 1) {
      state_qos_depth_ = 1;
    }
    if (temperature_qos_depth_ < 1) {
      temperature_qos_depth_ = 1;
    }

    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(command_qos_depth_)))
      .best_effort()
      .durability_volatile();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(state_qos_depth_)))
      .best_effort()
      .durability_volatile();
    auto temperature_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(temperature_qos_depth_)))
      .best_effort()
      .durability_volatile();

    command_sub_ = this->create_subscription<robot_msgs::msg::RobotCommand>(
      command_topic_,
      command_qos,
      std::bind(&DebugNode::command_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
      state_topic_,
      state_qos,
      std::bind(&DebugNode::state_callback, this, std::placeholders::_1));
    temperature_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      temperature_topic_,
      temperature_qos,
      std::bind(&DebugNode::temperature_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / summary_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DebugNode::log_summary, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Debug node started. command_topic=%s state_topic=%s temperature_topic=%s summary_hz=%.2f expected_motor_count=%d",
      command_topic_.c_str(),
      state_topic_.c_str(),
      temperature_topic_.c_str(),
      summary_hz_,
      expected_motor_count_);
    RCLCPP_INFO(
      this->get_logger(),
      "Temperature monitor armed on axis=%d warning=%.1fC error=%.1fC",
      temperature_monitor_axis_index_ + 1,
      temperature_warning_c_,
      temperature_error_c_);
  }

private:
  struct CommandSnapshot
  {
    std::vector<robot_msgs::msg::MotorCommand> motors;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t count{0};
  };

  struct StateSnapshot
  {
    std::vector<robot_msgs::msg::MotorState> motors;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t count{0};
  };

  struct TemperatureSnapshot
  {
    std::vector<float> data;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t count{0};
  };

  static std::string format_float_vector(const std::vector<double> & values)
  {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i != 0) {
        oss << ", ";
      }
      if (!std::isfinite(values[i])) {
        oss << "nan";
      } else {
        oss << std::fixed << std::setprecision(3) << values[i];
      }
    }
    oss << "]";
    return oss.str();
  }

  static std::string format_valid_mask(const std::vector<robot_msgs::msg::MotorState> & motors)
  {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < motors.size(); ++i) {
      if (i != 0) {
        oss << ", ";
      }
      oss << (motors[i].valid ? '1' : '0');
    }
    oss << "]";
    return oss.str();
  }

  std::string format_temperature_summary(const TemperatureSnapshot & snapshot) const
  {
    if (snapshot.data.empty()) {
      return "[]";
    }

    if (snapshot.data.size() % 2U == 0U) {
      std::vector<double> tmos;
      std::vector<double> tcoil;
      tmos.reserve(snapshot.data.size() / 2U);
      tcoil.reserve(snapshot.data.size() / 2U);
      for (size_t i = 0; i + 1 < snapshot.data.size(); i += 2) {
        tmos.push_back(static_cast<double>(snapshot.data[i]));
        tcoil.push_back(static_cast<double>(snapshot.data[i + 1]));
      }

      std::ostringstream oss;
      oss << "{Tmos=" << format_float_vector(tmos)
          << ", Tcoil=" << format_float_vector(tcoil) << "}";
      return oss.str();
    }

    std::vector<double> raw;
    raw.reserve(snapshot.data.size());
    for (const float value : snapshot.data) {
      raw.push_back(static_cast<double>(value));
    }
    return format_float_vector(raw);
  }

  static double age_seconds(const rclcpp::Time & now, const rclcpp::Time & stamp)
  {
    if (stamp.nanoseconds() == 0) {
      return -1.0;
    }
    return (now - stamp).seconds();
  }

  void check_axis_temperature(const TemperatureSnapshot & snapshot)
  {
    const size_t base_index = static_cast<size_t>(temperature_monitor_axis_index_) * 2U;
    if (snapshot.data.size() < base_index + 2U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Temperature monitor: axis %d unavailable in message of size %zu.",
        temperature_monitor_axis_index_ + 1,
        snapshot.data.size());
      return;
    }

    const double tmos = static_cast<double>(snapshot.data[base_index]);
    const double tcoil = static_cast<double>(snapshot.data[base_index + 1U]);
    const double max_temp = std::max(tmos, tcoil);

    if (!std::isfinite(tmos) || !std::isfinite(tcoil)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Axis %d temperature invalid: Tmos=%.3f Tcoil=%.3f",
        temperature_monitor_axis_index_ + 1,
        tmos,
        tcoil);
      return;
    }

    if (max_temp >= temperature_error_c_) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Axis %d over-temperature: Tmos=%.1fC Tcoil=%.1fC threshold=%.1fC",
        temperature_monitor_axis_index_ + 1,
        tmos,
        tcoil,
        temperature_error_c_);
      return;
    }

    if (max_temp >= temperature_warning_c_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Axis %d high temperature: Tmos=%.1fC Tcoil=%.1fC warning=%.1fC",
        temperature_monitor_axis_index_ + 1,
        tmos,
        tcoil,
        temperature_warning_c_);
    }
  }

  void command_callback(const robot_msgs::msg::RobotCommand::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_command_.motors = msg->motor_command;
    last_command_.stamp = this->get_clock()->now();
    ++last_command_.count;
  }

  void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_state_.motors = msg->motor_state;
    last_state_.stamp = this->get_clock()->now();
    ++last_state_.count;
  }

  void temperature_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_temperature_.data = msg->data;
    last_temperature_.stamp = this->get_clock()->now();
    ++last_temperature_.count;
  }

  void log_summary()
  {
    const rclcpp::Time now = this->get_clock()->now();

    CommandSnapshot command;
    StateSnapshot state;
    TemperatureSnapshot temperature;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      command = last_command_;
      state = last_state_;
      temperature = last_temperature_;
    }

    if (command.count == 0 || state.count == 0 || temperature.count == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Debug node waiting for topics: command=%s state=%s temperature=%s",
        command.count > 0 ? "ok" : "missing",
        state.count > 0 ? "ok" : "missing",
        temperature.count > 0 ? "ok" : "missing");
      return;
    }

    std::vector<double> command_q;
    std::vector<double> command_tau;
    command_q.reserve(command.motors.size());
    command_tau.reserve(command.motors.size());
    for (const auto & motor : command.motors) {
      command_q.push_back(static_cast<double>(motor.q));
      command_tau.push_back(static_cast<double>(motor.tau));
    }

    std::vector<double> state_q;
    std::vector<double> state_dq;
    std::vector<double> state_tau;
    state_q.reserve(state.motors.size());
    state_dq.reserve(state.motors.size());
    state_tau.reserve(state.motors.size());
    for (const auto & motor : state.motors) {
      state_q.push_back(static_cast<double>(motor.q));
      state_dq.push_back(static_cast<double>(motor.dq));
      state_tau.push_back(static_cast<double>(motor.tau_est));
    }

    const double command_age = age_seconds(now, command.stamp);
    const double state_age = age_seconds(now, state.stamp);
    const double temperature_age = age_seconds(now, temperature.stamp);

    check_axis_temperature(temperature);

    RCLCPP_INFO(
      this->get_logger(),
      "Debug summary | cmd_age=%.3fs state_age=%.3fs temp_age=%.3fs | cmd_size=%zu state_size=%zu temp_size=%zu | q_cmd=%s | q_fb=%s | dq_fb=%s | tau_cmd=%s | tau_fb=%s | valid=%s | temp=%s",
      command_age,
      state_age,
      temperature_age,
      command.motors.size(),
      state.motors.size(),
      temperature.data.size(),
      format_float_vector(command_q).c_str(),
      format_float_vector(state_q).c_str(),
      format_float_vector(state_dq).c_str(),
      format_float_vector(command_tau).c_str(),
      format_float_vector(state_tau).c_str(),
      format_valid_mask(state.motors).c_str(),
      format_temperature_summary(temperature).c_str());
  }

  std::mutex data_mutex_;
  CommandSnapshot last_command_;
  StateSnapshot last_state_;
  TemperatureSnapshot last_temperature_;

  std::string command_topic_;
  std::string state_topic_;
  std::string temperature_topic_;
  int expected_motor_count_{5};
  int temperature_monitor_axis_index_{2};
  double temperature_warning_c_{70.0};
  double temperature_error_c_{80.0};
  double summary_hz_{1.0};
  int command_qos_depth_{1};
  int state_qos_depth_{1};
  int temperature_qos_depth_{1};

  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr temperature_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugNode>());
  rclcpp::shutdown();
  return 0;
}
