#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ftservo_hls3625_teach/servo_bus.hpp"

namespace ftservo_hls3625_teach
{

class BusStatePublisher : public rclcpp::Node
{
public:
  BusStatePublisher()
  : Node("bus_state_publisher"),
    config_(declare_bus_config(*this)),
    bus_(config_)
  {
    position_filter_alpha_ = declare_parameter<double>("position_low_pass_alpha", 0.25);
    velocity_filter_alpha_ = declare_parameter<double>("velocity_low_pass_alpha", 0.25);
    position_filter_alpha_ = clamp_filter_alpha(position_filter_alpha_, "position_low_pass_alpha");
    velocity_filter_alpha_ = clamp_filter_alpha(velocity_filter_alpha_, "velocity_low_pass_alpha");
    filtered_positions_.assign(config_.ids.size(), 0.0);
    filtered_velocities_.assign(config_.ids.size(), 0.0);

    bus_.ping_all(get_logger());

    publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / config_.publish_hz));
    timer_ = create_wall_timer(period, std::bind(&BusStatePublisher::publish_once, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %zu calibrated joints on /joint_states with low-pass filter alpha(pos)=%.2f alpha(vel)=%.2f",
      config_.ids.size(),
      position_filter_alpha_,
      velocity_filter_alpha_);
  }

private:
  static double apply_low_pass(double previous, double sample, double alpha)
  {
    return previous + alpha * (sample - previous);
  }

  double clamp_filter_alpha(double alpha, const char * param_name)
  {
    if (!std::isfinite(alpha)) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter %s is not finite; falling back to 1.0.",
        param_name);
      return 1.0;
    }
    if (alpha < 0.0 || alpha > 1.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter %s=%.3f is outside [0, 1]; clamping.",
        param_name,
        alpha);
      return std::clamp(alpha, 0.0, 1.0);
    }
    return alpha;
  }

  void publish_once()
  {
    try {
      const auto states = bus_.read_states();

      if (states.size() != filtered_positions_.size()) {
        throw std::runtime_error("servo state size changed unexpectedly");
      }

      sensor_msgs::msg::JointState message;
      message.header.stamp = now();
      message.name = config_.joint_names;
      message.position.reserve(states.size());
      message.velocity.reserve(states.size());

      for (std::size_t i = 0; i < states.size(); ++i) {
        const auto & state = states[i];

        if (!filter_initialized_) {
          filtered_positions_[i] = state.position_rad;
          filtered_velocities_[i] = state.speed_rad_s;
        } else {
          filtered_positions_[i] = apply_low_pass(
            filtered_positions_[i], state.position_rad, position_filter_alpha_);
          filtered_velocities_[i] = apply_low_pass(
            filtered_velocities_[i], state.speed_rad_s, velocity_filter_alpha_);
        }

        message.position.push_back(filtered_positions_[i]);
        message.velocity.push_back(filtered_velocities_[i]);
      }

      filter_initialized_ = true;

      publisher_->publish(message);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "State publish failed: %s",
        error.what());
    }
  }

  BusConfig config_;
  ServoBus bus_;
  bool filter_initialized_{false};
  double position_filter_alpha_{0.25};
  double velocity_filter_alpha_{0.25};
  std::vector<double> filtered_positions_;
  std::vector<double> filtered_velocities_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ftservo_hls3625_teach

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;

  try {
    auto node = std::make_shared<ftservo_hls3625_teach::BusStatePublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    fprintf(stderr, "bus_state_publisher failed: %s\n", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
