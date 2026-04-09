#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class InverseDynamicsTargetTestNode : public rclcpp::Node
{
public:
    InverseDynamicsTargetTestNode() : Node("inverse_dynamics_target_test_node")
    {
        target_topic_ =
            this->declare_parameter("inverse_dynamics_test.target_topic", "/joint_target_state");
        state_topic_ =
            this->declare_parameter("inverse_dynamics_test.state_topic", "/arm2/_lowState/joint");
        ready_topic_ =
            this->declare_parameter("inverse_dynamics_test.ready_topic", "/robot_driver/ready");
        target_qos_depth_ =
            this->declare_parameter("inverse_dynamics_test.target_qos_depth", 10);
        publish_rate_hz_ =
            this->declare_parameter("inverse_dynamics_test.publish_rate_hz", 100.0);
        motion_frequency_hz_ =
            this->declare_parameter("inverse_dynamics_test.motion_frequency_hz", 0.12);
        safety_margin_rad_ =
            this->declare_parameter("inverse_dynamics_test.safety_margin_rad", 0.08);
        motion_range_ratio_ =
            this->declare_parameter("inverse_dynamics_test.motion_range_ratio", 0.12);
        max_abs_offset_rad_ =
            this->declare_parameter("inverse_dynamics_test.max_abs_offset_rad", 0.35);
        phase_offsets_ = this->declare_parameter<std::vector<double>>(
            "inverse_dynamics_test.phase_offsets",
            std::vector<double>{0.0, 0.5, 1.0, 1.4, 1.8});
        active_joint_order_ = this->declare_parameter<std::vector<int64_t>>(
            "inverse_dynamics_test.active_joint_order", std::vector<int64_t>{1, 2, 3, 4, 5});
        lower_limits_ = this->declare_parameter<std::vector<double>>(
            "inverse_dynamics_test.lower_limits",
            std::vector<double>{-3.1415, 0.0, -3.0, -2.0, -3.1415});
        upper_limits_ = this->declare_parameter<std::vector<double>>(
            "inverse_dynamics_test.upper_limits",
            std::vector<double>{3.1415, 3.14159265, 0.15, 1.57, 3.1415});

        normalize_vector_param(lower_limits_, -3.1415, "inverse_dynamics_test.lower_limits");
        normalize_vector_param(upper_limits_, 3.1415, "inverse_dynamics_test.upper_limits");
        normalize_vector_param(phase_offsets_, 0.0, "inverse_dynamics_test.phase_offsets");
        normalize_joint_order();

        if (publish_rate_hz_ < 1.0) {
            publish_rate_hz_ = 1.0;
        }
        if (publish_rate_hz_ < 150.0) {
            publish_rate_hz_ = 150.0;
        }
        if (target_qos_depth_ < 1) {
            target_qos_depth_ = 1;
        }
        if (safety_margin_rad_ < 0.0) {
            safety_margin_rad_ = 0.0;
        }
        if (motion_range_ratio_ <= 0.0) {
            motion_range_ratio_ = 0.05;
        }
        if (max_abs_offset_rad_ <= 0.0) {
            max_abs_offset_rad_ = 0.1;
        }
        if (motion_frequency_hz_ <= 0.0) {
            motion_frequency_hz_ = 0.12;
        }

        current_q_.fill(0.0);
        initialized_home_q_.fill(0.0);
        last_target_q_.fill(0.0);
        motion_amplitudes_.fill(0.0);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        target_pub_ = this->create_publisher<robot_msgs::msg::RobotState>(target_topic_, target_qos);
        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            qos,
            std::bind(&InverseDynamicsTargetTestNode::state_callback, this, std::placeholders::_1));
        ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&InverseDynamicsTargetTestNode::ready_callback, this, std::placeholders::_1));

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&InverseDynamicsTargetTestNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Inverse dynamics target test node started. target_topic=%s state_topic=%s ready_topic=%s publish_rate=%.1fHz",
            target_topic_.c_str(),
            state_topic_.c_str(),
            ready_topic_.c_str(),
            publish_rate_hz_);
    }

private:
    static constexpr int kDof = 5;

    static std::string format_vector(const std::array<double, kDof> & values)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i) {
            if (i != 0) {
                oss << ", ";
            }
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }

    void normalize_vector_param(std::vector<double> & values, double fallback, const char * name)
    {
        if (values.size() == kDof) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Parameter %s has size %zu, expected %d. Filling with fallback %.3f.",
            name,
            values.size(),
            kDof,
            fallback);
        values.assign(kDof, fallback);
    }

    void normalize_joint_order()
    {
        std::vector<int> normalized;
        normalized.reserve(active_joint_order_.size());
        std::array<bool, kDof> seen{};
        for (const auto raw_joint : active_joint_order_) {
            if (raw_joint < 1 || raw_joint > kDof) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ignore out-of-range joint index %ld in inverse_dynamics_test.active_joint_order.",
                    raw_joint);
                continue;
            }
            const int zero_based = static_cast<int>(raw_joint - 1);
            if (seen[static_cast<size_t>(zero_based)]) {
                continue;
            }
            seen[static_cast<size_t>(zero_based)] = true;
            normalized.push_back(zero_based);
        }

        if (normalized.empty()) {
            normalized = {0, 1, 2, 3, 4};
        }
        active_joint_indices_ = normalized;
    }

    bool extract_valid_state(
        const robot_msgs::msg::RobotState::SharedPtr & msg,
        std::array<double, kDof> & q_out) const
    {
        if (!msg || msg->motor_state.size() < static_cast<size_t>(kDof)) {
            return false;
        }

        for (int i = 0; i < kDof; ++i) {
            const auto & motor = msg->motor_state[static_cast<size_t>(i)];
            if (!motor.valid || !std::isfinite(motor.q)) {
                return false;
            }
            q_out[i] = motor.q;
        }
        return true;
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        std::array<double, kDof> q{};
        if (!extract_valid_state(msg, q)) {
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        current_q_ = q;
        has_measured_state_ = true;

        if (!home_initialized_) {
            initialize_home_locked();
        }
    }

    void ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        if (driver_ready_ != msg->data) {
            RCLCPP_INFO(
                this->get_logger(),
                "Driver ready changed: %s -> %s",
                driver_ready_ ? "true" : "false",
                msg->data ? "true" : "false");
        }
        driver_ready_ = msg->data;
    }

    void initialize_home_locked()
    {
        initialized_home_q_ = current_q_;
        for (int i = 0; i < kDof; ++i) {
            const double lower = lower_limits_[static_cast<size_t>(i)] + safety_margin_rad_;
            const double upper = upper_limits_[static_cast<size_t>(i)] - safety_margin_rad_;
            initialized_home_q_[i] = std::clamp(initialized_home_q_[i], lower, upper);

            const double available_positive = std::max(0.0, upper - initialized_home_q_[i]);
            const double available_negative = std::max(0.0, initialized_home_q_[i] - lower);
            const double limit_span =
                upper_limits_[static_cast<size_t>(i)] - lower_limits_[static_cast<size_t>(i)];
            const double desired_amplitude =
                std::min(max_abs_offset_rad_, limit_span * motion_range_ratio_);
            motion_amplitudes_[i] =
                std::max(0.0, std::min(desired_amplitude, std::min(available_positive, available_negative)));
        }

        home_initialized_ = true;
        motion_start_time_ = this->now();
        last_target_q_ = initialized_home_q_;

        RCLCPP_INFO(
            this->get_logger(),
            "Initialized test home q=%s amplitudes=%s",
            format_vector(initialized_home_q_).c_str(),
            format_vector(motion_amplitudes_).c_str());
    }

    void timer_callback()
    {
        if (!driver_ready_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Test target publisher waiting for /robot_driver/ready.");
            return;
        }

        std::array<double, kDof> home{};
        std::array<double, kDof> amplitudes{};
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!has_measured_state_ || !home_initialized_) {
                return;
            }
            home = initialized_home_q_;
            amplitudes = motion_amplitudes_;
        }

        const double elapsed = (this->now() - motion_start_time_).seconds();
        const double omega = 2.0 * M_PI * motion_frequency_hz_;
        std::array<bool, kDof> active_joint_mask{};
        for (const int joint_index : active_joint_indices_) {
            active_joint_mask[static_cast<size_t>(joint_index)] = true;
        }

        std::array<double, kDof> target_q = home;
        std::array<double, kDof> target_dq{};
        std::array<double, kDof> target_ddq{};
        for (int i = 0; i < kDof; ++i) {
            if (!active_joint_mask[static_cast<size_t>(i)] || amplitudes[static_cast<size_t>(i)] <= 1e-6) {
                continue;
            }

            const double phase =
                omega * elapsed + phase_offsets_[static_cast<size_t>(i)];
            const double sin_phase = std::sin(phase);
            const double cos_phase = std::cos(phase);
            const double amplitude = amplitudes[static_cast<size_t>(i)];

            target_q[static_cast<size_t>(i)] =
                home[static_cast<size_t>(i)] + amplitude * sin_phase;
            target_dq[static_cast<size_t>(i)] = amplitude * omega * cos_phase;
            target_ddq[static_cast<size_t>(i)] = -amplitude * omega * omega * sin_phase;
        }

        robot_msgs::msg::RobotState target_msg;
        target_msg.motor_state.resize(kDof);
        for (int i = 0; i < kDof; ++i) {
            auto & motor = target_msg.motor_state[static_cast<size_t>(i)];
            motor.q = static_cast<float>(target_q[static_cast<size_t>(i)]);
            motor.dq = static_cast<float>(target_dq[static_cast<size_t>(i)]);
            motor.ddq = static_cast<float>(target_ddq[static_cast<size_t>(i)]);
            motor.tau_est = 0.0f;
            motor.cur = 0.0f;
            motor.valid = true;
        }

        target_pub_->publish(target_msg);

        last_target_q_ = target_q;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Publishing coupled slow target: freq=%.3fHz q=%s dq=%s ddq=%s",
            motion_frequency_hz_,
            format_vector(last_target_q_).c_str(),
            format_vector(target_dq).c_str(),
            format_vector(target_ddq).c_str());
    }

    std::mutex data_mutex_;
    bool driver_ready_{false};
    bool has_measured_state_{false};
    bool home_initialized_{false};
    std::array<double, kDof> current_q_{};
    std::array<double, kDof> initialized_home_q_{};
    std::array<double, kDof> motion_amplitudes_{};
    std::array<double, kDof> last_target_q_{};
    rclcpp::Time motion_start_time_{0, 0, RCL_ROS_TIME};

    std::string target_topic_;
    std::string state_topic_;
    std::string ready_topic_;
    double publish_rate_hz_{100.0};
    int target_qos_depth_{10};
    double motion_frequency_hz_{0.12};
    double safety_margin_rad_{0.08};
    double motion_range_ratio_{0.12};
    double max_abs_offset_rad_{0.35};
    std::vector<double> phase_offsets_;
    std::vector<int64_t> active_joint_order_;
    std::vector<int> active_joint_indices_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;

    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr target_pub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseDynamicsTargetTestNode>());
    rclcpp::shutdown();
    return 0;
}
