#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class TeachPendantFollowNode : public rclcpp::Node
{
public:
    TeachPendantFollowNode() : Node("teach_pendant_follow_node")
    {
        teacher_topic_ = this->declare_parameter("teach_pendant.topic", "/joint_states");
        target_topic_ =
            this->declare_parameter("teach_pendant.target_topic", "/joint_target_state");
        mode_service_name_ =
            this->declare_parameter("teach_pendant.mode_service", "set_controller_mode");
        mode_name_ =
            this->declare_parameter("teach_pendant.mode_name", std::string("teach_pendant"));
        request_mode_on_startup_ =
            this->declare_parameter("teach_pendant.request_mode_on_startup", true);
        republish_hz_ = this->declare_parameter("teach_pendant.republish_hz", 50.0);
        max_abs_acceleration_ =
            this->declare_parameter("teach_pendant.max_abs_acceleration", 30.0);
        teacher_qos_depth_ = this->declare_parameter("teach_pendant.teacher_qos_depth", 10);
        target_qos_depth_ = this->declare_parameter("teach_pendant.target_qos_depth", 10);
        log_input_ = this->declare_parameter("teach_pendant.log_input", false);
        expected_joint_names_ = this->declare_parameter<std::vector<std::string>>(
            "teach_pendant.joint_names",
            std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5"});
        angle_window_lower_ = this->declare_parameter<std::vector<double>>(
            "teach_pendant.angle_window_lower",
            std::vector<double>{-M_PI, -0.5, -3.0, -2.0, -M_PI});
        angle_window_upper_ = this->declare_parameter<std::vector<double>>(
            "teach_pendant.angle_window_upper",
            std::vector<double>{M_PI, 3.5, 0.15, 1.57, M_PI});

        if (republish_hz_ < 1.0) {
            republish_hz_ = 1.0;
        }
        if (max_abs_acceleration_ < 0.0) {
            max_abs_acceleration_ = 0.0;
        }
        if (teacher_qos_depth_ < 1) {
            teacher_qos_depth_ = 1;
        }
        if (target_qos_depth_ < 1) {
            target_qos_depth_ = 1;
        }

        normalize_joint_names();
        normalize_angle_windows();

        target_q_ = Eigen::VectorXd::Zero(kDof);
        target_dq_ = Eigen::VectorXd::Zero(kDof);
        target_ddq_ = Eigen::VectorXd::Zero(kDof);
        prev_target_dq_ = Eigen::VectorXd::Zero(kDof);

        auto teacher_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(teacher_qos_depth_)))
                               .best_effort()
                               .durability_volatile();
        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();

        target_pub_ = this->create_publisher<robot_msgs::msg::RobotState>(target_topic_, target_qos);
        teacher_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            teacher_topic_,
            teacher_qos,
            std::bind(&TeachPendantFollowNode::teacher_callback, this, std::placeholders::_1));
        mode_client_ =
            this->create_client<robot_msgs::srv::SetControllerMode>(mode_service_name_);

        const auto republish_period = std::chrono::duration<double>(1.0 / republish_hz_);
        republish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(republish_period),
            std::bind(&TeachPendantFollowNode::publish_target, this));

        if (request_mode_on_startup_) {
            mode_request_timer_ = this->create_wall_timer(
                1s, std::bind(&TeachPendantFollowNode::request_teach_mode_if_needed, this));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Teach pendant bridge started. input=%s [sensor_msgs/msg/JointState], output=%s [robot_msgs/msg/RobotState], mode_service=%s mode=%s republish_hz=%.1f teacher_qos=best_effort/%d target_qos=reliable/%d",
            teacher_topic_.c_str(),
            target_topic_.c_str(),
            mode_service_name_.c_str(),
            mode_name_.c_str(),
            republish_hz_,
            teacher_qos_depth_,
            target_qos_depth_);
    }

private:
    static constexpr int kDof = 5;
    static constexpr double kAngleWrapLimit = M_PI;
    static constexpr double kFullTurnRad = 2.0 * M_PI;

    static std::string format_vector(const Eigen::VectorXd & values)
    {
        std::ostringstream oss;
        oss << "[";
        for (Eigen::Index i = 0; i < values.size(); ++i) {
            if (i != 0) {
                oss << ", ";
            }
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }

    static double normalize_angle_to_pi(double angle)
    {
        if (!std::isfinite(angle)) {
            return angle;
        }

        angle = std::fmod(angle + kAngleWrapLimit, kFullTurnRad);
        if (angle < 0.0) {
            angle += kFullTurnRad;
        }
        return angle - kAngleWrapLimit;
    }

    static double distance_to_interval(double value, double lower, double upper)
    {
        if (value < lower) {
            return lower - value;
        }
        if (value > upper) {
            return value - upper;
        }
        return 0.0;
    }

    static double shortest_angular_distance(double from, double to)
    {
        return normalize_angle_to_pi(to - from);
    }

    void normalize_joint_names()
    {
        if (expected_joint_names_.size() == static_cast<size_t>(kDof)) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "teach_pendant.joint_names size=%zu, expected=%d. Falling back to default joint1..joint5 order.",
            expected_joint_names_.size(),
            kDof);
        expected_joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    }

    void normalize_angle_windows()
    {
        const std::array<double, kDof> default_lower = {-M_PI, -0.5, -3.0, -2.0, -M_PI};
        const std::array<double, kDof> default_upper = {M_PI, 3.5, 0.15, 1.57, M_PI};

        if (angle_window_lower_.size() != static_cast<size_t>(kDof)) {
            RCLCPP_WARN(
                this->get_logger(),
                "teach_pendant.angle_window_lower size=%zu, expected=%d. Falling back to defaults.",
                angle_window_lower_.size(),
                kDof);
            angle_window_lower_.assign(default_lower.begin(), default_lower.end());
        }

        if (angle_window_upper_.size() != static_cast<size_t>(kDof)) {
            RCLCPP_WARN(
                this->get_logger(),
                "teach_pendant.angle_window_upper size=%zu, expected=%d. Falling back to defaults.",
                angle_window_upper_.size(),
                kDof);
            angle_window_upper_.assign(default_upper.begin(), default_upper.end());
        }

        for (int i = 0; i < kDof; ++i) {
            if (!std::isfinite(angle_window_lower_[static_cast<size_t>(i)]) ||
                !std::isfinite(angle_window_upper_[static_cast<size_t>(i)]) ||
                angle_window_lower_[static_cast<size_t>(i)] >=
                    angle_window_upper_[static_cast<size_t>(i)])
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Teach pendant angle window for joint %d is invalid. Resetting to default.",
                    i + 1);
                angle_window_lower_[static_cast<size_t>(i)] = default_lower[static_cast<size_t>(i)];
                angle_window_upper_[static_cast<size_t>(i)] = default_upper[static_cast<size_t>(i)];
            }
        }
    }

    std::vector<int> resolve_joint_indices(const sensor_msgs::msg::JointState & msg)
    {
        if (msg.name.size() < static_cast<size_t>(kDof)) {
            std::vector<int> sequential(kDof);
            for (int i = 0; i < kDof; ++i) {
                sequential[i] = i;
            }
            return sequential;
        }

        std::vector<int> indices(kDof, -1);
        bool all_found = true;
        for (int i = 0; i < kDof; ++i) {
            const auto it = std::find(
                msg.name.begin(), msg.name.end(), expected_joint_names_[static_cast<size_t>(i)]);
            if (it == msg.name.end()) {
                all_found = false;
                break;
            }
            indices[static_cast<size_t>(i)] = static_cast<int>(std::distance(msg.name.begin(), it));
        }

        if (all_found) {
            return indices;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Teach pendant JointState names do not match configured joint order. Falling back to sequential order.");

        std::vector<int> sequential(kDof);
        for (int i = 0; i < kDof; ++i) {
            sequential[i] = i;
        }
        return sequential;
    }

    double unwrap_continuous_angle(
        int joint_index,
        double raw_angle,
        bool has_reference,
        double reference)
    {
        const double wrapped = normalize_angle_to_pi(raw_angle);
        if (!has_reference || !std::isfinite(reference)) {
            return wrapped;
        }

        const double reference_wrapped = normalize_angle_to_pi(reference);
        const double delta = shortest_angular_distance(reference_wrapped, wrapped);
        const double unwrapped = reference + delta;

        (void)joint_index;
        return unwrapped;
    }

    double choose_windowed_angle(int joint_index, double continuous_angle)
    {
        const std::array<double, 3> candidates = {
            continuous_angle - kFullTurnRad,
            continuous_angle,
            continuous_angle + kFullTurnRad,
        };
        const double lower = angle_window_lower_[static_cast<size_t>(joint_index)];
        const double upper = angle_window_upper_[static_cast<size_t>(joint_index)];
        const double center = 0.5 * (lower + upper);

        const auto best_it = std::min_element(
            candidates.begin(),
            candidates.end(),
            [&](double lhs, double rhs) {
                const double lhs_window_distance = distance_to_interval(lhs, lower, upper);
                const double rhs_window_distance = distance_to_interval(rhs, lower, upper);
                if (lhs_window_distance != rhs_window_distance) {
                    return lhs_window_distance < rhs_window_distance;
                }

                const double lhs_center_distance = std::abs(lhs - center);
                const double rhs_center_distance = std::abs(rhs - center);
                if (lhs_center_distance != rhs_center_distance) {
                    return lhs_center_distance < rhs_center_distance;
                }

                return lhs < rhs;
            });

        const double chosen = *best_it;
        const double chosen_window_distance = distance_to_interval(chosen, lower, upper);
        if (chosen_window_distance > 0.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Teach pendant joint %d angle %.4f rad has no equivalent inside [%.4f, %.4f]. Clamping nearest candidate %.4f rad.",
                joint_index + 1,
                continuous_angle,
                lower,
                upper,
                chosen);
        }

        return std::clamp(chosen, lower, upper);
    }

    void teacher_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        if (msg->position.size() < static_cast<size_t>(kDof) ||
            msg->velocity.size() < static_cast<size_t>(kDof))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Teach pendant JointState invalid: position=%zu velocity=%zu expected>=%d",
                msg->position.size(),
                msg->velocity.size(),
                kDof);
            return;
        }

        const auto indices = resolve_joint_indices(*msg);

        Eigen::VectorXd previous_continuous_q = Eigen::VectorXd::Zero(kDof);
        bool has_previous_target = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            previous_continuous_q = last_continuous_q_;
            has_previous_target = has_target_;
        }

        Eigen::VectorXd q(kDof);
        Eigen::VectorXd dq(kDof);
        Eigen::VectorXd continuous_q(kDof);
        for (int i = 0; i < kDof; ++i) {
            const int source_index = indices[static_cast<size_t>(i)];
            if (source_index < 0 ||
                static_cast<size_t>(source_index) >= msg->position.size() ||
                static_cast<size_t>(source_index) >= msg->velocity.size())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Teach pendant JointState index mapping is out of range.");
                return;
            }

            continuous_q[i] = unwrap_continuous_angle(
                i,
                msg->position[static_cast<size_t>(source_index)],
                has_previous_target,
                previous_continuous_q[i]);
            q[i] = choose_windowed_angle(i, continuous_q[i]);
            dq[i] = msg->velocity[static_cast<size_t>(source_index)];

            if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Teach pendant JointState contains NaN/Inf.");
                return;
            }

            const double lower = angle_window_lower_[static_cast<size_t>(i)];
            const double upper = angle_window_upper_[static_cast<size_t>(i)];
            const bool at_lower_limit = std::abs(q[i] - lower) < 1e-9;
            const bool at_upper_limit = std::abs(q[i] - upper) < 1e-9;
            if ((at_lower_limit && dq[i] < 0.0) || (at_upper_limit && dq[i] > 0.0)) {
                dq[i] = 0.0;
            }
        }

        rclcpp::Time stamp(msg->header.stamp);
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
            stamp = this->get_clock()->now();
        }

        Eigen::VectorXd ddq = Eigen::VectorXd::Zero(kDof);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (has_target_ && stamp > last_target_stamp_) {
                const double dt = (stamp - last_target_stamp_).seconds();
                if (dt > std::numeric_limits<double>::epsilon()) {
                    ddq = (dq - prev_target_dq_) / dt;
                    for (int i = 0; i < kDof; ++i) {
                        ddq[i] = std::clamp(ddq[i], -max_abs_acceleration_, max_abs_acceleration_);

                        const double lower = angle_window_lower_[static_cast<size_t>(i)];
                        const double upper = angle_window_upper_[static_cast<size_t>(i)];
                        const bool at_lower_limit = std::abs(q[i] - lower) < 1e-9;
                        const bool at_upper_limit = std::abs(q[i] - upper) < 1e-9;
                        if ((at_lower_limit && ddq[i] < 0.0) || (at_upper_limit && ddq[i] > 0.0)) {
                            ddq[i] = 0.0;
                        }
                    }
                }
            }

            target_q_ = q;
            target_dq_ = dq;
            target_ddq_ = ddq;
            last_continuous_q_ = continuous_q;
            prev_target_dq_ = dq;
            last_target_stamp_ = stamp;
            has_target_ = true;
        }

        if (log_input_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                200,
                "Teach target q=%s dq=%s ddq=%s",
                format_vector(q).c_str(),
                format_vector(dq).c_str(),
                format_vector(ddq).c_str());
        }
    }

    void publish_target()
    {
        if (!has_target_) {
            return;
        }

        Eigen::VectorXd q(kDof);
        Eigen::VectorXd dq(kDof);
        Eigen::VectorXd ddq(kDof);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q = target_q_;
            dq = target_dq_;
            ddq = target_ddq_;
        }

        robot_msgs::msg::RobotState target_msg;
        target_msg.motor_state.resize(kDof);
        for (int i = 0; i < kDof; ++i) {
            auto & motor = target_msg.motor_state[static_cast<size_t>(i)];
            motor.q = static_cast<float>(q[i]);
            motor.dq = static_cast<float>(dq[i]);
            motor.ddq = static_cast<float>(ddq[i]);
            motor.tau_est = 0.0f;
            motor.cur = 0.0f;
            motor.valid = true;
        }

        target_pub_->publish(target_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Published teach target q=%s dq=%s ddq=%s -> %s",
            format_vector(q).c_str(),
            format_vector(dq).c_str(),
            format_vector(ddq).c_str(),
            target_topic_.c_str());
    }

    void request_teach_mode_if_needed()
    {
        if (!request_mode_on_startup_ || teach_mode_requested_) {
            return;
        }

        if (!mode_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for mode service %s before enabling teach pendant mode.",
                mode_service_name_.c_str());
            return;
        }

        auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
        request->mode = mode_name_;
        mode_client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedFuture future) {
                const auto response = future.get();
                if (!response) {
                    return;
                }
                if (response->success) {
                    teach_mode_requested_ = true;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Teach pendant mode enabled via %s: %s",
                        mode_service_name_.c_str(),
                        response->message.c_str());
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Teach pendant mode request rejected: %s",
                        response->message.c_str());
                }
            });
    }

    std::mutex data_mutex_;
    bool has_target_{false};
    bool teach_mode_requested_{false};
    bool request_mode_on_startup_{true};
    bool log_input_{false};
    double max_abs_acceleration_{30.0};
    double republish_hz_{50.0};
    int teacher_qos_depth_{10};
    int target_qos_depth_{10};
    std::string teacher_topic_;
    std::string target_topic_;
    std::string mode_service_name_;
    std::string mode_name_;
    std::vector<std::string> expected_joint_names_;
    std::vector<double> angle_window_lower_;
    std::vector<double> angle_window_upper_;
    Eigen::VectorXd target_q_;
    Eigen::VectorXd target_dq_;
    Eigen::VectorXd target_ddq_;
    Eigen::VectorXd last_continuous_q_{Eigen::VectorXd::Zero(kDof)};
    Eigen::VectorXd prev_target_dq_;
    rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr teacher_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr target_pub_;
    rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
    rclcpp::TimerBase::SharedPtr republish_timer_;
    rclcpp::TimerBase::SharedPtr mode_request_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeachPendantFollowNode>());
    rclcpp::shutdown();
    return 0;
}
