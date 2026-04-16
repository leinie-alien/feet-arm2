#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/set_payload_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "arm2_task/dynamics_manager.hpp"
#include "arm2_task/kinematics_engine.hpp"

using namespace std::chrono_literals;

class InverseDynamicsNode : public rclcpp::Node
{
public:
    InverseDynamicsNode() : Node("inverse_dynamics_node")
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
        std::string urdf = share_dir + "/" + rel_urdf_path;
        double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
        double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
        double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
        double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

        state_topic_ =
            this->declare_parameter("inverse_dynamics.state_topic", "/arm2/_lowState/joint");
        target_topic_ =
            this->declare_parameter("inverse_dynamics.target_topic", "/joint_target_state");
        command_topic_ =
            this->declare_parameter("inverse_dynamics.command_topic", "/arm2/_lowCmd/command");
        ready_topic_ =
            this->declare_parameter("inverse_dynamics.ready_topic", "/robot_driver/ready");
        mode_service_name_ =
            this->declare_parameter("inverse_dynamics.mode_service", "set_controller_mode");
        payload_service_name_ =
            this->declare_parameter("inverse_dynamics.payload_service", "set_payload_state");
        control_rate_hz_ = this->declare_parameter("inverse_dynamics.control_rate_hz", 100.0);
        hold_position_on_startup_ =
            this->declare_parameter("inverse_dynamics.hold_position_on_startup", true);
        default_mode_ =
            this->declare_parameter("inverse_dynamics.default_mode", std::string("moving"));
        target_timeout_sec_ =
            this->declare_parameter("inverse_dynamics.target_timeout_sec", 0.25);
        target_qos_depth_ = this->declare_parameter("inverse_dynamics.target_qos_depth", 10);
        log_input_ = this->declare_parameter("inverse_dynamics.log_input", false);

        auto fc = this->declare_parameter("dynamics.friction.fc", std::vector<double>(kDof, 0.0));
        auto fv = this->declare_parameter("dynamics.friction.fv", std::vector<double>(kDof, 0.0));
        auto ratios = this->declare_parameter(
            "dynamics.friction.GearRatio", std::vector<double>(kDof, 1.0));
        auto alpha = this->declare_parameter("dynamics.friction.alpha", 100.0);
        angle_window_lower_ = this->declare_parameter(
            "inverse_dynamics.angle_window_lower",
            std::vector<double>{-M_PI, -0.5, -3.0, -2.0, -M_PI});
        angle_window_upper_ = this->declare_parameter(
            "inverse_dynamics.angle_window_upper",
            std::vector<double>{M_PI, 3.5, 0.15, 1.57, M_PI});
        command_velocity_limits_ = this->declare_parameter(
            "inverse_dynamics.command_velocity_limits",
            std::vector<double>{2.0, 1.5, 2.0, 2.0, 3.0});
        command_torque_limits_ = this->declare_parameter(
            "inverse_dynamics.command_torque_limits",
            std::vector<double>{8.0, 20.0, 8.0, 8.0, 4.0});

        normalize_vector_param(fc, kDof, 0.0, "dynamics.friction.fc");
        normalize_vector_param(fv, kDof, 0.0, "dynamics.friction.fv");
        normalize_vector_param(ratios, kDof, 1.0, "dynamics.friction.GearRatio");
        normalize_vector_param(
            angle_window_lower_, kDof, -M_PI, "inverse_dynamics.angle_window_lower");
        normalize_vector_param(
            angle_window_upper_, kDof, M_PI, "inverse_dynamics.angle_window_upper");
        normalize_vector_param(
            command_velocity_limits_,
            kDof,
            1.0,
            "inverse_dynamics.command_velocity_limits");
        normalize_vector_param(
            command_torque_limits_,
            kDof,
            1.0,
            "inverse_dynamics.command_torque_limits");
        normalize_angle_windows();
        normalize_positive_limits(
            command_velocity_limits_, 1.0, "inverse_dynamics.command_velocity_limits");
        normalize_positive_limits(
            command_torque_limits_, 1.0, "inverse_dynamics.command_torque_limits");

        if (control_rate_hz_ < 1.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "inverse_dynamics.control_rate_hz=%.3f is too small; clamping to 1.0 Hz.",
                control_rate_hz_);
            control_rate_hz_ = 1.0;
        }
        if (target_timeout_sec_ < 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "inverse_dynamics.target_timeout_sec=%.3f is invalid; clamping to 0.0.",
                target_timeout_sec_);
            target_timeout_sec_ = 0.0;
        }
        if (target_qos_depth_ < 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "inverse_dynamics.target_qos_depth=%d is invalid; clamping to 1.",
                target_qos_depth_);
            target_qos_depth_ = 1;
        }

        kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(
            urdf,
            arm2_task::RobotGeometry(l1, l2, l3, l4));
        dyn_manager_ = std::make_unique<arm2_task::DynamicsManager>(urdf);
        dyn_manager_->initParams(fc, fv, ratios, alpha);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        current_q_ = Eigen::VectorXd::Zero(kDof);
        current_dq_ = Eigen::VectorXd::Zero(kDof);
        current_tau_est_ = Eigen::VectorXd::Zero(kDof);
        desired_state_ = arm2_task::JointState(kDof);
        last_continuous_target_q_ = Eigen::VectorXd::Zero(kDof);
        last_published_tau_ff_ = Eigen::VectorXd::Zero(kDof);
        last_target_update_time_ = std::chrono::steady_clock::now();

        load_all_gains();
        if (gains_map_.count(default_mode_) == 0U) {
            RCLCPP_WARN(
                this->get_logger(),
                "Default mode '%s' not found in gains.* parameters. Falling back to 'moving'.",
                default_mode_.c_str());
            default_mode_ = "moving";
        }
        current_mode_ = default_mode_;
        current_gains_ = gains_map_[current_mode_];
        publish_static_camera_tf();

        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        command_pub_ =
            this->create_publisher<robot_msgs::msg::RobotCommand>(command_topic_, state_qos);

        measured_state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            state_qos,
            std::bind(&InverseDynamicsNode::measured_state_callback, this, std::placeholders::_1));

        target_state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            target_topic_,
            target_qos,
            std::bind(&InverseDynamicsNode::target_state_callback, this, std::placeholders::_1));

        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&InverseDynamicsNode::driver_ready_callback, this, std::placeholders::_1));

        mode_service_ = this->create_service<robot_msgs::srv::SetControllerMode>(
            mode_service_name_,
            std::bind(
                &InverseDynamicsNode::handle_mode_change,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        payload_service_ = this->create_service<robot_msgs::srv::SetPayloadState>(
            payload_service_name_,
            std::bind(
                &InverseDynamicsNode::handle_payload_state,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&InverseDynamicsNode::control_loop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Inverse dynamics node started. target_topic=%s state_topic=%s command_topic=%s ready_topic=%s mode=%s payload_service=%s rate=%.1fHz target_timeout=%.3fs target_qos=reliable/%d hold_startup=%s",
            target_topic_.c_str(),
            state_topic_.c_str(),
            command_topic_.c_str(),
            ready_topic_.c_str(),
            current_mode_.c_str(),
            payload_service_name_.c_str(),
            control_rate_hz_,
            target_timeout_sec_,
            target_qos_depth_,
            hold_position_on_startup_ ? "true" : "false");
    }

private:
    static constexpr int kDof = 5;
    static constexpr double kAngleWrapLimit = M_PI;
    static constexpr double kFullTurnRad = 2.0 * M_PI;

    struct PDGains
    {
        std::vector<double> kp;
        std::vector<double> kd;
    };

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

    static double shortest_angular_distance(double from, double to)
    {
        return normalize_angle_to_pi(to - from);
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

    void normalize_vector_param(
        std::vector<double> & values,
        size_t expected_size,
        double fallback,
        const char * name)
    {
        if (values.size() == expected_size) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Parameter %s has size %zu, expected %zu. Using fallback %.3f.",
            name,
            values.size(),
            expected_size,
            fallback);
        values.assign(expected_size, fallback);
    }

    void normalize_positive_limits(
        std::vector<double> & values,
        double fallback,
        const char * name)
    {
        for (double & value : values) {
            if (!std::isfinite(value) || value <= 0.0) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Parameter %s contains invalid limit %.3f. Using fallback %.3f.",
                    name,
                    value,
                    fallback);
                value = fallback;
            }
        }
    }

    void normalize_angle_windows()
    {
        const std::array<double, kDof> default_lower = {-M_PI, -0.5, -3.0, -2.0, -M_PI};
        const std::array<double, kDof> default_upper = {M_PI, 3.5, 0.15, 1.57, M_PI};

        for (int i = 0; i < kDof; ++i) {
            if (!std::isfinite(angle_window_lower_[static_cast<size_t>(i)]) ||
                !std::isfinite(angle_window_upper_[static_cast<size_t>(i)]) ||
                angle_window_lower_[static_cast<size_t>(i)] >=
                    angle_window_upper_[static_cast<size_t>(i)])
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Inverse dynamics angle window for joint %d is invalid. Resetting to defaults.",
                    i + 1);
                angle_window_lower_[static_cast<size_t>(i)] = default_lower[static_cast<size_t>(i)];
                angle_window_upper_[static_cast<size_t>(i)] = default_upper[static_cast<size_t>(i)];
            }
        }
    }

    void load_all_gains()
    {
        const std::vector<std::string> modes = {
            "idle",
            "gravity_comp",
            "moving",
            "loaded",
            "teach_pendant",
            "teach_drag"};
        std::unordered_set<std::string> loaded;
        for (const auto & mode : modes) {
            PDGains gains;
            gains.kp = this->declare_parameter(
                "gains." + mode + ".kp", std::vector<double>(kDof, 0.0));
            gains.kd = this->declare_parameter(
                "gains." + mode + ".kd", std::vector<double>(kDof, 0.0));
            normalize_vector_param(gains.kp, kDof, 0.0, ("gains." + mode + ".kp").c_str());
            normalize_vector_param(gains.kd, kDof, 0.0, ("gains." + mode + ".kd").c_str());
            gains_map_[mode] = gains;
            loaded.insert(mode);
        }

        if (loaded.count("moving") == 0U) {
            gains_map_["moving"] =
                PDGains{std::vector<double>(kDof, 0.0), std::vector<double>(kDof, 0.0)};
        }
    }

    bool extract_joint_state(
        const robot_msgs::msg::RobotState::SharedPtr & msg,
        arm2_task::JointState & out_state,
        bool require_valid_flag,
        const char * source_name)
    {
        if (!msg || msg->motor_state.size() < static_cast<size_t>(kDof)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Ignore %s message: expected at least %d joints, got %zu.",
                source_name,
                kDof,
                msg ? msg->motor_state.size() : 0UL);
            return false;
        }

        for (int i = 0; i < kDof; ++i) {
            const auto & motor = msg->motor_state[static_cast<size_t>(i)];
            if (require_valid_flag && !motor.valid) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Ignore %s message: joint %d is marked invalid.",
                    source_name,
                    i);
                return false;
            }
            if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) || !std::isfinite(motor.ddq) ||
                !std::isfinite(motor.tau_est))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Ignore %s message: joint %d contains NaN/Inf.",
                    source_name,
                    i);
                return false;
            }

            out_state.q[i] = motor.q;
            out_state.dq[i] = motor.dq;
            out_state.ddq[i] = motor.ddq;
            out_state.tau[i] = motor.tau_est;
        }
        return true;
    }

    double unwrap_continuous_angle(double raw_angle, bool has_reference, double reference) const
    {
        const double wrapped = normalize_angle_to_pi(raw_angle);
        if (!has_reference || !std::isfinite(reference)) {
            return wrapped;
        }

        const double reference_wrapped = normalize_angle_to_pi(reference);
        const double delta = shortest_angular_distance(reference_wrapped, wrapped);
        return reference + delta;
    }

    double choose_windowed_angle(int joint_index, double continuous_angle) const
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

        return std::clamp(*best_it, lower, upper);
    }

    void measured_state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        arm2_task::JointState measured_state(kDof);
        if (!extract_joint_state(msg, measured_state, true, "measured_state")) {
            return;
        }

        Eigen::VectorXd q_snapshot = measured_state.q;
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_q_ = measured_state.q;
        current_dq_ = measured_state.dq;
        current_tau_est_ = measured_state.tau;
        has_measured_state_ = true;

        if (hold_position_on_startup_ && !has_target_state_) {
            desired_state_.q = current_q_;
            desired_state_.dq.setZero();
            desired_state_.ddq.setZero();
            has_target_state_ = true;
            target_initialized_from_feedback_ = true;
            last_continuous_target_q_ = current_q_;
            RCLCPP_INFO(
                this->get_logger(),
                "Initialized desired state from measured joint state so the node can hold position.");
        }

        // Mirror control_node: publish world -> Link_4 from the latest measured joint state.
        pinocchio::SE3 T_w_l4 = kin_engine_->forwardKinematics(q_snapshot);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "Link_4";
        t.transform.translation.x = T_w_l4.translation()(0);
        t.transform.translation.y = T_w_l4.translation()(1);
        t.transform.translation.z = T_w_l4.translation()(2);
        Eigen::Quaterniond q_rot(T_w_l4.rotation());
        t.transform.rotation.x = q_rot.x();
        t.transform.rotation.y = q_rot.y();
        t.transform.rotation.z = q_rot.z();
        t.transform.rotation.w = q_rot.w();
        tf_broadcaster_->sendTransform(t);
    }

    void target_state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        arm2_task::JointState target_state(kDof);
        if (!extract_joint_state(msg, target_state, false, "target_state")) {
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < kDof; ++i) {
            const bool has_reference = has_target_state_ || target_initialized_from_feedback_;
            const double continuous_angle = unwrap_continuous_angle(
                target_state.q[i],
                has_reference,
                last_continuous_target_q_[i]);
            last_continuous_target_q_[i] = continuous_angle;
            target_state.q[i] = choose_windowed_angle(i, continuous_angle);

            const double lower = angle_window_lower_[static_cast<size_t>(i)];
            const double upper = angle_window_upper_[static_cast<size_t>(i)];
            const bool at_lower_limit = std::abs(target_state.q[i] - lower) < 1e-9;
            const bool at_upper_limit = std::abs(target_state.q[i] - upper) < 1e-9;
            if ((at_lower_limit && target_state.dq[i] < 0.0) ||
                (at_upper_limit && target_state.dq[i] > 0.0))
            {
                target_state.dq[i] = 0.0;
                target_state.ddq[i] = 0.0;
            }
        }

        desired_state_ = target_state;
        has_target_state_ = true;
        target_initialized_from_feedback_ = false;
        last_target_update_time_ = std::chrono::steady_clock::now();
        target_timeout_latched_ = false;
        ++target_update_count_;
    }

    void driver_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
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

    void handle_mode_change(
        const std::shared_ptr<robot_msgs::srv::SetControllerMode::Request> request,
        std::shared_ptr<robot_msgs::srv::SetControllerMode::Response> response)
    {
        const auto it = gains_map_.find(request->mode);
        if (it == gains_map_.end()) {
            response->success = false;
            response->message = "Unknown mode: " + request->mode;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_mode_ = request->mode;
            current_gains_ = it->second;
        }

        response->success = true;
        response->message = "Mode switched to: " + request->mode;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void handle_payload_state(
        const std::shared_ptr<robot_msgs::srv::SetPayloadState::Request> request,
        std::shared_ptr<robot_msgs::srv::SetPayloadState::Response> response)
    {
        if (!std::isfinite(request->mass) || request->mass < 0.0) {
            response->success = false;
            response->message = "Invalid payload mass.";
            return;
        }

        Eigen::Vector3d com(request->com[0], request->com[1], request->com[2]);
        if (!std::isfinite(com[0]) || !std::isfinite(com[1]) || !std::isfinite(com[2])) {
            response->success = false;
            response->message = "Invalid payload COM.";
            return;
        }

        dyn_manager_->setPayloadState(request->has_load, request->mass, com);
        response->success = true;
        response->message = request->has_load ? "Payload model enabled." : "Payload model cleared.";
        RCLCPP_INFO(
            this->get_logger(),
            "Payload state updated: has_load=%s mass=%.4f com=[%.4f, %.4f, %.4f]",
            request->has_load ? "true" : "false",
            request->mass,
            com[0],
            com[1],
            com[2]);
    }

    bool is_teach_drag_mode(const std::string & mode_name) const
    {
        return mode_name == "teach_drag";
    }

    bool build_teach_drag_state(
        arm2_task::JointState & desired,
        Eigen::VectorXd & tau_ff,
        bool & target_from_feedback)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_measured_state_) {
            return false;
        }

        desired.q = current_q_;
        desired.dq = Eigen::VectorXd::Zero(kDof);
        desired.ddq = Eigen::VectorXd::Zero(kDof);

        tau_ff =
            dyn_manager_->computeInverseDynamics(desired) + dyn_manager_->computeFriction(current_dq_);
        target_from_feedback = true;
        return true;
    }

    bool build_tracked_target_state(
        arm2_task::JointState & desired,
        Eigen::VectorXd & tau_ff,
        bool & target_from_feedback,
        const std::string & mode_name)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_measured_state_) {
            return false;
        }

        if (!has_target_state_) {
            if (!hold_position_on_startup_) {
                return false;
            }

            desired.q = current_q_;
            desired.dq = Eigen::VectorXd::Zero(kDof);
            desired.ddq = Eigen::VectorXd::Zero(kDof);
            target_from_feedback = true;
        } else {
            desired = desired_state_;
            target_from_feedback = target_initialized_from_feedback_;
        }

        if (target_timeout_sec_ > 0.0 && has_target_state_) {
            const double target_age_sec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_target_update_time_).count();
            if (target_age_sec > target_timeout_sec_) {
                desired.q = current_q_;
                desired.dq = Eigen::VectorXd::Zero(kDof);
                desired.ddq = Eigen::VectorXd::Zero(kDof);
                target_from_feedback = true;
                if (!target_timeout_latched_) {
                    target_timeout_latched_ = true;
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Inverse dynamics target timed out after %.3fs in mode %s. Falling back to measured hold.",
                        target_age_sec,
                        mode_name.c_str());
                }
            }
        }

        tau_ff = dyn_manager_->getFeedForwardTorque(desired);
        return true;
    }

    void enforce_command_limits(
        Eigen::VectorXd & q_cmd,
        Eigen::VectorXd & dq_cmd,
        Eigen::VectorXd & ddq_cmd,
        Eigen::VectorXd & tau_cmd) const
    {
        for (int i = 0; i < kDof; ++i) {
            const double lower = angle_window_lower_[static_cast<size_t>(i)];
            const double upper = angle_window_upper_[static_cast<size_t>(i)];
            q_cmd[i] = std::clamp(q_cmd[i], lower, upper);
            dq_cmd[i] = std::clamp(
                dq_cmd[i],
                -command_velocity_limits_[static_cast<size_t>(i)],
                command_velocity_limits_[static_cast<size_t>(i)]);
            ddq_cmd[i] = std::clamp(
                ddq_cmd[i],
                -2.0 * command_velocity_limits_[static_cast<size_t>(i)],
                2.0 * command_velocity_limits_[static_cast<size_t>(i)]);
            tau_cmd[i] = std::clamp(
                tau_cmd[i],
                -command_torque_limits_[static_cast<size_t>(i)],
                command_torque_limits_[static_cast<size_t>(i)]);

            const bool at_lower_limit = std::abs(q_cmd[i] - lower) < 1e-9;
            const bool at_upper_limit = std::abs(q_cmd[i] - upper) < 1e-9;
            if ((at_lower_limit && dq_cmd[i] < 0.0) || (at_upper_limit && dq_cmd[i] > 0.0)) {
                dq_cmd[i] = 0.0;
                ddq_cmd[i] = 0.0;
            }
        }
    }

    void control_loop()
    {
        if (!driver_ready_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Inverse dynamics waiting for /robot_driver/ready.");
            return;
        }

        std::string mode_name;
        PDGains gains;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            mode_name = current_mode_;
            gains = current_gains_;
        }

        arm2_task::JointState desired(kDof);
        Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(kDof);
        bool target_from_feedback = false;
        const bool valid_state = is_teach_drag_mode(mode_name)
                                     ? build_teach_drag_state(desired, tau_ff, target_from_feedback)
                                     : build_tracked_target_state(desired, tau_ff, target_from_feedback, mode_name);
        if (!valid_state) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Inverse dynamics waiting for valid measured/target state in mode %s.",
                mode_name.c_str());
            return;
        }

        Eigen::VectorXd q_cmd = desired.q;
        Eigen::VectorXd dq_cmd = desired.dq;
        Eigen::VectorXd ddq_cmd = desired.ddq;
        Eigen::VectorXd tau_cmd = tau_ff;
        enforce_command_limits(q_cmd, dq_cmd, ddq_cmd, tau_cmd);

        robot_msgs::msg::RobotCommand cmd_msg;
        cmd_msg.motor_command.reserve(kDof);
        for (int i = 0; i < kDof; ++i) {
            robot_msgs::msg::MotorCommand cmd;
            cmd.q = static_cast<float>(q_cmd[i]);
            cmd.dq = static_cast<float>(dq_cmd[i]);
            cmd.tau = static_cast<float>(tau_cmd[i]);
            cmd.kp = static_cast<float>(gains.kp[static_cast<size_t>(i)]);
            cmd.kd = static_cast<float>(gains.kd[static_cast<size_t>(i)]);
            cmd_msg.motor_command.push_back(cmd);
        }

        command_pub_->publish(cmd_msg);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_published_tau_ff_ = tau_cmd;
            ++command_publish_count_;
        }

        if (log_input_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Inverse dynamics mode=%s target_from_feedback=%s q=%s dq=%s ddq=%s tau=%s",
                mode_name.c_str(),
                target_from_feedback ? "true" : "false",
                format_vector(q_cmd).c_str(),
                format_vector(dq_cmd).c_str(),
                format_vector(ddq_cmd).c_str(),
                format_vector(tau_cmd).c_str());
        }
    }

    std::mutex data_mutex_;
    bool driver_ready_{false};
    bool has_measured_state_{false};
    bool has_target_state_{false};
    bool target_initialized_from_feedback_{false};
    bool target_timeout_latched_{false};
    bool log_input_{false};
    std::string current_mode_;
    PDGains current_gains_;
    std::unordered_map<std::string, PDGains> gains_map_;
    Eigen::VectorXd current_q_;
    Eigen::VectorXd current_dq_;
    Eigen::VectorXd current_tau_est_;
    arm2_task::JointState desired_state_;
    Eigen::VectorXd last_continuous_target_q_;
    Eigen::VectorXd last_published_tau_ff_;
    uint64_t target_update_count_{0};
    uint64_t command_publish_count_{0};
    std::chrono::steady_clock::time_point last_target_update_time_{};

    std::string state_topic_;
    std::string target_topic_;
    std::string command_topic_;
    std::string ready_topic_;
    std::string mode_service_name_;
    std::string payload_service_name_;
    std::string default_mode_;
    double control_rate_hz_{100.0};
    double target_timeout_sec_{0.25};
    int target_qos_depth_{10};
    bool hold_position_on_startup_{true};
    std::vector<double> angle_window_lower_;
    std::vector<double> angle_window_upper_;
    std::vector<double> command_velocity_limits_;
    std::vector<double> command_torque_limits_;

    std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;
    std::unique_ptr<arm2_task::DynamicsManager> dyn_manager_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr measured_state_sub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr target_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Service<robot_msgs::srv::SetControllerMode>::SharedPtr mode_service_;
    rclcpp::Service<robot_msgs::srv::SetPayloadState>::SharedPtr payload_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void publish_static_camera_tf();
};

void InverseDynamicsNode::publish_static_camera_tf()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    const auto parent_frame =
        this->declare_parameter("camera_extrinsics.parent_frame", std::string("Link_4"));
    const auto child_frame =
        this->declare_parameter("camera_extrinsics.child_frame", std::string("camera_link"));
    auto pos =
        this->declare_parameter("camera_extrinsics.pos", std::vector<double>{0.0, -0.07, 0.0});
    auto quat =
        this->declare_parameter(
            "camera_extrinsics.quat",
            std::vector<double>{0.0, 0.70710678, 0.0, 0.70710678});
    if (pos.size() != 3U) {
        RCLCPP_ERROR(
            this->get_logger(),
            "camera_extrinsics.pos size=%zu, expected 3. Falling back to [0.0, -0.07, 0.0].",
            pos.size());
        pos = {0.0, -0.07, 0.0};
    }
    if (quat.size() != 4U) {
        RCLCPP_ERROR(
            this->get_logger(),
            "camera_extrinsics.quat size=%zu, expected 4. Falling back to [0.0, 0.70710678, 0.0, 0.70710678].",
            quat.size());
        quat = {0.0, 0.70710678, 0.0, 0.70710678};
    }
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;
    t.transform.translation.x = pos[0];
    t.transform.translation.y = pos[1];
    t.transform.translation.z = pos[2];
    t.transform.rotation.x = quat[0];
    t.transform.rotation.y = quat[1];
    t.transform.rotation.z = quat[2];
    t.transform.rotation.w = quat[3];
    static_tf_broadcaster_->sendTransform(t);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseDynamicsNode>());
    rclcpp::shutdown();
    return 0;
}
