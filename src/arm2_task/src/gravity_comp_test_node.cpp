#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/set_payload_state.hpp"

#include "arm2_task/dynamics_manager.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class GravityCompTestNode : public rclcpp::Node
{
public:
    GravityCompTestNode() : Node("gravity_comp_test_node")
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
        std::string urdf = share_dir + "/" + rel_urdf_path;

        auto fc = this->declare_parameter("dynamics.friction.fc", std::vector<double>(5, 0.0));
        auto fv = this->declare_parameter("dynamics.friction.fv", std::vector<double>(5, 0.0));
        auto ratios = this->declare_parameter("dynamics.friction.GearRatio", std::vector<double>(5, 1.0));
        auto alpha = this->declare_parameter("dynamics.friction.alpha", 100.0);
        gains_mode_ =
            this->declare_parameter("gravity_comp_test.gains_mode", std::string("gravity_comp"));
        kp_ = this->declare_parameter("gains." + gains_mode_ + ".kp", std::vector<double>(5, 0.0));
        kd_ = this->declare_parameter("gains." + gains_mode_ + ".kd", std::vector<double>(5, 0.0));
        enable_payload_service_ =
            this->declare_parameter("gravity_comp_test.enable_payload_service", false);
        payload_service_name_ =
            this->declare_parameter("gravity_comp_test.payload_service", "set_payload_state");

        dyn_manager_ = std::make_unique<arm2_task::DynamicsManager>(urdf);
        dyn_manager_->initParams(fc, fv, ratios, alpha);

        current_q_ = Eigen::VectorXd::Zero(5);
        current_dq_ = Eigen::VectorXd::Zero(5);
        current_tau_est_ = Eigen::VectorXd::Zero(5);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        command_pub_ = this->create_publisher<robot_msgs::msg::RobotCommand>("/arm2/_lowCmd/command", qos);

        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "/arm2/_lowState/joint",
            qos,
            std::bind(&GravityCompTestNode::state_callback, this, std::placeholders::_1));

        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot_driver/ready",
            ready_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg) {
                    if (driver_ready_ != msg->data) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Driver ready changed: %s -> %s",
                            driver_ready_ ? "true" : "false",
                            msg->data ? "true" : "false");
                        if (!msg->data && has_data_) {
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            RCLCPP_WARN(
                                this->get_logger(),
                                "Driver dropped not-ready while holding state. joint2 snapshot: q=%.3f dq=%.3f tau_est=%.3f last_cmd_q=%.3f last_cmd_tau=%.3f last_tau_g=%.3f",
                                current_q_[1],
                                current_dq_[1],
                                current_tau_est_[1],
                                last_command_q_[1],
                                last_command_tau_[1],
                                last_tau_g_[1]);
                        }
                    }
                    driver_ready_ = msg->data;
                }
            });

        if (enable_payload_service_) {
            payload_service_ = this->create_service<robot_msgs::srv::SetPayloadState>(
                payload_service_name_,
                std::bind(
                    &GravityCompTestNode::handle_payload_state,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

        timer_ = this->create_wall_timer(10ms, std::bind(&GravityCompTestNode::control_loop, this));
        debug_timer_ = this->create_wall_timer(1s, std::bind(&GravityCompTestNode::debug_summary, this));

        RCLCPP_INFO(this->get_logger(), "Gravity compensation test node started.");
        RCLCPP_INFO(
            this->get_logger(),
            "Gravity compensation PD gains (%s): kp=%s, kd=%s",
            gains_mode_.c_str(),
            format_std_vector(kp_).c_str(),
            format_std_vector(kd_).c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Gravity compensation payload service: %s%s",
            enable_payload_service_ ? "enabled on " : "disabled",
            enable_payload_service_ ? payload_service_name_.c_str() : "");
    }

private:
    static std::string format_std_vector(const std::vector<double>& values)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i) {
            if (i != 0) {
                oss << ", ";
            }
            oss << std::fixed << std::setprecision(3) << values[i];
        }
        oss << "]";
        return oss.str();
    }

    static std::string format_eigen_vector(const Eigen::VectorXd& values)
    {
        std::ostringstream oss;
        oss << "[";
        for (Eigen::Index i = 0; i < values.size(); ++i) {
            if (i != 0) {
                oss << ", ";
            }
            oss << std::fixed << std::setprecision(3) << values[i];
        }
        oss << "]";
        return oss.str();
    }

    static std::string format_valid_mask(const robot_msgs::msg::RobotState::SharedPtr& msg)
    {
        std::ostringstream oss;
        oss << "[";
        if (msg) {
            for (size_t i = 0; i < msg->motor_state.size(); ++i) {
                if (i != 0) {
                    oss << ", ";
                }
                oss << (msg->motor_state[i].valid ? '1' : '0');
            }
        }
        oss << "]";
        return oss.str();
    }

    static std::string format_state_field(
        const robot_msgs::msg::RobotState::SharedPtr& msg,
        float robot_msgs::msg::MotorState::* field)
    {
        std::ostringstream oss;
        oss << "[";
        if (msg) {
            for (size_t i = 0; i < msg->motor_state.size(); ++i) {
                if (i != 0) {
                    oss << ", ";
                }
                oss << std::fixed << std::setprecision(3) << (msg->motor_state[i].*field);
            }
        }
        oss << "]";
        return oss.str();
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        ++state_message_count_;
        if (!msg || msg->motor_state.size() < 5) {
            ++short_state_count_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Ignore short/invalid robot state message. size=%zu",
                msg ? msg->motor_state.size() : 0UL);
            return;
        }
        for (int i = 0; i < 5; ++i) {
            if (!msg->motor_state[i].valid) {
                ++invalid_state_count_;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Ignore stale/invalid robot state message. valid_mask=%s q=%s dq=%s tau_est=%s",
                    format_valid_mask(msg).c_str(),
                    format_state_field(msg, &robot_msgs::msg::MotorState::q).c_str(),
                    format_state_field(msg, &robot_msgs::msg::MotorState::dq).c_str(),
                    format_state_field(msg, &robot_msgs::msg::MotorState::tau_est).c_str());
                return;
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < 5; ++i) {
            current_q_[i] = msg->motor_state[i].q;
            current_dq_[i] = msg->motor_state[i].dq;
            current_tau_est_[i] = msg->motor_state[i].tau_est;
        }
        has_data_ = true;
        ++accepted_state_count_;
        last_valid_state_time_ = this->now();
    }

    void control_loop()
    {
        if (!driver_ready_ || !has_data_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Gravity compensation waiting: driver_ready=%s has_data=%s",
                driver_ready_ ? "true" : "false",
                has_data_ ? "true" : "false");
            return;
        }

        Eigen::VectorXd q(5);
        Eigen::VectorXd dq(5);
        Eigen::VectorXd tau_est(5);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q = current_q_;
            dq = current_dq_;
            tau_est = current_tau_est_;
        }

        arm2_task::JointState des_state(5);
        des_state.q = q;
        des_state.dq = Eigen::VectorXd::Zero(5);
        des_state.ddq = Eigen::VectorXd::Zero(5);

        const Eigen::VectorXd tau_g = dyn_manager_->computeInverseDynamics(des_state);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_tau_g_ = tau_g;
            last_published_q_ = q;
            last_command_q_ = q;
            last_command_tau_ = tau_g;
            ++command_publish_count_;
        }

        const double joint2_tau_sign_product = tau_g[1] * tau_est[1];
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Gravity compensation status: q=%s dq=%s tau_g=%s tau_est=%s | joint2: q=%.3f dq=%.3f tau_g=%.3f tau_est=%.3f cmd_q=%.3f cmd_tau=%.3f q_err=%.3f tau_sign=%s kp=%.3f kd=%.3f",
            format_eigen_vector(q).c_str(),
            format_eigen_vector(dq).c_str(),
            format_eigen_vector(tau_g).c_str(),
            format_eigen_vector(tau_est).c_str(),
            q[1],
            dq[1],
            tau_g[1],
            tau_est[1],
            q[1],
            tau_g[1],
            0.0,
            joint2_tau_sign_product > 0.0 ? "same" : (joint2_tau_sign_product < 0.0 ? "opposite" : "zero"),
            kp_[1],
            kd_[1]);

        robot_msgs::msg::RobotCommand msg;
        msg.motor_command.reserve(5);
        for (int i = 0; i < 5; ++i) {
            robot_msgs::msg::MotorCommand cmd;
            cmd.q = static_cast<float>(q[i]);
            cmd.dq = 0.0f;
            cmd.tau = static_cast<float>(tau_g[i]);
            cmd.kp = static_cast<float>(kp_[i]);
            cmd.kd = static_cast<float>(kd_[i]);
            msg.motor_command.push_back(cmd);
        }

        command_pub_->publish(msg);
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
            "Gravity compensation payload updated: has_load=%s mass=%.4f com=[%.4f, %.4f, %.4f]",
            request->has_load ? "true" : "false",
            request->mass,
            com[0],
            com[1],
            com[2]);
    }

    void debug_summary()
    {
        const rclcpp::Time now = this->now();
        const double state_age_sec =
            accepted_state_count_ == 0 ? -1.0 : (now - last_valid_state_time_).seconds();

        Eigen::VectorXd q_snapshot(5);
        Eigen::VectorXd dq_snapshot(5);
        Eigen::VectorXd tau_snapshot(5);
        Eigen::VectorXd tau_est_snapshot(5);
        Eigen::VectorXd cmd_q_snapshot(5);
        Eigen::VectorXd cmd_tau_snapshot(5);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q_snapshot = current_q_;
            dq_snapshot = current_dq_;
            tau_snapshot = last_tau_g_;
            tau_est_snapshot = current_tau_est_;
            cmd_q_snapshot = last_command_q_;
            cmd_tau_snapshot = last_command_tau_;
        }

        const double joint2_tau_sign_product = cmd_tau_snapshot[1] * tau_est_snapshot[1];

        RCLCPP_INFO(
            this->get_logger(),
            "Gravity debug summary: ready=%s has_data=%s state_msgs=%llu accepted=%llu invalid=%llu short=%llu cmd_pub=%llu state_age=%.3f q2=%.3f dq2=%.3f tau_g2=%.3f tau_est2=%.3f cmd_q2=%.3f cmd_tau2=%.3f tau_sign=%s q=%s tau_g=%s tau_est=%s",
            driver_ready_ ? "true" : "false",
            has_data_ ? "true" : "false",
            static_cast<unsigned long long>(state_message_count_),
            static_cast<unsigned long long>(accepted_state_count_),
            static_cast<unsigned long long>(invalid_state_count_),
            static_cast<unsigned long long>(short_state_count_),
            static_cast<unsigned long long>(command_publish_count_),
            state_age_sec,
            q_snapshot[1],
            dq_snapshot[1],
            tau_snapshot[1],
            tau_est_snapshot[1],
            cmd_q_snapshot[1],
            cmd_tau_snapshot[1],
            joint2_tau_sign_product > 0.0 ? "same" : (joint2_tau_sign_product < 0.0 ? "opposite" : "zero"),
            format_eigen_vector(q_snapshot).c_str(),
            format_eigen_vector(tau_snapshot).c_str(),
            format_eigen_vector(tau_est_snapshot).c_str());
    }

    std::mutex data_mutex_;
    bool driver_ready_{false};
    bool has_data_{false};
    bool enable_payload_service_{false};
    std::string gains_mode_;
    std::string payload_service_name_;
    std::vector<double> kp_;
    std::vector<double> kd_;
    Eigen::VectorXd current_q_;
    Eigen::VectorXd current_dq_;
    Eigen::VectorXd current_tau_est_;
    Eigen::VectorXd last_tau_g_{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd last_published_q_{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd last_command_q_{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd last_command_tau_{Eigen::VectorXd::Zero(5)};
    rclcpp::Time last_valid_state_time_{0, 0, RCL_ROS_TIME};
    uint64_t state_message_count_{0};
    uint64_t accepted_state_count_{0};
    uint64_t invalid_state_count_{0};
    uint64_t short_state_count_{0};
    uint64_t command_publish_count_{0};

    std::unique_ptr<arm2_task::DynamicsManager> dyn_manager_;

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Service<robot_msgs::srv::SetPayloadState>::SharedPtr payload_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GravityCompTestNode>());
    rclcpp::shutdown();
    return 0;
}
