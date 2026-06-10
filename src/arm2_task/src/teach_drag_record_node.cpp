#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <limits>
#include <limits.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int kDof = 5;
using JointArray = std::array<double, kDof>;

struct RecordedSample
{
    double time_from_start_sec{0.0};
    JointArray q{};
    JointArray dq{};
    JointArray tau{};
};

bool is_absolute_path(const std::string & path)
{
    return !path.empty() && path.front() == '/';
}

std::string join_paths(const std::string & lhs, const std::string & rhs)
{
    if (lhs.empty()) {
        return rhs;
    }
    if (rhs.empty()) {
        return lhs;
    }
    if (lhs.back() == '/') {
        return lhs + rhs;
    }
    return lhs + "/" + rhs;
}

std::string current_working_directory()
{
    char buffer[PATH_MAX];
    if (::getcwd(buffer, sizeof(buffer)) == nullptr) {
        return ".";
    }
    return std::string(buffer);
}

std::string resolve_workspace_path(const std::string & path)
{
    if (is_absolute_path(path)) {
        return path;
    }

    const char * workspace_dir = std::getenv("ARM_WORKSPACE_DIR");
    if (workspace_dir != nullptr && std::strlen(workspace_dir) > 0U) {
        return join_paths(workspace_dir, path);
    }

    return join_paths(current_working_directory(), path);
}

std::string parent_directory(const std::string & path)
{
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return std::string();
    }
    if (pos == 0U) {
        return "/";
    }
    return path.substr(0, pos);
}

bool ensure_directory_recursive(const std::string & directory)
{
    if (directory.empty() || directory == ".") {
        return true;
    }

    std::string current;
    std::size_t start = 0U;
    if (directory.front() == '/') {
        current = "/";
        start = 1U;
    }

    while (start <= directory.size()) {
        const auto separator = directory.find('/', start);
        const std::string part = directory.substr(start, separator - start);
        if (!part.empty()) {
            current = current == "/" ? current + part : current + "/" + part;
            if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                return false;
            }
        }

        if (separator == std::string::npos) {
            break;
        }
        start = separator + 1U;
    }

    return true;
}

bool file_exists(const std::string & path)
{
    return ::access(path.c_str(), F_OK) == 0;
}

double l2_norm_delta(const JointArray & lhs, const JointArray & rhs)
{
    double accum = 0.0;
    for (int i = 0; i < kDof; ++i) {
        const double delta = lhs[static_cast<std::size_t>(i)] - rhs[static_cast<std::size_t>(i)];
        accum += delta * delta;
    }
    return std::sqrt(accum);
}

}  // namespace

class TeachDragRecordNode : public rclcpp::Node
{
public:
    TeachDragRecordNode() : Node("teach_drag_record_node")
    {
        output_file_ =
            resolve_workspace_path(this->declare_parameter(
                "teach_drag_record.output_file",
                std::string("recordings/teach_drag_trajectory.csv")));
        state_topic_ =
            this->declare_parameter("teach_drag_record.state_topic", std::string("/arm2/_lowState/joint"));
        ready_topic_ =
            this->declare_parameter("teach_drag_record.ready_topic", std::string("/robot_driver/ready"));
        autosave_period_sec_ =
            this->declare_parameter("teach_drag_record.autosave_period_sec", 1.0);
        min_sample_period_sec_ =
            this->declare_parameter("teach_drag_record.min_sample_period_sec", 0.05);
        min_joint_delta_norm_ =
            this->declare_parameter("teach_drag_record.min_joint_delta_norm", 0.01);
        require_ready_ =
            this->declare_parameter("teach_drag_record.require_ready", true);
        overwrite_output_ =
            this->declare_parameter("teach_drag_record.overwrite_output", true);

        if (!overwrite_output_ && file_exists(output_file_)) {
            throw std::runtime_error("teach_drag_record.output_file already exists: " + output_file_);
        }

        const auto output_dir = parent_directory(output_file_);
        if (!ensure_directory_recursive(output_dir)) {
            throw std::runtime_error("Failed to create output directory: " + output_dir);
        }

        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            state_qos,
            std::bind(&TeachDragRecordNode::state_callback, this, std::placeholders::_1));

        ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&TeachDragRecordNode::ready_callback, this, std::placeholders::_1));

        if (autosave_period_sec_ > 0.0) {
            autosave_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(autosave_period_sec_)),
                std::bind(&TeachDragRecordNode::autosave_callback, this));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Teach drag record node started. state_topic=%s ready_topic=%s output=%s autosave=%.2fs min_period=%.3fs min_delta=%.4f require_ready=%s overwrite=%s",
            state_topic_.c_str(),
            ready_topic_.c_str(),
            output_file_.c_str(),
            autosave_period_sec_,
            min_sample_period_sec_,
            min_joint_delta_norm_,
            require_ready_ ? "true" : "false",
            overwrite_output_ ? "true" : "false");
    }

    ~TeachDragRecordNode() override
    {
        finalize();
    }

    void finalize()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (finalized_) {
            return;
        }
        finalized_ = true;

        maybe_append_latest_sample_locked(true);
        if (dirty_) {
            save_samples_locked("final");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Teach drag recorder exiting without new samples. received=%zu recorded=%zu",
                received_state_count_,
                samples_.size());
        }
    }

private:
    void ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        if (driver_ready_ != msg->data) {
            RCLCPP_INFO(
                this->get_logger(),
                "Teach drag recorder driver ready changed: %s -> %s",
                driver_ready_ ? "true" : "false",
                msg->data ? "true" : "false");
        }
        driver_ready_ = msg->data;
    }

    bool extract_state(
        const robot_msgs::msg::RobotState::SharedPtr & msg,
        JointArray & q,
        JointArray & dq,
        JointArray & tau) const
    {
        if (!msg || msg->motor_state.size() < static_cast<std::size_t>(kDof)) {
            return false;
        }

        for (int i = 0; i < kDof; ++i) {
            const auto & motor = msg->motor_state[static_cast<std::size_t>(i)];
            if (!motor.valid || !std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
                !std::isfinite(motor.tau_est))
            {
                return false;
            }
            q[static_cast<std::size_t>(i)] = motor.q;
            dq[static_cast<std::size_t>(i)] = motor.dq;
            tau[static_cast<std::size_t>(i)] = motor.tau_est;
        }
        return true;
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        if (require_ready_ && !driver_ready_) {
            return;
        }

        JointArray q{};
        JointArray dq{};
        JointArray tau{};
        if (!extract_state(msg, q, dq, tau)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Teach drag recorder ignored invalid state message.");
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(data_mutex_);
        ++received_state_count_;

        latest_q_ = q;
        latest_dq_ = dq;
        latest_tau_ = tau;
        has_latest_state_ = true;

        if (!recording_started_) {
            recording_started_ = true;
            start_time_ = now;
            latest_time_from_start_sec_ = 0.0;
            append_sample_locked(0.0, q, dq, tau);
            RCLCPP_INFO(
                this->get_logger(),
                "Teach drag recording started. First sample captured at t=0.0s");
            return;
        }

        latest_time_from_start_sec_ =
            std::chrono::duration<double>(now - start_time_).count();
        maybe_append_latest_sample_locked(false);
    }

    void maybe_append_latest_sample_locked(bool force)
    {
        if (!has_latest_state_) {
            return;
        }

        if (!force && latest_time_from_start_sec_ - last_record_time_sec_ < min_sample_period_sec_) {
            return;
        }

        if (!force && has_recorded_sample_) {
            const double delta_norm = l2_norm_delta(latest_q_, last_recorded_q_);
            if (delta_norm < min_joint_delta_norm_) {
                return;
            }
        }

        append_sample_locked(
            latest_time_from_start_sec_,
            latest_q_,
            latest_dq_,
            latest_tau_);
    }

    void append_sample_locked(
        double time_from_start_sec,
        const JointArray & q,
        const JointArray & dq,
        const JointArray & tau)
    {
        RecordedSample sample;
        sample.time_from_start_sec = time_from_start_sec;
        sample.q = q;
        sample.dq = dq;
        sample.tau = tau;
        samples_.push_back(sample);

        last_record_time_sec_ = time_from_start_sec;
        last_recorded_q_ = q;
        has_recorded_sample_ = true;
        dirty_ = true;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Teach drag recorder captured %zu samples. latest_t=%.3fs",
            samples_.size(),
            time_from_start_sec);
    }

    void autosave_callback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!dirty_) {
            return;
        }
        save_samples_locked("autosave");
    }

    void save_samples_locked(const char * reason)
    {
        const std::string tmp_path = output_file_ + ".tmp";
        std::ofstream output(tmp_path, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open temporary recording file for write: %s",
                tmp_path.c_str());
            return;
        }

        output << std::fixed << std::setprecision(6);
        output << "time_from_start_sec";
        for (int i = 0; i < kDof; ++i) {
            output << ",q" << (i + 1);
        }
        for (int i = 0; i < kDof; ++i) {
            output << ",dq" << (i + 1);
        }
        for (int i = 0; i < kDof; ++i) {
            output << ",tau" << (i + 1);
        }
        output << "\n";

        for (const auto & sample : samples_) {
            output << sample.time_from_start_sec;
            for (double value : sample.q) {
                output << "," << value;
            }
            for (double value : sample.dq) {
                output << "," << value;
            }
            for (double value : sample.tau) {
                output << "," << value;
            }
            output << "\n";
        }
        output.close();

        if (std::rename(tmp_path.c_str(), output_file_.c_str()) != 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to replace recording file %s with %s: %s",
                output_file_.c_str(),
                tmp_path.c_str(),
                std::strerror(errno));
            return;
        }

        dirty_ = false;
        RCLCPP_INFO(
            this->get_logger(),
            "Teach drag recorder %s saved %zu samples to %s (received=%zu)",
            reason,
            samples_.size(),
            output_file_.c_str(),
            received_state_count_);
    }

    std::mutex data_mutex_;
    std::vector<RecordedSample> samples_;
    JointArray latest_q_{};
    JointArray latest_dq_{};
    JointArray latest_tau_{};
    JointArray last_recorded_q_{};

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::TimerBase::SharedPtr autosave_timer_;

    std::string output_file_;
    std::string state_topic_;
    std::string ready_topic_;
    double autosave_period_sec_{1.0};
    double min_sample_period_sec_{0.05};
    double min_joint_delta_norm_{0.01};
    bool require_ready_{true};
    bool overwrite_output_{true};
    bool driver_ready_{false};
    bool recording_started_{false};
    bool has_latest_state_{false};
    bool has_recorded_sample_{false};
    bool dirty_{false};
    bool finalized_{false};
    std::chrono::steady_clock::time_point start_time_{};
    double latest_time_from_start_sec_{0.0};
    double last_record_time_sec_{-std::numeric_limits<double>::infinity()};
    std::size_t received_state_count_{0U};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TeachDragRecordNode> node;
    int rc = 0;

    try {
        node = std::make_shared<TeachDragRecordNode>();
        rclcpp::spin(node);
    } catch (const std::exception & error) {
        std::fprintf(stderr, "teach_drag_record_node failed: %s\n", error.what());
        rc = 1;
    }

    if (node) {
        node->finalize();
        node.reset();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return rc;
}
