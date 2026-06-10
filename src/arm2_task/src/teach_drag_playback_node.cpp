#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <limits.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/motor_state.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int kDof = 5;
using JointArray = std::array<double, kDof>;

struct PlaybackSample
{
    double time_from_start_sec{0.0};
    JointArray q{};
    JointArray dq{};
    JointArray ddq{};
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

std::string trim(const std::string & input)
{
    const auto first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return std::string();
    }
    const auto last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1U);
}

std::vector<std::string> split_csv(const std::string & line)
{
    std::vector<std::string> tokens;
    std::stringstream stream(line);
    std::string token;
    while (std::getline(stream, token, ',')) {
        tokens.push_back(trim(token));
    }
    return tokens;
}

bool try_parse_double(const std::string & input, double & output)
{
    try {
        std::size_t consumed = 0U;
        output = std::stod(input, &consumed);
        return consumed == input.size();
    } catch (...) {
        return false;
    }
}

JointArray lerp(const JointArray & lhs, const JointArray & rhs, double alpha)
{
    JointArray result{};
    for (int i = 0; i < kDof; ++i) {
        result[static_cast<std::size_t>(i)] =
            lhs[static_cast<std::size_t>(i)] +
            alpha * (rhs[static_cast<std::size_t>(i)] - lhs[static_cast<std::size_t>(i)]);
    }
    return result;
}

}  // namespace

class TeachDragPlaybackNode : public rclcpp::Node
{
public:
    TeachDragPlaybackNode() : Node("teach_drag_playback_node")
    {
        input_file_ =
            resolve_workspace_path(this->declare_parameter(
                "teach_drag_playback.input_file",
                std::string("recordings/teach_drag_trajectory.csv")));
        target_topic_ =
            this->declare_parameter("teach_drag_playback.target_topic", std::string("/joint_target_state"));
        ready_topic_ =
            this->declare_parameter("teach_drag_playback.ready_topic", std::string("/robot_driver/ready"));
        mode_service_name_ =
            this->declare_parameter("teach_drag_playback.mode_service", std::string("set_controller_mode"));
        mode_name_ =
            this->declare_parameter("teach_drag_playback.mode_name", std::string("moving"));
        request_mode_on_startup_ =
            this->declare_parameter("teach_drag_playback.request_mode_on_startup", true);
        wait_for_ready_ =
            this->declare_parameter("teach_drag_playback.wait_for_ready", true);
        publish_rate_hz_ =
            this->declare_parameter("teach_drag_playback.publish_rate_hz", 100.0);
        target_qos_depth_ =
            this->declare_parameter("teach_drag_playback.target_qos_depth", 10);
        hold_last_sample_ =
            this->declare_parameter("teach_drag_playback.hold_last_sample", true);
        loop_playback_ =
            this->declare_parameter("teach_drag_playback.loop", false);
        playback_speed_ =
            this->declare_parameter("teach_drag_playback.playback_speed", 1.0);
        log_progress_ =
            this->declare_parameter("teach_drag_playback.log_progress", false);

        if (publish_rate_hz_ < 1.0) {
            publish_rate_hz_ = 1.0;
        }
        if (target_qos_depth_ < 1) {
            target_qos_depth_ = 1;
        }
        if (playback_speed_ <= 0.0) {
            playback_speed_ = 1.0;
        }

        load_samples();
        if (samples_.empty()) {
            throw std::runtime_error("No valid playback samples loaded from " + input_file_);
        }

        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        target_pub_ = this->create_publisher<robot_msgs::msg::RobotState>(target_topic_, target_qos);
        ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&TeachDragPlaybackNode::ready_callback, this, std::placeholders::_1));

        if (request_mode_on_startup_) {
            mode_client_ =
                this->create_client<robot_msgs::srv::SetControllerMode>(mode_service_name_);
            mode_request_timer_ = this->create_wall_timer(
                1s,
                std::bind(&TeachDragPlaybackNode::request_mode_if_needed, this));
        }

        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_)),
            std::bind(&TeachDragPlaybackNode::publish_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Teach drag playback node started. input=%s samples=%zu target_topic=%s ready_topic=%s publish_rate=%.1fHz speed=%.2fx wait_for_ready=%s hold_last=%s loop=%s",
            input_file_.c_str(),
            samples_.size(),
            target_topic_.c_str(),
            ready_topic_.c_str(),
            publish_rate_hz_,
            playback_speed_,
            wait_for_ready_ ? "true" : "false",
            hold_last_sample_ ? "true" : "false",
            loop_playback_ ? "true" : "false");
    }

private:
    void load_samples()
    {
        std::ifstream input(input_file_);
        if (!input.is_open()) {
            throw std::runtime_error("Failed to open playback file: " + input_file_);
        }

        std::string line;
        bool has_explicit_dq = false;
        while (std::getline(input, line)) {
            line = trim(line);
            if (line.empty() || line.front() == '#') {
                continue;
            }

            const auto tokens = split_csv(line);
            if (tokens.empty()) {
                continue;
            }

            double time_from_start_sec = 0.0;
            if (!try_parse_double(tokens.front(), time_from_start_sec)) {
                continue;
            }

            if (tokens.size() != 6U && tokens.size() != 11U && tokens.size() != 16U) {
                throw std::runtime_error("Unexpected playback CSV column count in " + input_file_);
            }

            PlaybackSample sample;
            sample.time_from_start_sec = time_from_start_sec;
            std::size_t cursor = 1U;
            for (int i = 0; i < kDof; ++i) {
                double value = 0.0;
                if (!try_parse_double(tokens[cursor++], value)) {
                    throw std::runtime_error("Failed to parse q column in " + input_file_);
                }
                sample.q[static_cast<std::size_t>(i)] = value;
            }

            if (tokens.size() >= 11U) {
                has_explicit_dq = true;
                for (int i = 0; i < kDof; ++i) {
                    double value = 0.0;
                    if (!try_parse_double(tokens[cursor++], value)) {
                        throw std::runtime_error("Failed to parse dq column in " + input_file_);
                    }
                    sample.dq[static_cast<std::size_t>(i)] = value;
                }
            } else {
                sample.dq.fill(0.0);
            }

            sample.ddq.fill(0.0);
            samples_.push_back(sample);
        }

        if (!std::is_sorted(
                samples_.begin(),
                samples_.end(),
                [](const PlaybackSample & lhs, const PlaybackSample & rhs) {
                    return lhs.time_from_start_sec < rhs.time_from_start_sec;
                }))
        {
            std::sort(
                samples_.begin(),
                samples_.end(),
                [](const PlaybackSample & lhs, const PlaybackSample & rhs) {
                    return lhs.time_from_start_sec < rhs.time_from_start_sec;
                });
        }

        if (!has_explicit_dq && samples_.size() >= 2U) {
            for (std::size_t i = 0; i + 1U < samples_.size(); ++i) {
                const double dt =
                    samples_[i + 1U].time_from_start_sec - samples_[i].time_from_start_sec;
                if (dt <= 1e-6) {
                    continue;
                }
                for (int joint = 0; joint < kDof; ++joint) {
                    samples_[i].dq[static_cast<std::size_t>(joint)] =
                        (samples_[i + 1U].q[static_cast<std::size_t>(joint)] -
                         samples_[i].q[static_cast<std::size_t>(joint)]) /
                        dt;
                }
            }
            samples_.back().dq = samples_[samples_.size() - 2U].dq;
        }

        if (samples_.size() >= 2U) {
            for (std::size_t i = 0; i + 1U < samples_.size(); ++i) {
                const double dt =
                    samples_[i + 1U].time_from_start_sec - samples_[i].time_from_start_sec;
                if (dt <= 1e-6) {
                    continue;
                }
                for (int joint = 0; joint < kDof; ++joint) {
                    samples_[i].ddq[static_cast<std::size_t>(joint)] =
                        (samples_[i + 1U].dq[static_cast<std::size_t>(joint)] -
                         samples_[i].dq[static_cast<std::size_t>(joint)]) /
                        dt;
                }
            }
            samples_.back().ddq = samples_[samples_.size() - 2U].ddq;
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
                "Teach drag playback driver ready changed: %s -> %s",
                driver_ready_ ? "true" : "false",
                msg->data ? "true" : "false");
        }
        driver_ready_ = msg->data;
    }

    void request_mode_if_needed()
    {
        if (!request_mode_on_startup_ || mode_request_completed_) {
            return;
        }

        if (!mode_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Teach drag playback waiting for mode service %s",
                mode_service_name_.c_str());
            return;
        }

        auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
        request->mode = mode_name_;
        mode_client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedFuture future) {
                const auto response = future.get();
                if (response && response->success) {
                    mode_request_completed_ = true;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Teach drag playback requested controller mode: %s",
                        mode_name_.c_str());
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Teach drag playback mode request failed for %s",
                        mode_name_.c_str());
                }
            });
    }

    void publish_callback()
    {
        if (wait_for_ready_ && !driver_ready_) {
            return;
        }

        if (playback_completed_ && !loop_playback_) {
            if (hold_last_sample_) {
                publish_sample(samples_.back());
            }
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!playback_started_) {
            playback_started_ = true;
            playback_start_time_ = now;
            sample_cursor_ = 0U;
            playback_completed_ = false;
            RCLCPP_INFO(this->get_logger(), "Teach drag playback started.");
        }

        double playback_time_sec =
            std::chrono::duration<double>(now - playback_start_time_).count() * playback_speed_;
        const double trajectory_end_sec = samples_.back().time_from_start_sec;

        if (trajectory_end_sec > 0.0 && playback_time_sec > trajectory_end_sec) {
            if (loop_playback_) {
                while (playback_time_sec > trajectory_end_sec) {
                    playback_time_sec -= trajectory_end_sec;
                }
                playback_start_time_ =
                    now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(playback_time_sec / playback_speed_));
                sample_cursor_ = 0U;
            } else {
                playback_completed_ = true;
                if (hold_last_sample_) {
                    publish_sample(samples_.back());
                }
                RCLCPP_INFO(this->get_logger(), "Teach drag playback reached the end of the trajectory.");
                return;
            }
        }

        while (sample_cursor_ + 1U < samples_.size() &&
               samples_[sample_cursor_ + 1U].time_from_start_sec < playback_time_sec)
        {
            ++sample_cursor_;
        }

        PlaybackSample sample = interpolate_sample(playback_time_sec);
        publish_sample(sample);

        if (log_progress_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Teach drag playback progress %.2f / %.2f sec",
                playback_time_sec,
                trajectory_end_sec);
        }
    }

    PlaybackSample interpolate_sample(double playback_time_sec) const
    {
        if (samples_.size() == 1U || sample_cursor_ + 1U >= samples_.size()) {
            return samples_[std::min(sample_cursor_, samples_.size() - 1U)];
        }

        const auto & lhs = samples_[sample_cursor_];
        const auto & rhs = samples_[sample_cursor_ + 1U];
        const double span = rhs.time_from_start_sec - lhs.time_from_start_sec;
        if (span <= 1e-6) {
            return lhs;
        }

        const double alpha =
            std::clamp((playback_time_sec - lhs.time_from_start_sec) / span, 0.0, 1.0);
        PlaybackSample interpolated;
        interpolated.time_from_start_sec = playback_time_sec;
        interpolated.q = lerp(lhs.q, rhs.q, alpha);
        interpolated.dq = lerp(lhs.dq, rhs.dq, alpha);
        interpolated.ddq = lerp(lhs.ddq, rhs.ddq, alpha);
        return interpolated;
    }

    void publish_sample(const PlaybackSample & sample)
    {
        robot_msgs::msg::RobotState msg;
        msg.motor_state.reserve(kDof);

        for (int i = 0; i < kDof; ++i) {
            robot_msgs::msg::MotorState motor;
            motor.q = static_cast<float>(sample.q[static_cast<std::size_t>(i)]);
            motor.dq = static_cast<float>(sample.dq[static_cast<std::size_t>(i)]);
            motor.ddq = static_cast<float>(sample.ddq[static_cast<std::size_t>(i)]);
            motor.tau_est = 0.0f;
            motor.cur = 0.0f;
            motor.valid = true;
            msg.motor_state.push_back(motor);
        }

        target_pub_->publish(msg);
    }

    std::vector<PlaybackSample> samples_;
    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr target_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr mode_request_timer_;

    std::string input_file_;
    std::string target_topic_;
    std::string ready_topic_;
    std::string mode_service_name_;
    std::string mode_name_;
    bool request_mode_on_startup_{true};
    bool wait_for_ready_{true};
    double publish_rate_hz_{100.0};
    int target_qos_depth_{10};
    bool hold_last_sample_{true};
    bool loop_playback_{false};
    double playback_speed_{1.0};
    bool log_progress_{false};
    bool driver_ready_{false};
    bool mode_request_completed_{false};
    bool playback_started_{false};
    bool playback_completed_{false};
    std::chrono::steady_clock::time_point playback_start_time_{};
    std::size_t sample_cursor_{0U};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;

    try {
        auto node = std::make_shared<TeachDragPlaybackNode>();
        rclcpp::spin(node);
    } catch (const std::exception & error) {
        std::fprintf(stderr, "teach_drag_playback_node failed: %s\n", error.what());
        rc = 1;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return rc;
}
