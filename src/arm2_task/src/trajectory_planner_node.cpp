#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
    using MoveJoint = robot_msgs::action::MoveJoint;
    using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

    TrajectoryPlannerNode() : Node("trajectory_planner_node")
    {
        state_topic_ =
            this->declare_parameter("trajectory_planner.state_topic", "/arm2/_lowState/joint");
        target_topic_ =
            this->declare_parameter("trajectory_planner.target_topic", "/joint_target_state");
        ready_topic_ =
            this->declare_parameter("trajectory_planner.ready_topic", "/robot_driver/ready");
        action_name_ =
            this->declare_parameter("trajectory_planner.action_name", std::string("move_joint"));
        publish_rate_hz_ =
            this->declare_parameter("trajectory_planner.publish_rate_hz", 100.0);
        max_velocity_limit_ =
            this->declare_parameter("trajectory_planner.max_velocity", 0.8);
        max_acceleration_limit_ =
            this->declare_parameter("trajectory_planner.max_acceleration", 1.2);
        default_blend_radius_ =
            this->declare_parameter("trajectory_planner.dist_threshold", 0.05);
        min_segment_duration_ =
            this->declare_parameter("trajectory_planner.min_segment_duration", 0.5);
        target_qos_depth_ =
            this->declare_parameter("trajectory_planner.target_qos_depth", 10);
        log_targets_ =
            this->declare_parameter("trajectory_planner.log_targets", false);

        if (publish_rate_hz_ < 1.0) {
            publish_rate_hz_ = 1.0;
        }
        if (max_velocity_limit_ <= 0.0) {
            max_velocity_limit_ = 0.8;
        }
        if (max_acceleration_limit_ <= 0.0) {
            max_acceleration_limit_ = 1.2;
        }
        if (default_blend_radius_ < 0.0) {
            default_blend_radius_ = 0.0;
        }
        if (min_segment_duration_ <= 0.0) {
            min_segment_duration_ = 0.5;
        }
        if (target_qos_depth_ < 1) {
            target_qos_depth_ = 1;
        }

        current_q_ = Eigen::VectorXd::Zero(kDof);
        current_dq_ = Eigen::VectorXd::Zero(kDof);
        target_q_ = Eigen::VectorXd::Zero(kDof);
        target_dq_ = Eigen::VectorXd::Zero(kDof);
        target_ddq_ = Eigen::VectorXd::Zero(kDof);

        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        target_pub_ = this->create_publisher<robot_msgs::msg::RobotState>(target_topic_, target_qos);
        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            state_qos,
            std::bind(&TrajectoryPlannerNode::state_callback, this, std::placeholders::_1));
        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&TrajectoryPlannerNode::driver_ready_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<MoveJoint>(
            this,
            action_name_,
            std::bind(&TrajectoryPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryPlannerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryPlannerNode::handle_accepted, this, std::placeholders::_1));

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TrajectoryPlannerNode::publish_loop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Trajectory planner node started. action=%s state_topic=%s target_topic=%s ready_topic=%s publish_rate=%.1fHz v_max=%.3f a_max=%.3f blend=%.3f",
            action_name_.c_str(),
            state_topic_.c_str(),
            target_topic_.c_str(),
            ready_topic_.c_str(),
            publish_rate_hz_,
            max_velocity_limit_,
            max_acceleration_limit_,
            default_blend_radius_);
    }

private:
    static constexpr int kDof = 5;

    struct TrajSegment
    {
        Eigen::VectorXd q_start;
        Eigen::VectorXd q_end;
        double t_total{0.0};
        rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    };

    bool validate_state_msg(const robot_msgs::msg::RobotState::SharedPtr & msg) const
    {
        if (!msg || msg->motor_state.size() < static_cast<size_t>(kDof)) {
            return false;
        }
        for (int i = 0; i < kDof; ++i) {
            const auto & motor = msg->motor_state[static_cast<size_t>(i)];
            if (!motor.valid || !std::isfinite(motor.q) || !std::isfinite(motor.dq)) {
                return false;
            }
        }
        return true;
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        if (!validate_state_msg(msg)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Ignore invalid planner state message.");
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < kDof; ++i) {
            current_q_[i] = msg->motor_state[static_cast<size_t>(i)].q;
            current_dq_[i] = msg->motor_state[static_cast<size_t>(i)].dq;
        }
        has_state_ = true;
        if (!target_initialized_) {
            target_q_ = current_q_;
            target_dq_.setZero();
            target_ddq_.setZero();
            target_initialized_ = true;
        }
    }

    void driver_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        if (driver_ready_ != msg->data) {
            RCLCPP_INFO(
                this->get_logger(),
                "Planner driver ready changed: %s -> %s",
                driver_ready_ ? "true" : "false",
                msg->data ? "true" : "false");
        }
        driver_ready_ = msg->data;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const MoveJoint::Goal> goal)
    {
        if (!driver_ready_ || !has_state_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Reject move goal: driver_ready=%d has_state=%d",
                driver_ready_,
                has_state_);
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!goal || goal->num_points <= 0 ||
            goal->joint_targets.size() != static_cast<size_t>(goal->num_points * kDof))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Reject move goal: invalid target array size=%zu num_points=%d",
                goal ? goal->joint_targets.size() : 0UL,
                goal ? goal->num_points : 0);
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveJoint>)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waypoint_queue_.clear();
        active_segment_ = nullptr;
        next_segment_ = nullptr;
        if (has_state_) {
            target_q_ = current_q_;
        }
        target_dq_.setZero();
        target_ddq_.setZero();
        is_moving_.store(false);
        action_progress_ = 0.0;
        completed_segments_ = 0;
        total_segments_ = 0;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            waypoint_queue_.clear();
            active_segment_ = nullptr;
            next_segment_ = nullptr;

            for (int i = 0; i < goal->num_points; ++i) {
                Eigen::VectorXd q_target(kDof);
                for (int j = 0; j < kDof; ++j) {
                    q_target[j] = goal->joint_targets[static_cast<size_t>(i * kDof + j)];
                }
                waypoint_queue_.push_back(q_target);
            }

            current_max_v_ = goal->max_velocity > 0.0 ? goal->max_velocity : max_velocity_limit_;
            current_max_a_ =
                goal->max_acceleration > 0.0 ? goal->max_acceleration : max_acceleration_limit_;
            current_blend_radius_ =
                goal->blend_radius > 0.0 ? goal->blend_radius : default_blend_radius_;

            total_segments_ = static_cast<size_t>(goal->num_points);
            completed_segments_ = 0;
            action_progress_ = 0.0;

            if (!target_initialized_) {
                target_q_ = current_q_;
                target_initialized_ = true;
            }

            if (!waypoint_queue_.empty()) {
                active_segment_ = plan_trapezoid(target_q_, waypoint_queue_.front());
                waypoint_queue_.pop_front();
                is_moving_.store(true);
            }
        }

        std::thread(
            std::bind(&TrajectoryPlannerNode::execute_move, this, std::placeholders::_1),
            goal_handle)
            .detach();
    }

    void execute_move(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
    {
        auto feedback = std::make_shared<MoveJoint::Feedback>();
        auto result = std::make_shared<MoveJoint::Result>();
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                handle_cancel(goal_handle);
                result->success = false;
                result->message = "Trajectory canceled";
                goal_handle->canceled(result);
                return;
            }

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                feedback->progress = action_progress_;
                feedback->current_errors.clear();
                if (has_state_ && target_initialized_) {
                    feedback->current_errors.resize(kDof);
                    Eigen::VectorXd err = target_q_ - current_q_;
                    for (int i = 0; i < kDof; ++i) {
                        feedback->current_errors[static_cast<size_t>(i)] = err[i];
                    }
                }
            }
            goal_handle->publish_feedback(feedback);

            if (!is_moving_.load()) {
                break;
            }
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->success = true;
            result->message = "Trajectory finished";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory action finished.");
        }
    }

    std::shared_ptr<TrajSegment> plan_trapezoid(
        const Eigen::VectorXd & start,
        const Eigen::VectorXd & end)
    {
        auto seg = std::make_shared<TrajSegment>();
        seg->q_start = start;
        seg->q_end = end;
        seg->start_time = this->get_clock()->now();

        Eigen::VectorXd delta_q = (end - start).cwiseAbs();
        double max_t = min_segment_duration_;

        for (int i = 0; i < kDof; ++i) {
            if (delta_q[i] < 1e-4) {
                continue;
            }
            const double t_v = delta_q[i] / current_max_v_;
            const double t_a = std::sqrt(delta_q[i] / current_max_a_) * 2.5;
            max_t = std::max(max_t, std::max(t_v, t_a));
        }

        seg->t_total = max_t;
        return seg;
    }

    static void compute_segment_state(
        const TrajSegment & seg,
        double t,
        Eigen::VectorXd & q,
        Eigen::VectorXd & v,
        Eigen::VectorXd & a)
    {
        const double T = seg.t_total;
        t = std::clamp(t, 0.0, T);

        const double s = t / T;
        const double s2 = s * s;
        const double s3 = s2 * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;

        const double pos_factor = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
        const double vel_factor = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
        const double acc_factor = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);

        const Eigen::VectorXd delta_q = seg.q_end - seg.q_start;
        q = seg.q_start + delta_q * pos_factor;
        v = delta_q * vel_factor;
        a = delta_q * acc_factor;
    }

    void publish_loop()
    {
        if (!driver_ready_ || !has_state_ || !target_initialized_) {
            return;
        }

        Eigen::VectorXd q_final = target_q_;
        Eigen::VectorXd dq_final = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd ddq_final = Eigen::VectorXd::Zero(kDof);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (active_segment_) {
                const double t =
                    (this->get_clock()->now() - active_segment_->start_time).seconds();
                compute_segment_state(*active_segment_, t, q_final, dq_final, ddq_final);

                const double dist_to_target = (active_segment_->q_end - q_final).norm();
                if (dist_to_target < current_blend_radius_ && !waypoint_queue_.empty() && !next_segment_) {
                    next_segment_ = plan_trapezoid(active_segment_->q_end, waypoint_queue_.front());
                    waypoint_queue_.pop_front();
                }

                if (next_segment_) {
                    const double t_next =
                        (this->get_clock()->now() - next_segment_->start_time).seconds();
                    Eigen::VectorXd q2(kDof), v2(kDof), a2(kDof);
                    compute_segment_state(*next_segment_, t_next, q2, v2, a2);
                    q_final += (q2 - next_segment_->q_start);
                    dq_final += v2;
                    ddq_final += a2;

                    if (t >= active_segment_->t_total) {
                        ++completed_segments_;
                        active_segment_ = next_segment_;
                        next_segment_ = nullptr;
                    }
                } else if (t >= active_segment_->t_total) {
                    ++completed_segments_;
                    active_segment_ = nullptr;
                    if (waypoint_queue_.empty()) {
                        is_moving_.store(false);
                    }
                }

                const double segment_progress = active_segment_
                                                    ? std::clamp(
                                                          (this->get_clock()->now() - active_segment_->start_time)
                                                                  .seconds() /
                                                              std::max(1e-6, active_segment_->t_total),
                                                          0.0,
                                                          1.0)
                                                    : 1.0;
                if (total_segments_ > 0) {
                    action_progress_ =
                        std::clamp(
                            (static_cast<double>(completed_segments_) + segment_progress) /
                                static_cast<double>(total_segments_),
                            0.0,
                            1.0);
                }

                target_q_ = q_final;
                target_dq_ = dq_final;
                target_ddq_ = ddq_final;
            } else {
                target_dq_.setZero();
                target_ddq_.setZero();
                dq_final = target_dq_;
                ddq_final = target_ddq_;
                if (!is_moving_.load()) {
                    action_progress_ = total_segments_ > 0 ? 1.0 : 0.0;
                }
            }
        }

        publish_target(q_final, dq_final, ddq_final);

        if (log_targets_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Planner target q=[%.3f, %.3f, %.3f, %.3f, %.3f] dq=[%.3f, %.3f, %.3f, %.3f, %.3f] ddq=[%.3f, %.3f, %.3f, %.3f, %.3f]",
                q_final[0],
                q_final[1],
                q_final[2],
                q_final[3],
                q_final[4],
                dq_final[0],
                dq_final[1],
                dq_final[2],
                dq_final[3],
                dq_final[4],
                ddq_final[0],
                ddq_final[1],
                ddq_final[2],
                ddq_final[3],
                ddq_final[4]);
        }
    }

    void publish_target(
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & dq,
        const Eigen::VectorXd & ddq)
    {
        robot_msgs::msg::RobotState msg;
        msg.motor_state.resize(kDof);
        for (int i = 0; i < kDof; ++i) {
            auto & motor = msg.motor_state[static_cast<size_t>(i)];
            motor.q = static_cast<float>(q[i]);
            motor.dq = static_cast<float>(dq[i]);
            motor.ddq = static_cast<float>(ddq[i]);
            motor.tau_est = 0.0f;
            motor.cur = 0.0f;
            motor.valid = true;
        }
        target_pub_->publish(msg);
    }

    std::mutex data_mutex_;
    bool driver_ready_{false};
    bool has_state_{false};
    bool target_initialized_{false};
    std::atomic<bool> is_moving_{false};

    Eigen::VectorXd current_q_;
    Eigen::VectorXd current_dq_;
    Eigen::VectorXd target_q_;
    Eigen::VectorXd target_dq_;
    Eigen::VectorXd target_ddq_;

    std::deque<Eigen::VectorXd> waypoint_queue_;
    std::shared_ptr<TrajSegment> active_segment_{nullptr};
    std::shared_ptr<TrajSegment> next_segment_{nullptr};

    double publish_rate_hz_{100.0};
    double max_velocity_limit_{0.8};
    double max_acceleration_limit_{1.2};
    double default_blend_radius_{0.05};
    double min_segment_duration_{0.5};
    int target_qos_depth_{10};
    bool log_targets_{false};

    double current_max_v_{0.8};
    double current_max_a_{1.2};
    double current_blend_radius_{0.05};
    double action_progress_{0.0};
    size_t total_segments_{0};
    size_t completed_segments_{0};

    std::string state_topic_;
    std::string target_topic_;
    std::string ready_topic_;
    std::string action_name_;

    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr target_pub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp_action::Server<MoveJoint>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
