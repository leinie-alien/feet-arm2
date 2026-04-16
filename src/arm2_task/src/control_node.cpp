#include <chrono>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <map>
#include <deque>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"

#include "arm2_task/kinematics_engine.hpp"
#include "arm2_task/dynamics_manager.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
    // --- Action 类型别名 (修复编译报错的关键) ---
    using MoveJoint = robot_msgs::action::MoveJoint;
    using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

    struct PDGains
    {
        std::vector<double> kp;
        std::vector<double> kd;
        bool operator==(const PDGains &other) const
        {
            return kp == other.kp && kd == other.kd;
        }
    };

    ControlNode() : Node("controller_node")
    {
        // 1. 初始化参数
        // 1. 动态获取包路径与 URDF
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
        std::string urdf = share_dir + "/" + rel_urdf_path;
        double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
        double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
        double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
        double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

        max_velocity_limit_ = this->declare_parameter("max_joint_velocity", 1.2);
        max_acceleration_limit_ = this->declare_parameter("max_acceleration_limit", 1.0);

        // 在 ControlNode 构造函数中添加：
        auto fc = this->declare_parameter("dynamics.friction.fc", std::vector<double>(5, 0.0));
        auto fv = this->declare_parameter("dynamics.friction.fv", std::vector<double>(5, 0.0));
        auto ratios = this->declare_parameter("dynamics.friction.GearRatio", std::vector<double>(5, 1.0));
        auto alpha = this->declare_parameter("dynamics.friction.alpha", 100.0);

        // 2. 初始化核心组件
        kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(urdf, arm2_task::RobotGeometry(l1, l2, l3, l4));
        dyn_manager_ = std::make_unique<arm2_task::DynamicsManager>(urdf);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // 3. 状态初始化
        current_q_ = Eigen::VectorXd::Zero(5);
        current_dq_ = Eigen::VectorXd::Zero(5);  // 必须添加
        current_tau_ = Eigen::VectorXd::Zero(5); // 必须添加
        command_q_ = Eigen::VectorXd::Zero(5);
        command_v_ = Eigen::VectorXd::Zero(5); // 建议添加
        command_a_ = Eigen::VectorXd::Zero(5); // 建议添加

        is_moving_.store(false);

        // 4. ROS 接口
        // 建议：使用 KeepLast(1)，强制丢弃所有旧指令，只处理最新的一条
        // 使用与驱动节点完全相同的定义
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        command_pub_ = this->create_publisher<robot_msgs::msg::RobotCommand>("/arm2/_lowCmd/command", qos);

        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "/arm2/_lowState/joint",
            qos,
            std::bind(&ControlNode::state_callback, this, std::placeholders::_1));

        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot_driver/ready",
            ready_qos,
            std::bind(&ControlNode::driver_ready_callback, this, std::placeholders::_1));

        mode_service_ = this->create_service<robot_msgs::srv::SetControllerMode>(
            "set_controller_mode", std::bind(&ControlNode::handle_mode_change, this, std::placeholders::_1, std::placeholders::_2));

        payload_service_ = this->create_service<robot_msgs::srv::GetPayloadEstimate>(
            "get_payload_estimate",
            std::bind(&ControlNode::handle_payload_estimate, this, std::placeholders::_1, std::placeholders::_2));

        action_server_ = rclcpp_action::create_server<MoveJoint>(
            this, "move_joint",
            std::bind(&ControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ControlNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ControlNode::handle_accepted, this, std::placeholders::_1));

        // 在 ControlNode 构造函数中：
        friction_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/debug/friction_torque", 10);

        control_timer_ = this->create_wall_timer(10ms, std::bind(&ControlNode::control_loop, this));

        dyn_manager_->initParams(fc, fv, ratios, alpha);
        load_all_gains();
        publish_static_camera_tf();
        RCLCPP_INFO(this->get_logger(), "Control Node Initialized.");
    }

private:
    // --- 轨迹段管理结构 ---
    struct TrajSegment
    {
        Eigen::VectorXd q_start, q_end;
        double t_acc, t_total;
        rclcpp::Time start_time;
    };

    // --- 成员变量 ---
    std::atomic<bool> is_moving_{false};
    bool is_blending_{false};
    bool has_data_{false};
    bool driver_ready_{false};
    bool system_ready_logged_{false};
    std::mutex data_mutex_;

    Eigen::VectorXd current_q_, current_dq_, current_tau_;
    Eigen::VectorXd command_q_, command_v_, command_a_;

    std::deque<Eigen::VectorXd> waypoint_queue_;
    std::shared_ptr<TrajSegment> active_segment_{nullptr};
    std::shared_ptr<TrajSegment> next_segment_{nullptr};

    double current_max_v_, current_max_a_, current_R_;
    double max_velocity_limit_, max_acceleration_limit_;

    PDGains current_gains_;
    std::map<std::string, PDGains> gains_map_;

    std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;
    std::unique_ptr<arm2_task::DynamicsManager> dyn_manager_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Service<robot_msgs::srv::SetControllerMode>::SharedPtr mode_service_;
    rclcpp::Service<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_service_;
    rclcpp_action::Server<MoveJoint>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr friction_pub_;

    // --- 内部函数原型 ---
    void load_all_gains();
    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg);
    void driver_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void control_loop();
    std::shared_ptr<TrajSegment> plan_trapezoid(const Eigen::VectorXd &start, const Eigen::VectorXd &end);

    // --- Action 回调 ---
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>)
    {
        if (!driver_ready_ || !has_data_)
        {
            RCLCPP_WARN(this->get_logger(), "Reject move goal: driver_ready=%d, has_state=%d", driver_ready_, has_data_);
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveJoint>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
    void handle_mode_change(const std::shared_ptr<robot_msgs::srv::SetControllerMode::Request> req,
                            std::shared_ptr<robot_msgs::srv::SetControllerMode::Response> res);
    void handle_payload_estimate(const std::shared_ptr<robot_msgs::srv::GetPayloadEstimate::Request> req,
                                 std::shared_ptr<robot_msgs::srv::GetPayloadEstimate::Response> res);
    void publish_command(const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &tau);
    void publish_static_camera_tf();
    /**
     * @brief 轨迹插值核心函数
     * @param seg 输入的轨迹段信息
     * @param t 当前相对于段起始点的时间(s)
     * @param q 输出：关节位置指令
     * @param v 输出：关节速度指令
     * @param a 输出：关节加速度指令
     */
    void compute_segment_state(const TrajSegment &seg, double t,
                               Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &a);
};

// ================= 实现部分 =================

void ControlNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    std::lock_guard<std::mutex> lock(data_mutex_);

    waypoint_queue_.clear();
    active_segment_ = nullptr;
    next_segment_ = nullptr;

    for (int i = 0; i < goal->num_points; ++i)
    {
        Eigen::VectorXd q_target(5);
        for (int j = 0; j < 5; ++j)
            q_target[j] = goal->joint_targets[i * 5 + j];
        waypoint_queue_.push_back(q_target);
    }

    current_max_v_ = (goal->max_velocity > 0) ? goal->max_velocity : max_velocity_limit_;
    current_max_a_ = (goal->max_acceleration > 0) ? goal->max_acceleration : max_acceleration_limit_;
    current_R_ = (goal->blend_radius > 0) ? goal->blend_radius : 0.15;

    if (!waypoint_queue_.empty())
    {
        active_segment_ = plan_trapezoid(current_q_, waypoint_queue_.front());
        waypoint_queue_.pop_front();
        is_moving_.store(true);
    }

    std::thread{std::bind(&ControlNode::execute_move, this, std::placeholders::_1), goal_handle}.detach();
}

void ControlNode::execute_move(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
{
    auto feedback = std::make_shared<MoveJoint::Feedback>();
    auto result = std::make_shared<MoveJoint::Result>();
    rclcpp::Rate loop_rate(10); // 10Hz 反馈足够

    while (rclcpp::ok())
    {
        // 1. 处理取消请求
        if (goal_handle->is_canceling())
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_moving_.store(false);
            waypoint_queue_.clear();
            active_segment_ = nullptr;
            next_segment_ = nullptr; // 也要清理 next
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // 2. 检查运动是否真正结束 (由 control_loop 将 active_segment_ 置空)
        if (!is_moving_.load())
        {
            break;
        }

        // 3. 计算并推送真实进度 (不再是固定的 0.5)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (active_segment_)
            {
                double t = (this->get_clock()->now() - active_segment_->start_time).seconds();
                // 进度 = 当前运行时间 / 总预设时间 (简化估算)
                feedback->progress = std::min(0.99, t / active_segment_->t_total);
            }
        }
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if (rclcpp::ok())
    {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Movement finished.");
    }
}

void ControlNode::state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    if (!msg || msg->motor_state.size() < 5)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ignore invalid robot state message, size=%zu",
                             msg ? msg->motor_state.size() : 0UL);
        return;
    }

    for (int i = 0; i < 5; ++i)
    {
        if (!msg->motor_state[i].valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Ignore stale/invalid robot state message.");
            return;
        }
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    for (int i = 0; i < 5; ++i)
    {
        current_q_[i] = msg->motor_state[i].q;
        current_dq_[i] = msg->motor_state[i].dq;
        current_tau_[i] = msg->motor_state[i].tau_est; // 解析反馈力矩用于估计
    }
    if (!has_data_)
    {
        command_q_ = current_q_;
        has_data_ = true;
    }

    // TF 广播逻辑
    pinocchio::SE3 T_w_l4 = kin_engine_->forwardKinematics(current_q_);
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

void ControlNode::driver_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    if (driver_ready_ != msg->data)
    {
        driver_ready_ = msg->data;
        system_ready_logged_ = false;
        RCLCPP_INFO(this->get_logger(), "Driver ready state changed: %s", driver_ready_ ? "true" : "false");
    }
}

// --- 【调试代码】使用 stringstream 进行类型转换 ---
auto to_str = [](const Eigen::VectorXd &v)
{
    std::stringstream ss;
    ss << v.transpose().format(Eigen::IOFormat(4, 0, ", ", "", "", ""));
    return ss.str();
};

void ControlNode::control_loop()
{
    if (!driver_ready_ || !has_data_)
        return;

    if (!system_ready_logged_)
    {
        RCLCPP_INFO(this->get_logger(), "Control Node handshake ready: driver online and first joint state received.");
        system_ready_logged_ = true;
    }

    // 局部变量，用于存储本次循环最终要发布的指令
    Eigen::VectorXd q_final(5), v_final(5), a_final(5);

    // 初始化：默认速度和加速度为0
    v_final.setZero();
    a_final.setZero();

    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (active_segment_)
        {
            // --- 1. 运动状态：计算插值 ---
            double t = (this->get_clock()->now() - active_segment_->start_time).seconds();
            compute_segment_state(*active_segment_, t, q_final, v_final, a_final);

            // --- 2. Blending 核心逻辑 ---
            double dist_to_target = (active_segment_->q_end - q_final).norm();

            // 满足以下条件开始混合：距离够近、队列有下个点、还没启动下个段
            if (dist_to_target < current_R_ && !waypoint_queue_.empty() && !next_segment_)
            {
                next_segment_ = plan_trapezoid(active_segment_->q_end, waypoint_queue_.front());
                waypoint_queue_.pop_front();
            }

            if (next_segment_)
            {
                double t_next = (this->get_clock()->now() - next_segment_->start_time).seconds();
                Eigen::VectorXd q2, v2, a2;
                compute_segment_state(*next_segment_, t_next, q2, v2, a2);

                // 矢量叠加实现平滑过渡
                q_final += (q2 - next_segment_->q_start);
                v_final += v2;
                a_final += a2;

                // 切换条件：当前段物理时间结束
                if (t >= active_segment_->t_total)
                {
                    active_segment_ = next_segment_;
                    next_segment_ = nullptr;
                }
            }
            else if (t >= active_segment_->t_total)
            {
                active_segment_ = nullptr; // 当前段完成
                if (waypoint_queue_.empty())
                {
                    is_moving_.store(false); // 通知 Action 线程结束
                }
            }

            // 【关键修改】在运动过程中，实时更新锁定的目标位置
            command_q_ = q_final;
        }
        else
        {
            // --- 3. 静止状态：锁定位置 ---
            // 不再使用 q_final = current_q_，而是保持最后一次的目标位置
            // 这样即便你在仿真中拖动，PID 也会产生拉力将臂拉回 command_q_
            q_final = command_q_;
        }
    }

    // --- 4. 动力学前馈计算 ---
    arm2_task::JointState des_state(5);
    des_state.q = q_final;
    des_state.dq = v_final;
    des_state.ddq = a_final;

    // 获取当前位姿下的重力/摩擦力前馈
    Eigen::VectorXd tau_f = dyn_manager_->computeFriction(des_state.dq);
    Eigen::VectorXd tau_ff = dyn_manager_->getFeedForwardTorque(des_state);

    // --- 【新增】发布摩擦力话题 ---
    std_msgs::msg::Float32MultiArray friction_msg;
    friction_msg.data.resize(5);
    for (int i = 0; i < 5; ++i)
    {
        friction_msg.data[i] = static_cast<float>(tau_f[i]);
    }
    friction_pub_->publish(friction_msg);

    if (active_segment_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "FF Input: Q:[%s] | DQ:[%s] | DDQ:[%s]",
                             to_str(q_final).c_str(),
                             to_str(v_final).c_str(),
                             to_str(a_final).c_str());

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "FF Output Torque: [%s]",
                             to_str(tau_ff).c_str());
    }

    publish_command(q_final, v_final, tau_ff);
}
void ControlNode::compute_segment_state(const TrajSegment &seg, double t,
                                        Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &a)
{
    double T = seg.t_total; // 轨迹总时长

    // 1. 时间边界保护
    if (t < 0)
        t = 0;
    if (t > T)
        t = T;

    // 2. 计算归一化时间参数 s = t / T (s范围 0~1)
    double s = t / T;
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;

    // 3. 五次多项式系数计算 (满足起止速度和加速度为0)
    // 位置系数: h(s) = 10s^3 - 15s^4 + 6s^5
    // 速度系数: h'(s) = (30s^2 - 60s^3 + 30s^4) / T
    // 加速度系数: h''(s) = (60s - 180s2 + 120s3) / T^2
    double pos_factor = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    double vel_factor = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
    double acc_factor = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);

    // 4. 计算各轴的瞬时状态
    Eigen::VectorXd delta_q = seg.q_end - seg.q_start;
    q = seg.q_start + delta_q * pos_factor; // 插值后的位置
    v = delta_q * vel_factor;               // 插值后的速度
    a = delta_q * acc_factor;               // 插值后的加速度
}

void ControlNode::load_all_gains()
{
    std::vector<std::string> modes = {"idle", "gravity_comp", "moving", "loaded"};
    for (const auto &mode : modes)
    {
        PDGains g;
        g.kp = this->declare_parameter("gains." + mode + ".kp", std::vector<double>(5, 0.0));
        g.kd = this->declare_parameter("gains." + mode + ".kd", std::vector<double>(5, 0.0));
        gains_map_[mode] = g;
    }
    current_gains_ = gains_map_["gravity_comp"];
}

void ControlNode::handle_mode_change(const std::shared_ptr<robot_msgs::srv::SetControllerMode::Request> request,
                                     std::shared_ptr<robot_msgs::srv::SetControllerMode::Response> response)
{
    if (!driver_ready_ || !has_data_)
    {
        response->success = false;
        response->message = "Controller not ready: waiting for driver or joint state";
        return;
    }

    if (gains_map_.count(request->mode))
    {
        current_gains_ = gains_map_[request->mode];
        response->success = true;
        response->message = "Mode switched to: " + request->mode;

        // --- 调试信息：打印当前关节 0 和 1 的 Kp 值，确认不是 0 ---
        RCLCPP_INFO(this->get_logger(),
                    "\033[1;34m[Mode Switch]\033[0m %s | Gains[0] Kp: %.1f, Kd: %.1f | Gains[1] Kp: %.1f, Kd: %.1f | Gains[2] Kp: %.1f, Kd: %.1f | Gains[3] Kp: %.1f, Kd: %.1f | Gains[4] Kp: %.1f, Kd: %.1f",
                    request->mode.c_str(), current_gains_.kp[0], current_gains_.kd[0], current_gains_.kp[1], current_gains_.kd[1], current_gains_.kp[2], current_gains_.kd[2], current_gains_.kp[3], current_gains_.kd[3], current_gains_.kp[4], current_gains_.kd[4]);
    }
    else
    {
        response->success = false;
        response->message = "Unknown mode: " + request->mode;
        RCLCPP_WARN(this->get_logger(), "\033[1;31m[Mode Error]\033[0m %s", response->message.c_str());
    }
}
void ControlNode::handle_payload_estimate(const std::shared_ptr<robot_msgs::srv::GetPayloadEstimate::Request> /*request*/,
                                          std::shared_ptr<robot_msgs::srv::GetPayloadEstimate::Response> response)
{
    if (!driver_ready_ || !has_data_)
    {
        response->success = false;
        response->message = "Waiting for driver ready and robot state data...";
        return;
    }

    // 构造当前实际状态
    arm2_task::JointState actual_state(5);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        actual_state.q = current_q_;
        actual_state.dq = current_dq_;
        actual_state.tau = current_tau_; // 实际反馈力矩
    }

    // 调用 DynamicsManager 内部算法
    double m_est = dyn_manager_->estimatePayloadMass(actual_state);

    response->mass = static_cast<float>(m_est);
    response->success = true;
    response->message = "Estimation success";
    RCLCPP_INFO(this->get_logger(), "\033[1;35m[Payload Service]\033[0m Estimated: %.3f kg", m_est);
}

void ControlNode::publish_static_camera_tf()
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
void ControlNode::publish_command(const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &tau)
{
    robot_msgs::msg::RobotCommand msg;
    for (int i = 0; i < 5; ++i)
    {
        robot_msgs::msg::MotorCommand m_cmd;
        m_cmd.q = q[i];
        m_cmd.dq = v[i];
        m_cmd.tau = tau[i];
        m_cmd.kp = static_cast<float>(current_gains_.kp[i]);
        m_cmd.kd = static_cast<float>(current_gains_.kd[i]);
        msg.motor_command.push_back(m_cmd);
    }
    command_pub_->publish(msg);
}

std::shared_ptr<ControlNode::TrajSegment> ControlNode::plan_trapezoid(const Eigen::VectorXd &start, const Eigen::VectorXd &end)
{
    auto seg = std::make_shared<TrajSegment>();
    seg->q_start = start;
    seg->q_end = end;
    seg->start_time = this->get_clock()->now();

    // 1. 计算各关节的位移绝对值
    Eigen::VectorXd delta_q = (end - start).cwiseAbs();

    // 2. 初始化最大所需时间
    double max_t = 0.5; // 设定一个最小时间阈值（如0.5秒），防止瞬移导致的冲击

    // 3. 遍历每个关节，根据速度和加速度限制计算该轴所需时间
    // 这里采用简化的梯形/五次多项式时间估算公式：
    // T > dist / v_max  (匀速限制)
    // T > sqrt(dist / a_max) * 2 (加速限制)
    for (int i = 0; i < 5; ++i)
    {
        if (delta_q[i] < 1e-4)
            continue; // 忽略微小位移

        // 基于最大速度限制的时间: t = distance / velocity
        double t_v = delta_q[i] / current_max_v_;

        // 基于最大加速度限制的时间:
        // 在起止速度为0的五次多项式中，峰值加速度约为 5.8 * dist / T^2
        // 为了简化计算且留有余量，我们取估算公式：
        double t_a = std::sqrt(delta_q[i] / current_max_a_) * 2.5;

        // 取该轴所需的最长时间
        double joint_min_t = std::max(t_v, t_a);

        // 同步所有轴：取所有轴中耗时最长的那个
        if (joint_min_t > max_t)
        {
            max_t = joint_min_t;
        }
    }

    seg->t_total = max_t;

    // 打印调试信息，确认规划出的时间是否合理
    RCLCPP_DEBUG(this->get_logger(), "Plan Trajectory: dist_norm=%.3f, time=%.3f s",
                 (end - start).norm(), max_t);

    return seg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
