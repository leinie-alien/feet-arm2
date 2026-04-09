#include <chrono>
#include <atomic>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/set_suction.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "arm2_task/common_units.hpp"
#include "arm2_task/kinematics_engine.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // 必须有，用于 rclcpp_action 命名空间
#include "robot_msgs/action/move_joint.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class TaskNode : public rclcpp::Node
{
public:
    using MoveJoint = robot_msgs::action::MoveJoint;
    using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;
    TaskNode() : Node("task_manager_node"), state_(arm2_task::TaskState::IDLE)
    {
        // 1. 初始化参数
        // 1. 动态获取包路径与 URDF
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
        std::string urdf = share_dir + "/" + rel_urdf_path;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 建议从参数服务器读取 L1-L4，确保与控制节点一致
        double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
        double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
        double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
        double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

        load_presets(); // 读取预设位姿

        max_v_ = this->declare_parameter("trajectory_planner.max_velocity", 1.0);
        max_a_ = this->declare_parameter("trajectory_planner.max_acceleration", 2.0);
        dist_threshold_ = this->declare_parameter("trajectory_planner.dist_threshold", 0.1);

        // 2. 初始化逆解引擎
        kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(
            urdf, arm2_task::RobotGeometry(l1, l2, l3, l4));
        target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/task/target_pose", 10);
        // 定义与驱动节点一致的 QoS 策略
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();

        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "/arm2/_lowState/joint",
            qos, // 使用上面定义的 qos
            [this](const robot_msgs::msg::RobotState::SharedPtr msg)
            {
                if (!msg || msg->motor_state.size() < 5)
                {
                    return;
                }
                for (int i = 0; i < 5; ++i)
                {
                    if (!msg->motor_state[i].valid)
                    {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            1000,
                            "Ignore stale/invalid robot state message.");
                        return;
                    }
                }
                std::lock_guard<std::mutex> lock(mtx_);
                // 注意：原代码中 q_current_ 被反复赋值为 Zero，建议只在初始化或异常时清零
                if (q_current_.size() != 5)
                    q_current_ = Eigen::VectorXd::Zero(5);

                for (int i = 0; i < 5; ++i)
                {
                    q_current_[i] = msg->motor_state[i].q;
                }
                has_robot_data_ = true;
            });

        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot_driver/ready",
            ready_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!msg)
                {
                    return;
                }
                driver_ready_.store(msg->data);
            });

        pick_client_ = this->create_client<robot_msgs::srv::GetPickPos>("get_pick_pos");
        suction_client_ = this->create_client<robot_msgs::srv::SetSuction>("set_suction");
        mode_client_ = this->create_client<robot_msgs::srv::SetControllerMode>("set_controller_mode");
        payload_client_ = this->create_client<robot_msgs::srv::GetPayloadEstimate>("get_payload_estimate");
        move_joint_client_ = rclcpp_action::create_client<MoveJoint>(this, "move_joint");

        RCLCPP_INFO(this->get_logger(), "Task Node Started.");
    }

    ~TaskNode()
    {
        if (task_thread_.joinable())
            task_thread_.join();
    }

    // 在 TaskNode 类中添加
    void start()
    {
        task_thread_ = std::thread(&TaskNode::run_task_sequence, this);
    }

    // 1. 发送复位请求
    void request_reset()
    {
        if (presets_.count("reset"))
        {
            auto goal_q = presets_["reset"];
            send_move_goal({goal_q});
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Preset 'reset' not found!");
        }
    }

    void look_out_action(geometry_msgs::msg::Pose target_world_pose)
    {
        if (presets_.count("look_out"))
        {
            auto goal_q = presets_["look_out"];
            goal_q[0] = std::atan2(target_world_pose.position.y, target_world_pose.position.x); // 重新计算底座旋转角
            auto link4_pose = kin_engine_->forwardKinematics(goal_q);
            goal_q[4] = std::atan2(target_world_pose.position.z - link4_pose.translation().z(),
                                   std::sqrt(std::pow(target_world_pose.position.x - link4_pose.translation().x(), 2) +
                                             std::pow(target_world_pose.position.y - link4_pose.translation().y(), 2))); // 计算新的 Pitch 角

            send_move_goal({goal_q});
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Preset 'look_out' not found!");
        }
    }

    void load_action()
    {
        if (presets_.count("load"))
        {
            auto goal_q = presets_["load"];
            send_move_goal({goal_q});
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Preset 'load' not found!");
        }
    }

    void grasp_action(geometry_msgs::msg::Pose target_world_pose)
    {
        Eigen::VectorXd q_ik(5);
        double target_pitch = -1.57; // 或者根据需求设定俯仰角

        // 1. 手动转换类型：将 ROS Point 转为 Eigen::Vector3d
        Eigen::Vector3d target_p(
            target_world_pose.position.x,
            target_world_pose.position.y,
            target_world_pose.position.z + 0.2);

        // 2. 调用 solveIK (匹配第 34 行定义的 3 参数版本)
        if (kin_engine_->solveIK(target_p, target_pitch, q_ik))
        {
            send_move_goal({q_ik});
            ik_success_ = true;
        }
        else
        {
            ik_success_ = false;
            RCLCPP_ERROR(this->get_logger(), "IK failed for grasp action!");
        }
    }

private:
    bool wait_for_system_ready()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot_driver_socket readiness...");
        while (rclcpp::ok() && !driver_ready_.load())
        {
            rclcpp::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for first robot joint state...");
        while (rclcpp::ok() && !has_robot_data_)
        {
            rclcpp::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for control services and action server...");
        while (rclcpp::ok())
        {
            const bool mode_ready = mode_client_->wait_for_service(500ms);
            const bool payload_ready = payload_client_->wait_for_service(500ms);
            const bool suction_ready = suction_client_->wait_for_service(500ms);
            const bool action_ready = move_joint_client_->wait_for_action_server(500ms);

            if (mode_ready && payload_ready && suction_ready && action_ready)
            {
                return true;
            }
        }

        return false;
    }

    void load_presets()
    {
        // 定义想要加载的动作名称列表
        std::vector<std::string> preset_names = {"reset", "look_out", "load"};

        for (const auto &name : preset_names)
        {
            // 1. 从参数服务器读取角度制数组
            // 默认值设为全 0
            auto angles_deg = this->declare_parameter("presets." + name, std::vector<double>(5, 0.0));

            // 2. 转换为 Eigen::VectorXd 并进行弧度转换
            Eigen::VectorXd q_rad(5);
            for (int i = 0; i < 5; ++i)
            {
                // 公式：弧度 = 角度 * M_PI / 180.0
                q_rad[i] = angles_deg[i] * M_PI / 180.0;
            }

            // 3. 预存到 Map 中供后续使用
            presets_[name] = q_rad;

            RCLCPP_INFO(this->get_logger(), "Loaded preset '%s' (converted to radians)", name.c_str());
        }
    }
    // 发送关节空间目标
    void send_move_goal(const std::vector<Eigen::VectorXd> &q_waypoints)
    {
        if (!move_joint_client_->wait_for_action_server(10s))
            return;

        auto goal_msg = MoveJoint::Goal();
        goal_msg.max_velocity = max_v_; // 从 params.yaml 读取
        goal_msg.max_acceleration = max_a_;
        goal_msg.blend_radius = dist_threshold_; // 混合半径 R
        goal_msg.num_points = q_waypoints.size();

        for (const auto &q : q_waypoints)
        {
            for (int i = 0; i < 5; ++i)
                goal_msg.joint_targets.push_back(q[i]);
        }

        auto send_goal_options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto &result)
        {
            action_finished_ = true; // 触发状态机跳转
        };

        move_joint_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // 新增：支持单个点的重载版本
    void send_move_goal(const Eigen::VectorXd &q_single)
    {
        // 将单个点包装成 vector 然后调用原函数
        std::vector<Eigen::VectorXd> waypoints{q_single};
        send_move_goal(waypoints);
    }
    /**
     * @brief 坐标获取服务请求：使用 TF2 处理感知延迟并转换至世界坐标系
     */
    bool call_pick_service_sync(const std::string &object_name, geometry_msgs::msg::Pose *out_pose)
    {
        // 1. 检查服务是否存在
        if (!pick_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return false;
        }

        // 2. 构造请求
        auto request = std::make_shared<robot_msgs::srv::GetPickPos::Request>();
        request->object_name = object_name;

        // 3. 发送异步请求，但立即获取 future 对象
        auto result_future = pick_client_->async_send_request(request);

        // 4. 【关键】阻塞等待结果
        // 使用 wait_for 等待，防止服务器挂掉导致永久死锁
        if (result_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out");
            return false;
        }

        // 5. 获取响应结果（此时已确保数据到达）
        auto response = result_future.get();

        if (response->pick_pose.header.frame_id.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received empty pose data");
            return false;
        }

        try
        {
            rclcpp::Time capture_time = response->pick_pose.header.stamp;

            // TF 变换依然需要等待数据进入缓存
            if (!tf_buffer_->canTransform("world", response->pick_pose.header.frame_id,
                                          capture_time, tf2::durationFromSec(0.5)))
            {
                RCLCPP_ERROR(this->get_logger(), "TF back-tracking failed");
                return false;
            }

            geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
                "world", response->pick_pose.header.frame_id, capture_time);

            // 执行转换并直接填充结果
            if (out_pose != nullptr)
            {
                tf2::doTransform(response->pick_pose.pose, *out_pose, t_stamped);
                RCLCPP_INFO(this->get_logger(), "Sync Pick Success: x=%.3f y=%.3f  z=%.3f", out_pose->position.x, out_pose->position.y, out_pose->position.z);
            }
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF2 Error: %s", ex.what());
            return false;
        }
    }

    int request_payload_estimate()
    {
        // 1. 等待服务可用
        if (!payload_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Service [GetPayloadEstimate] not available.");
            return 0;
        }

        // 2. 创建请求
        auto request = std::make_shared<robot_msgs::srv::GetPayloadEstimate::Request>();

        // 3. 发送异步请求并获取 future 对象
        auto result_future = payload_client_->async_send_request(request);

        // 4. 同步等待响应 (设置超时防止永久阻塞)
        // 注意：如果此节点是单线程 Executor 且在回调中调用，此处会死锁
        std::future_status status = result_future.wait_for(std::chrono::seconds(2));

        if (status == std::future_status::ready)
        {
            auto response = result_future.get();
            if (response && response->success)
            {
                // 5. 加锁并更新数据
                std::lock_guard<std::mutex> lock(mtx_);
                this->last_estimated_mass_ = response->mass;
                this->is_mass_updated_ = true;
                return 1; // 成功获取并更新
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service responded with failure.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out or failed to get response.");
        }

        return 0; // 失败返回 0
    }

    // --- 已定义的模式请求函数 ---
    int request_mode_switch(const std::string &mode_name)
    {
        if (!mode_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Mode switch service not available");
            return 0;
        }

        auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
        request->mode = mode_name;

        mode_client_->async_send_request(request,
                                         [this, mode_name](rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedFuture future)
                                         {
                                             auto response = future.get();
                                             if (response->success)
                                             {
                                                 RCLCPP_INFO(this->get_logger(), "\033[1;36m[Mode Success]\033[0m Controller switched to: %s", mode_name.c_str());
                                             }
                                         });
        return 1;
    }

    int set_suction(bool activate)
    {
        if (!suction_client_->wait_for_service(std::chrono::seconds(1)))
            return 0;

        auto request = std::make_shared<robot_msgs::srv::SetSuction::Request>();
        request->activate = activate;

        // 修正：显式指定 SharedFuture 类型避免编译推导错误
        suction_client_->async_send_request(request,
                                            [this, activate](rclcpp::Client<robot_msgs::srv::SetSuction>::SharedFuture future)
                                            {
                                                auto response = future.get();
                                                if (response->success)
                                                {
                                                    RCLCPP_INFO(this->get_logger(), "Suction %s", activate ? "ON" : "OFF");
                                                }
                                            });
        return 1;
    }

    void run_task_sequence()
    {
        if (!wait_for_system_ready())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Task Node [Joint Control Mode] Ready.");

        while (rclcpp::ok())
        {
            std::cout << "\n========== 任务控制面板 (关节空间/IK) ==========" << std::endl;
            std::cout << "1: 执行 Step 1 (重置到预设 IDLE)" << std::endl;
            std::cout << "2: 执行 Step 2 (识别并动态移动到 LOOKOUT)" << std::endl;
            std::cout << "3: 执行 Step 3 (执行 IK 抓取 GRASP)" << std::endl;
            std::cout << "4: 执行 Step 4 (负载估计与模式切换)" << std::endl;
            std::cout << "5: 执行 Step 5 (移动到预设 LOAD 点)" << std::endl;
            std::cout << "0: 退出程序" << std::endl;
            std::cout << "请输入指令数字: ";

            int input_cmd;
            if (!(std::cin >> input_cmd))
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            geometry_msgs::msg::Pose target_pose;
            Eigen::VectorXd q_(5);
            switch (input_cmd)
            {
            case 1:
                RCLCPP_INFO(this->get_logger(), ">>> Step 1: Resetting to IDLE...");
                set_suction(false);
                request_mode_switch("moving");
                request_reset(); // 使用预设的 "reset" 位姿

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                request_mode_switch("idle");
                RCLCPP_INFO(this->get_logger(), "Step 1 完成。");
                break;

            case 2:
                RCLCPP_INFO(this->get_logger(), ">>> Step 2: Dynamic Looking...");
                request_mode_switch("moving");

                q_ << 0, 160, -130, 40, 0;
                for (int i = 0; i < q_.size(); ++i)
                {
                    // 角度转弧度公式：弧度 = 角度 * (π / 180)
                    q_(i) = q_(i) * (M_PI / 180.0);
                }
                send_move_goal(q_);

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                RCLCPP_INFO(this->get_logger(), "Step 2 完成。");
                break;

            case 3:
                RCLCPP_INFO(this->get_logger(), ">>> Step 3: IK Grasping...");
                request_mode_switch("moving");

                q_ << 180, 90, -90, -90, 0;
                for (int i = 0; i < q_.size(); ++i)
                {
                    // 角度转弧度公式：弧度 = 角度 * (π / 180)
                    q_(i) = q_(i) * (M_PI / 180.0);
                }
                send_move_goal(q_);

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                RCLCPP_INFO(this->get_logger(), "Step 3 完成。");
                break;

            case 4:
                RCLCPP_INFO(this->get_logger(), ">>> Step 4: IK Grasping...");
                request_mode_switch("moving");

                q_ << 90, 0, 0, 0, -90;
                for (int i = 0; i < q_.size(); ++i)
                {
                    // 角度转弧度公式：弧度 = 角度 * (π / 180)
                    q_(i) = q_(i) * (M_PI / 180.0);
                }
                send_move_goal(q_);

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                RCLCPP_INFO(this->get_logger(), "Step 4 完成。");
                break;

            case 5:
                RCLCPP_INFO(this->get_logger(), ">>> Step 5: IK Grasping...");
                request_mode_switch("moving");

                q_ << 0, 0, 0, 0, -90;
                for (int i = 0; i < q_.size(); ++i)
                {
                    // 角度转弧度公式：弧度 = 角度 * (π / 180)
                    q_(i) = q_(i) * (M_PI / 180.0);
                }
                send_move_goal(q_);

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                RCLCPP_INFO(this->get_logger(), "Step 4 完成。");
                break;

            case 6:
            { // <--- 在这里添加左大括号
                RCLCPP_INFO(this->get_logger(), ">>> Step 6: IK Grasping...");
                request_mode_switch("moving");
                
                geometry_msgs::msg::Pose target_pose;
                if (call_pick_service_sync("box", &target_pose)) 
                {
                    grasp_action(target_pose);
                }

                while (rclcpp::ok() && !action_finished_)
                {
                    rclcpp::sleep_for(50ms);
                }
                action_finished_ = false;
                RCLCPP_INFO(this->get_logger(), "Step 6 完成。");
                break;
            } // <--- 在这里添加右大括号
            
            case 0:
                RCLCPP_INFO(this->get_logger(), "收到退出指令。");
                rclcpp::shutdown();
                return;

            default:
                RCLCPP_WARN(this->get_logger(), "无效指令: %d", input_cmd);
                break;
            }
        }
    }

    // 成员变量
    arm2_task::TaskState state_;
    bool has_robot_data_ = false;
    Eigen::VectorXd q_current_;
    geometry_msgs::msg::Pose target_world_pose_;
    std::mutex mtx_;
    std::atomic<bool> driver_ready_{false};

    // 重量估计相关变量
    float last_estimated_mass_ = 0.0f;
    double max_v_ = 1.0;
    double max_a_ = 1.0;
    double dist_threshold_ = 0.05;
    std::atomic<bool> ik_success_ = true;
    std::atomic<bool> is_mass_updated_ = false;
    std::atomic<bool> is_pose_updated_ = false;
    std::atomic<bool> is_action_running_ = false; // 新增标志位，跟踪 Action 执行状态
    std::atomic<bool> action_finished_ = false;   // 新增：用于通知状态机切换

    std::map<std::string, Eigen::VectorXd> presets_; // 预设位姿
    std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
    rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
    rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
    rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client_;
    rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client_;

    std::atomic<bool> is_running_{true}; // 用于安全退出线程
    std::thread task_thread_;            // 添加这一行
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode>();

    // 使用多线程执行器，允许 Timer 和 Action 回调并行
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    node->start();
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
