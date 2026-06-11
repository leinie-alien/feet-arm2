#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "arm2_task/common_units.hpp"
#include "arm2_task/kinematics_engine.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/get_place_pos.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/set_suction.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TaskNode : public rclcpp::Node
{
public:
  using MoveJoint = robot_msgs::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;

  TaskNode()
      : Node("task_manager_node"),
        state_(arm2_task::TaskState::IDLE)
  {
    // ── URDF & Kinematics ──────────────────────────────────────────────────
    const std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
    const std::string rel_urdf = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
    const std::string urdf = share_dir + "/" + rel_urdf;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
    const double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
    const double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
    const double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

    kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(
        urdf, arm2_task::RobotGeometry(l1, l2, l3, l4));

    // ── Trajectory Parameters ──────────────────────────────────────────────
    load_presets();
    max_v_ = this->declare_parameter("trajectory_planner.max_velocity", 1.0);
    max_a_ = this->declare_parameter("trajectory_planner.max_acceleration", 2.0);
    dist_threshold_ = this->declare_parameter("trajectory_planner.dist_threshold", 0.1);

    // ── Task Parameters ────────────────────────────────────────────────────
    require_payload_service_ = this->declare_parameter("task.require_payload_service", false);
    require_suction_service_ = this->declare_parameter("task.require_suction_service", false);

    // Grasp / place parameters
    grasp_pitch_ = this->declare_parameter("task_step6.grasp_pitch", -1.57);
    tool_pitch_offset_ = this->declare_parameter("task_step6.tool_pitch_offset", 0.0);
    tool_yaw_offset_ = this->declare_parameter("task_step6.tool_yaw_offset", 0.0);
    object_height_ = this->declare_parameter("task_step6.object_height", 0.05);
    pre_grasp_offset_ = this->declare_parameter("task_step6.pre_grasp_offset", 0.10);
    tool_offset_x_ = this->declare_parameter("task_step6.tool_offset_x", 0.0);
    tool_offset_y_ = this->declare_parameter("task_step6.tool_offset_y", 0.0);
    tool_offset_z_ = this->declare_parameter("task_step6.tool_offset_z", 0.0);
    step6_pick_object_name_ = this->declare_parameter(
        "task_step6.pick_object_name", std::string("box"));
    step6_use_mock_target_ = this->declare_parameter("task_step6.use_mock_target", false);
    step6_mock_x_ = this->declare_parameter("task_step6.mock_x", 0.35);
    step6_mock_y_ = this->declare_parameter("task_step6.mock_y", 0.0);
    step6_mock_z_ = this->declare_parameter("task_step6.mock_z", 0.12);

    pre_place_offset_ = this->declare_parameter("task_place.pre_place_offset", 0.12);
    place_retreat_offset_ = this->declare_parameter("task_place.retreat_offset", 0.15);
    tool_tip_length_ = this->declare_parameter("task_step6.tool_tip_length", 0.0);

    // Place-frame parameters (狗头相机放置)
    place_frame_hover_height_  = this->declare_parameter("task_place_frame.hover_height",   0.25);
    place_frame_contact_offset_= this->declare_parameter("task_place_frame.contact_offset",  0.0);
    place_frame_name_          = this->declare_parameter("task_place_frame.frame_name",      std::string("target_frame"));
    place_frame_roll_sign_     = this->declare_parameter("task_place_frame.roll_sign",        1.0);
    place_frame_use_mock_      = this->declare_parameter("task_place_frame.use_mock_target", false);
    place_frame_mock_x_        = this->declare_parameter("task_place_frame.mock_x",           0.35);
    place_frame_mock_y_        = this->declare_parameter("task_place_frame.mock_y",           0.0);
    place_frame_mock_z_        = this->declare_parameter("task_place_frame.mock_z",           0.0);
    place_frame_mock_yaw_      = this->declare_parameter("task_place_frame.mock_yaw",         0.0);

    // Stack parameters (箱子叠放，task_stack)
    stack_hover_height_   = this->declare_parameter("task_stack.hover_height",    0.05);
    stack_contact_offset_ = this->declare_parameter("task_stack.contact_offset",  0.25);
    stack_service_name_   = this->declare_parameter("task_stack.stack_service",   std::string("get_stack_pos"));
    stack_roll_sign_      = this->declare_parameter("task_stack.roll_sign",        1.0);
    stack_use_mock_       = this->declare_parameter("task_stack.use_mock_target", false);
    stack_mock_x_         = this->declare_parameter("task_stack.mock_x",           0.35);
    stack_mock_y_         = this->declare_parameter("task_stack.mock_y",           0.0);
    stack_mock_z_         = this->declare_parameter("task_stack.mock_z",           0.1);
    stack_mock_yaw_       = this->declare_parameter("task_stack.mock_yaw",         0.0);

    // Phase-2 alignment parameters
    align_threshold_ = this->declare_parameter("visual_align.align_threshold", 0.005);
    align_max_iters_ = this->declare_parameter("visual_align.max_iters", 5);

    // ── Publishers & Subscribers ───────────────────────────────────────────
    target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/task/target_pose", 10);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
        "/arm2/_lowState/joint",
        state_qos,
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
              return;
            }
          }
          std::lock_guard<std::mutex> lock(mtx_);
          if (q_current_.size() != 5)
          {
            q_current_ = Eigen::VectorXd::Zero(5);
            dq_current_ = Eigen::VectorXd::Zero(5);
          }
          for (int i = 0; i < 5; ++i)
          {
            q_current_[i] = msg->motor_state[i].q;
            dq_current_[i] = msg->motor_state[i].dq;
          }
          has_robot_data_ = true;
        });

    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot_driver/ready",
        ready_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (msg)
          {
            driver_ready_.store(msg->data);
          }
        });

    // ── Remote Control (optional) ──────────────────────────────────────────
    remote_mode_ = this->declare_parameter("task.remote_mode", false);
    if (remote_mode_)
    {
      status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm/status", 10);
      cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/arm/cmd", 10,
          [this](const std_msgs::msg::String::SharedPtr msg)
          {
            if (!msg) return;
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            pending_cmd_ = msg->data;
            cmd_cv_.notify_one();
          });
      RCLCPP_INFO(this->get_logger(), "Remote control mode enabled.");
    }

    // ── Service & Action Clients ───────────────────────────────────────────
    pick_client_ = this->create_client<robot_msgs::srv::GetPickPos>("get_pick_pos");
    place_client_ = this->create_client<robot_msgs::srv::GetPlacePos>("get_place_pos");
    stack_client_ = this->create_client<robot_msgs::srv::GetPlacePos>("get_stack_pos");
    suction_client_ = this->create_client<robot_msgs::srv::SetSuction>("set_suction");
    mode_client_ = this->create_client<robot_msgs::srv::SetControllerMode>("set_controller_mode");
    payload_client_ = this->create_client<robot_msgs::srv::GetPayloadEstimate>("get_payload_estimate");
    move_joint_client_ = rclcpp_action::create_client<MoveJoint>(this, "move_joint");

    RCLCPP_INFO(this->get_logger(), "Task Node Started.");
  }

  ~TaskNode()
  {
    is_running_.store(false);
    if (task_thread_.joinable())
    {
      task_thread_.join();
    }
  }

  void start()
  {
    if (remote_mode_)
    {
      task_thread_ = std::thread(&TaskNode::run_remote_control, this);
    }
    else
    {
      task_thread_ = std::thread(&TaskNode::run_task_sequence, this);
    }
  }

private:
  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 1 — Infrastructure (arm 原有，已实机验证，不得修改)
  // ═══════════════════════════════════════════════════════════════════════════

  bool wait_for_system_ready()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for driver readiness...");
    while (rclcpp::ok() && is_running_.load() && !driver_ready_.load())
    {
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for first robot joint state...");
    while (rclcpp::ok() && is_running_.load() && !has_robot_data_.load())
    {
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for control services and action server...");
    while (rclcpp::ok() && is_running_.load())
    {
      const bool mode_ready = mode_client_->wait_for_service(500ms);
      const bool payload_ready = !require_payload_service_ || payload_client_->wait_for_service(500ms);
      const bool suction_ready = !require_suction_service_ || suction_client_->wait_for_service(500ms);
      const bool action_ready = move_joint_client_->wait_for_action_server(500ms);

      if (mode_ready && payload_ready && suction_ready && action_ready)
      {
        if (!require_suction_service_ && !suction_client_->wait_for_service(100ms))
        {
          RCLCPP_INFO(this->get_logger(),
                      "Optional suction service [set_suction] not available; suction commands will be skipped.");
        }
        return true;
      }
    }
    return false;
  }

  void load_presets()
  {
    const std::vector<std::string> preset_names = {"reset", "look_out", "load", "carry"};
    for (const auto &name : preset_names)
    {
      const auto angles_deg =
          this->declare_parameter("presets." + name, std::vector<double>(5, 0.0));
      Eigen::VectorXd q_rad(5);
      for (int i = 0; i < 5; ++i)
      {
        q_rad[i] = angles_deg[i] * M_PI / 180.0;
      }
      presets_[name] = q_rad;
      RCLCPP_INFO(this->get_logger(), "Loaded preset '%s'", name.c_str());
    }
  }

  /** 将角度归一化到 [-π, π)。*/
  static double normalize_angle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  /**
   * @brief 从 TF 中推算物体的偏航角（相对于 world 系）。
   *        在俯视场景下，这等价于物体长轴相对于基座 X 轴的夹角。
   *        实现：用当前 joint_0 作为 base_yaw，物体 world XY 方向作为 object_yaw，
   *        tool_roll = object_yaw - base_yaw（让工具对准物体方向）。
   *        若无需特定对齐方向则返回 0。
   */
  double get_object_yaw(const geometry_msgs::msg::Pose &object_world)
  {
    // object 与 world 原点的方位角就是物体相对基座的方向
    const double object_yaw = std::atan2(
        object_world.position.y, object_world.position.x);

    // do_look_out 和 solveIK 都将 joint_0 设为 atan2(y,x)，
    // 以此作为 base_yaw，避免依赖异步 q_current_ 读取时机（action 完成时
    // 话题可能尚未更新，导致 base_yaw 读到旧值从而 roll 偏转一个大角度后 IK 失败）。
    const double base_yaw = object_yaw;
    return normalize_angle(object_yaw - base_yaw); // = 0.0，吸盘默认不额外旋转
  }

  /**
   * @brief 从视觉返回的 world 系 orientation 中提取箱子边缘相对于 joint_0 的 roll 偏转量。
   *        R = [U | V | normal]，U 轴为箱子主边方向。tf2::doTransform 将 Pose 转到 world 系后，
   *        四元数第一列即为 U 轴在 world 的方向。利用矩形 180° 对称性归一化到 [-π/2, π/2]。
   *        若 orientation 为 identity（手动输入或视觉未提供朝向），返回 0。
   */
  double get_box_edge_roll(const geometry_msgs::msg::Pose &world_pose)
  {
    const auto &q = world_pose.orientation;

    if (std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 &&
        std::abs(q.z) < 1e-6 && std::abs(q.w - 1.0) < 1e-6)
    {
      RCLCPP_INFO(this->get_logger(), "[box_edge_roll] identity orientation, roll=0");
      return 0.0;
    }

    const double ux = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double uy = 2.0 * (q.x * q.y + q.z * q.w);
    const double edge_yaw = std::atan2(uy, ux);

    const double base_yaw = std::atan2(world_pose.position.y, world_pose.position.x);
    double roll = normalize_angle(edge_yaw - base_yaw);

    if (roll >  M_PI / 2.0) roll -= M_PI;
    if (roll < -M_PI / 2.0) roll += M_PI;

    RCLCPP_INFO(this->get_logger(),
                "[box_edge_roll] edge_yaw=%.3f base_yaw=%.3f roll=%.3f rad",
                edge_yaw, base_yaw, roll);
    return roll;
  }

  /**
   * @brief 发送轨迹目标（多路点）。返回 false 表示 action server 不可用或 goal 被拒绝。
   */
  bool send_move_goal(const std::vector<Eigen::VectorXd> &q_waypoints)
  {
    if (!move_joint_client_->wait_for_action_server(10s))
    {
      is_action_running_ = false;
      action_finished_ = true;
      {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        last_action_succeeded_ = false;
        last_action_message_ = "move_joint action server not available.";
      }
      RCLCPP_ERROR(this->get_logger(), "move_joint action server not available.");
      return false;
    }

    MoveJoint::Goal goal_msg;
    goal_msg.max_velocity = max_v_;
    goal_msg.max_acceleration = max_a_;
    goal_msg.blend_radius = dist_threshold_;
    goal_msg.num_points = static_cast<int32_t>(q_waypoints.size());
    for (const auto &q : q_waypoints)
    {
      for (int i = 0; i < 5; ++i)
      {
        goal_msg.joint_targets.push_back(q[i]);
      }
    }

    auto send_goal_options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();
    action_finished_ = false;
    is_action_running_ = true;
    {
      std::lock_guard<std::mutex> lock(action_result_mutex_);
      last_action_succeeded_ = false;
      last_action_message_.clear();
    }

    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandleMoveJoint> goal_handle)
    {
      if (!goal_handle)
      {
        is_action_running_ = false;
        action_finished_ = true;
        {
          std::lock_guard<std::mutex> lock(action_result_mutex_);
          last_action_succeeded_ = false;
          last_action_message_ = "move_joint goal rejected by action server.";
        }
        RCLCPP_ERROR(this->get_logger(), "move_joint goal was rejected by the action server.");
      }
    };

    send_goal_options.result_callback =
        [this](const GoalHandleMoveJoint::WrappedResult &result)
    {
      is_action_running_ = false;
      action_finished_ = true;
      bool succeeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      std::string msg;
      if (result.result)
      {
        succeeded = succeeded && result.result->success;
        msg = result.result->message;
      }
      else if (!succeeded)
      {
        msg = "Action finished without a result payload.";
      }
      if (msg.empty())
      {
        msg = succeeded ? "success" : "move_joint action failed.";
      }
      {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        last_action_succeeded_ = succeeded;
        last_action_message_ = msg;
      }
      if (!succeeded)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "move_joint action failed. code=%d message=%s",
                     static_cast<int>(result.code), msg.c_str());
      }
    };

    move_joint_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  bool send_move_goal(const Eigen::VectorXd &q_single)
  {
    return send_move_goal(std::vector<Eigen::VectorXd>{q_single});
  }

  /**
   * @brief 阻塞等待当前 action 完成。带超时，返回 false 表示失败/超时。
   */
  bool wait_for_action_completion(std::chrono::seconds timeout = std::chrono::seconds(30))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && is_running_.load() && !action_finished_)
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        is_action_running_ = false;
        {
          std::lock_guard<std::mutex> lock(action_result_mutex_);
          last_action_succeeded_ = false;
          last_action_message_ = "Timed out waiting for move_joint action.";
        }
        RCLCPP_ERROR(this->get_logger(),
                     "Timed out waiting for move_joint action after %lds.", timeout.count());
        return false;
      }
      rclcpp::sleep_for(50ms);
    }

    if ((!rclcpp::ok() || !is_running_.load()) && !action_finished_)
    {
      std::lock_guard<std::mutex> lock(action_result_mutex_);
      last_action_succeeded_ = false;
      last_action_message_ = "Task loop stopped before action finished.";
      return false;
    }

    bool succeeded;
    std::string msg;
    {
      std::lock_guard<std::mutex> lock(action_result_mutex_);
      succeeded = last_action_succeeded_;
      msg = last_action_message_;
    }
    action_finished_ = false;
    if (!succeeded)
    {
      RCLCPP_ERROR(this->get_logger(), "move_joint action reported failure: %s",
                   msg.empty() ? "unknown error" : msg.c_str());
    }
    return rclcpp::ok() && succeeded;
  }

  /**
   * @brief 同步调用 get_pick_pos 服务，将结果 TF 变换到 world 坐标系。
   *        此为 arm 已验证的感知接口，Phase 2 对齐闭环也复用此函数。
   */
  bool call_pick_service_sync(const std::string &object_name,
                              geometry_msgs::msg::Pose *out_pose)
  {
    if (!pick_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "get_pick_pos service not available");
      return false;
    }

    auto request = std::make_shared<robot_msgs::srv::GetPickPos::Request>();
    request->object_name = object_name;

    auto result_future = pick_client_->async_send_request(request);
    if (result_future.wait_for(15s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "get_pick_pos service call timed out (waited 15 seconds)");
      return false;
    }

    const auto response = result_future.get();
    if (!response)
    {
      RCLCPP_ERROR(this->get_logger(), "get_pick_pos service call returned null response");
      return false;
    }
    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "get_pick_pos service returned failure (success=false)");
      return false;
    }

    const std::string frame_id = response->pick_pose.header.frame_id;
    if (frame_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "get_pick_pos returned empty frame_id");
      return false;
    }

    try
    {
      // 用最新可用变换（TimePointZero），避免仿真 1s 延迟导致时间戳过旧
      geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
          "world", frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

      if (out_pose != nullptr)
      {
        tf2::doTransform(response->pick_pose.pose, *out_pose, t_stamped);
        RCLCPP_INFO(this->get_logger(),
                    "get_pick_pos frame=%s -> world=(%.3f, %.3f, %.3f)",
                    frame_id.c_str(),
                    out_pose->position.x, out_pose->position.y, out_pose->position.z);
      }
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF2 error: %s", ex.what());
      return false;
    }
  }


  bool call_place_service_sync(const std::string &frame_name,
                               geometry_msgs::msg::Pose *out_pose)
  {
    if (!place_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "get_place_pos service not available");
      return false;
    }

    auto request = std::make_shared<robot_msgs::srv::GetPlacePos::Request>();
    request->frame_name = frame_name;

    auto result_future = place_client_->async_send_request(request);
    if (result_future.wait_for(15s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "get_place_pos service call timed out (waited 15 seconds)");
      return false;
    }

    const auto response = result_future.get();
    if (!response)
    {
      RCLCPP_ERROR(this->get_logger(), "get_place_pos service call returned null response");
      return false;
    }
    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "get_place_pos service returned failure (success=false)");
      return false;
    }

    const std::string frame_id = response->place_pose.header.frame_id;
    if (frame_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "get_place_pos returned empty frame_id");
      return false;
    }

    try
    {
      geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
          "world", frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

      if (out_pose != nullptr)
      {
        geometry_msgs::msg::PoseStamped pose_world;
        tf2::doTransform(response->place_pose, pose_world, t_stamped);
        *out_pose = pose_world.pose;
        RCLCPP_INFO(this->get_logger(),
                    "get_place_pos frame=%s -> world=(%.3f, %.3f, %.3f)",
                    frame_id.c_str(),
                    out_pose->position.x, out_pose->position.y, out_pose->position.z);
      }
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF2 error in call_place_service_sync: %s", ex.what());
      return false;
    }
  }

  bool call_stack_service_sync(const std::string &frame_name,
                               geometry_msgs::msg::Pose *out_pose)
  {
    if (!stack_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "get_stack_pos service not available");
      return false;
    }

    auto request = std::make_shared<robot_msgs::srv::GetPlacePos::Request>();
    request->frame_name = frame_name;

    auto result_future = stack_client_->async_send_request(request);
    if (result_future.wait_for(15s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "get_stack_pos service call timed out");
      return false;
    }

    const auto response = result_future.get();
    if (!response || !response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "get_stack_pos service returned failure");
      return false;
    }

    const std::string frame_id = response->place_pose.header.frame_id;
    if (frame_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "get_stack_pos returned empty frame_id");
      return false;
    }

    try
    {
      geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
          "world", frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

      if (out_pose != nullptr)
      {
        geometry_msgs::msg::PoseStamped pose_world;
        tf2::doTransform(response->place_pose, pose_world, t_stamped);
        *out_pose = pose_world.pose;
        RCLCPP_INFO(this->get_logger(),
                    "get_stack_pos frame=%s -> world=(%.3f, %.3f, %.3f)",
                    frame_id.c_str(),
                    out_pose->position.x, out_pose->position.y, out_pose->position.z);
      }
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF2 error in call_stack_service_sync: %s", ex.what());
      return false;
    }
  }

  double get_frame_yaw(const geometry_msgs::msg::Pose &frame_world)
  {
    const auto &q = frame_world.orientation;
    const double frame_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const double joint0_ik = std::atan2(
        frame_world.position.y, frame_world.position.x);

    double tool_roll = normalize_angle(
        place_frame_roll_sign_ * (frame_yaw - joint0_ik));

    // 矩形 180° 对称：与 get_box_edge_roll 保持一致，归一化到 [-π/2, π/2]
    if (tool_roll >  M_PI / 2.0) tool_roll -= M_PI;
    if (tool_roll < -M_PI / 2.0) tool_roll += M_PI;

    RCLCPP_INFO(this->get_logger(),
                "[get_frame_yaw] frame_yaw=%.3f joint0_ik=%.3f tool_roll=%.3f",
                frame_yaw, joint0_ik, tool_roll);
    return tool_roll;
  }

  bool do_place_move_with_orientation(const geometry_msgs::msg::Pose &frame_world)
  {
    const double tool_roll = get_frame_yaw(frame_world);
    const double pitch = grasp_pitch_ + tool_pitch_offset_;

    const Eigen::Vector3d ee_target(
        frame_world.position.x,
        frame_world.position.y,
        frame_world.position.z + place_frame_contact_offset_);

    const Eigen::Vector3d pre_target(
        ee_target.x(), ee_target.y(), ee_target.z() + place_frame_hover_height_);

    RCLCPP_INFO(this->get_logger(),
                "[place_frame] ee_z=%.3f pre_z=%.3f pitch=%.2f roll=%.2f",
                ee_target.z(), pre_target.z(), pitch, tool_roll);

    Eigen::VectorXd q_pre(5), q_place(5);
    if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[place_frame] IK failed for pre-place (%.3f, %.3f, %.3f)",
                   pre_target.x(), pre_target.y(), pre_target.z());
      return false;
    }
    if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[place_frame] IK failed for place (%.3f, %.3f, %.3f)",
                   ee_target.x(), ee_target.y(), ee_target.z());
      return false;
    }
    q_pre[0]   += tool_yaw_offset_;
    q_place[0] += tool_yaw_offset_;

    if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place}))
    {
      return false;
    }
    if (!wait_for_action_completion())
    {
      return false;
    }

    // 垂直后退
    Eigen::VectorXd q_snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      q_snap = q_current_;
    }

    // 到达放置点后先松开吸盘，再后退
    RCLCPP_INFO(this->get_logger(), "[place_frame] suction OFF before retreat");
    rclcpp::sleep_for(200ms);
    do_suction_off();
    rclcpp::sleep_for(300ms);

    if (q_snap.size() == 5)
    {
      auto fk = kin_engine_->forwardKinematics(q_snap);
      Eigen::Vector3d retreat_pos = fk.translation();
      retreat_pos.z() += place_retreat_offset_;

      Eigen::VectorXd q_retreat(5);
      if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat))
      {
        q_retreat[0] += tool_yaw_offset_;
        RCLCPP_INFO(this->get_logger(),
                    "[place_frame] retreating %.2fm upward", place_retreat_offset_);
        if (send_move_goal(std::vector<Eigen::VectorXd>{q_retreat}))
        {
          wait_for_action_completion();
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[place_frame] retreat IK failed, skipping.");
      }
    }
    return true;
  }

  bool do_stack_move_with_orientation(const geometry_msgs::msg::Pose &box_top_world)
  {
    const double tool_roll =
        normalize_angle(stack_roll_sign_ * (
            std::atan2(
                2.0 * (box_top_world.orientation.w * box_top_world.orientation.z +
                       box_top_world.orientation.x * box_top_world.orientation.y),
                1.0 - 2.0 * (box_top_world.orientation.y * box_top_world.orientation.y +
                             box_top_world.orientation.z * box_top_world.orientation.z))
            - std::atan2(box_top_world.position.y, box_top_world.position.x)));

    const double pitch = grasp_pitch_ + tool_pitch_offset_;

    const Eigen::Vector3d ee_target(
        box_top_world.position.x,
        box_top_world.position.y,
        box_top_world.position.z + stack_contact_offset_);

    const Eigen::Vector3d pre_target(
        ee_target.x(), ee_target.y(), ee_target.z() + stack_hover_height_);

    RCLCPP_INFO(this->get_logger(),
                "[stack] ee_z=%.3f pre_z=%.3f pitch=%.2f roll=%.2f",
                ee_target.z(), pre_target.z(), pitch, tool_roll);

    Eigen::VectorXd q_pre(5), q_stack(5);
    if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[stack] IK failed for pre-stack (%.3f, %.3f, %.3f)",
                   pre_target.x(), pre_target.y(), pre_target.z());
      return false;
    }
    if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_stack))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[stack] IK failed for stack (%.3f, %.3f, %.3f)",
                   ee_target.x(), ee_target.y(), ee_target.z());
      return false;
    }
    q_pre[0]   += tool_yaw_offset_;
    q_stack[0] += tool_yaw_offset_;

    if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_stack}))
    {
      return false;
    }
    if (!wait_for_action_completion())
    {
      return false;
    }

    // 垂直后退
    Eigen::VectorXd q_snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      q_snap = q_current_;
    }

    // 到达叠放点后先松开吸盘，再后退
    RCLCPP_INFO(this->get_logger(), "[stack] suction OFF before retreat");
    rclcpp::sleep_for(200ms);
    do_suction_off();
    rclcpp::sleep_for(300ms);

    if (q_snap.size() == 5)
    {
      auto fk = kin_engine_->forwardKinematics(q_snap);
      Eigen::Vector3d retreat_pos = fk.translation();
      retreat_pos.z() += place_retreat_offset_;

      Eigen::VectorXd q_retreat(5);
      if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat))
      {
        q_retreat[0] += tool_yaw_offset_;
        RCLCPP_INFO(this->get_logger(),
                    "[stack] retreating %.2fm upward", place_retreat_offset_);
        if (send_move_goal(std::vector<Eigen::VectorXd>{q_retreat}))
        {
          wait_for_action_completion();
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[stack] retreat IK failed, skipping.");
      }
    }
    return true;
  }

  /**
   * @brief 同步切换控制器模式，阻塞等待服务响应。
   */
  int request_mode_switch(const std::string &mode_name)
  {
    if (!mode_client_->wait_for_service(1s))
    {
      RCLCPP_ERROR(this->get_logger(), "Mode switch service not available");
      return 0;
    }

    auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
    request->mode = mode_name;

    auto result_future = mode_client_->async_send_request(request);
    if (result_future.wait_for(2s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Mode switch to [%s] timed out.", mode_name.c_str());
      return 0;
    }

    const auto response = result_future.get();
    if (!response || !response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "Mode switch to [%s] failed.", mode_name.c_str());
      return 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "\033[1;36m[Mode]\033[0m Controller switched to: %s", mode_name.c_str());
    return 1;
  }

  /**
   * @brief 同步控制吸盘，阻塞等待服务响应。
   */
  int set_suction(bool activate)
  {
    if (!suction_client_->wait_for_service(1s))
    {
      if (require_suction_service_)
      {
        RCLCPP_ERROR(this->get_logger(), "Required suction service is not available.");
        return 0;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Optional suction service unavailable; skipping suction command.");
      return 1; // 非必需时跳过，不视为失败
    }

    auto request = std::make_shared<robot_msgs::srv::SetSuction::Request>();
    request->activate = activate;

    auto result_future = suction_client_->async_send_request(request);
    if (result_future.wait_for(2s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Suction command [%s] timed out.",
                   activate ? "ON" : "OFF");
      return 0;
    }

    const auto response = result_future.get();
    if (!response || !response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "Suction command [%s] failed.",
                   activate ? "ON" : "OFF");
      return 0;
    }

    RCLCPP_INFO(this->get_logger(), "Suction %s", activate ? "ON" : "OFF");
    return 1;
  }

  int request_payload_estimate()
  {
    if (!payload_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Service [GetPayloadEstimate] not available.");
      return 0;
    }

    auto request = std::make_shared<robot_msgs::srv::GetPayloadEstimate::Request>();
    auto result_future = payload_client_->async_send_request(request);
    if (result_future.wait_for(2s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Payload estimate service call timed out.");
      return 0;
    }

    const auto response = result_future.get();
    if (!response || !response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "Payload estimate service responded with failure.");
      return 0;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    last_estimated_mass_ = response->mass;
    is_mass_updated_ = true;
    return 1;
  }

  bool wait_for_user_command(int *input_cmd)
  {
    if (input_cmd == nullptr)
    {
      return false;
    }

    while (rclcpp::ok() && is_running_.load())
    {
      pollfd stdin_poll{};
      stdin_poll.fd = STDIN_FILENO;
      stdin_poll.events = POLLIN;

      const int poll_rc = ::poll(&stdin_poll, 1, 200);
      if (poll_rc == 0)
      {
        continue;
      }
      if (poll_rc < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        RCLCPP_ERROR(this->get_logger(), "poll(stdin) failed.");
        return false;
      }
      if ((stdin_poll.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        RCLCPP_INFO(this->get_logger(), "stdin became unavailable; stopping task loop.");
        return false;
      }

      if (!(std::cin >> *input_cmd))
      {
        if (std::cin.eof())
        {
          RCLCPP_INFO(this->get_logger(), "stdin closed; stopping task loop.");
          return false;
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "请输入有效数字: " << std::flush;
        continue;
      }
      return true;
    }
    return false;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 2 — Task Building Blocks（从 aarm 迁移，使用 arm 接口）
  // ═══════════════════════════════════════════════════════════════════════════

  /** 等待关节速度降到阈值以下（运动稳定）。*/
  void wait_joints_still(double dq_threshold = 0.02, int timeout_ms = 1000)
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    auto stable_since = std::chrono::steady_clock::time_point{};
    bool stable_seen = false;

    while (rclcpp::ok() && is_running_.load())
    {
      Eigen::VectorXd dq_snapshot;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        dq_snapshot = dq_current_;
      }

      const bool has_dq = (dq_snapshot.size() == 5);
      const bool all_still =
          has_dq && (dq_snapshot.array().abs() < dq_threshold).all();
      const auto now = std::chrono::steady_clock::now();

      if (all_still)
      {
        if (!stable_seen)
        {
          stable_since = now;
          stable_seen = true;
        }
        else if (now - stable_since >= 150ms)
        {
          return;
        }
      }
      else
      {
        stable_seen = false;
      }

      if (now >= deadline)
      {
        double max_abs_dq = -1.0;
        if (has_dq)
        {
          max_abs_dq = dq_snapshot.cwiseAbs().maxCoeff();
        }
        RCLCPP_WARN(this->get_logger(),
                    "[wait_joints_still] timeout after %d ms, continue with max|dq|=%.4f",
                    timeout_ms, max_abs_dq);
        return;
      }

      rclcpp::sleep_for(20ms);
    }
  }

  /** 复位：关吸盘 → moving → reset → idle */
  void do_reset()
  {
    RCLCPP_INFO(this->get_logger(), "[reset] suction OFF -> moving -> reset -> idle");
    set_suction(false);
    if (!request_mode_switch("moving"))
    {
      return;
    }
    if (!presets_.count("reset"))
    {
      RCLCPP_ERROR(this->get_logger(), "Preset 'reset' not found!");
      return;
    }
    if (send_move_goal({presets_["reset"]}))
    {
      wait_for_action_completion();
    }
    request_mode_switch("idle");
  }

  /** 带吸盘复位（不关吸盘）：moving → reset → idle */
  void do_reset_suction()
  {
    RCLCPP_INFO(this->get_logger(), "[reset_suction] moving -> reset -> idle");
    if (!request_mode_switch("moving"))
    {
      return;
    }
    if (!presets_.count("reset"))
    {
      RCLCPP_ERROR(this->get_logger(), "Preset 'reset' not found!");
      return;
    }
    if (send_move_goal({presets_["reset"]}))
    {
      wait_for_action_completion();
    }
    request_mode_switch("idle");
  }

  /** 移到 load 预设（俯瞰姿态）。*/
  void do_load()
  {
    if (!presets_.count("load"))
    {
      RCLCPP_ERROR(this->get_logger(), "Preset 'load' not found!");
      return;
    }
    if (send_move_goal({presets_["load"]}))
    {
      wait_for_action_completion();
    }
  }

  /**
   * @brief 瞭望动作：joint_0 朝向目标 XY，其余关节用 look_out 预设。
   *        joint_4 固定为 0（相机视野正前向）。
   */
  void do_look_out(const geometry_msgs::msg::Pose &target)
  {
    if (!presets_.count("look_out"))
    {
      RCLCPP_ERROR(this->get_logger(), "Preset 'look_out' not found!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[look_out] yaw toward (%.3f, %.3f)",
                target.position.x, target.position.y);

    auto goal_q = presets_["look_out"];
    goal_q[0] = std::atan2(target.position.y, target.position.x);
    goal_q[4] = 0.0;
    if (send_move_goal({goal_q}))
    {
      wait_for_action_completion();
    }
  }

  /** 吸盘 ON：等臂稳定 → suction ON → 切 loaded 模式。*/
  void do_suction_on()
  {
    RCLCPP_INFO(this->get_logger(), "[suction] waiting 400ms for arm to settle...");
    rclcpp::sleep_for(400ms);
    set_suction(true);
    rclcpp::sleep_for(500ms);
    request_mode_switch("loaded");
  }

  /** 吸盘 OFF：suction OFF → 切 moving 模式。*/
  void do_suction_off()
  {
    RCLCPP_INFO(this->get_logger(), "[suction] OFF -> mode=moving");
    set_suction(false);
    request_mode_switch("moving");
  }

  /**
   * @brief 抓取运动（显式 roll）：pre-grasp → grasp 双段轨迹。
   */
  bool do_grasp_move(const geometry_msgs::msg::Pose &target, double tool_roll)
  {
    const double pitch = grasp_pitch_ + tool_pitch_offset_;

    // 吸盘接触面目标点（XY 偏移沿臂方向旋转，不受 joint_0 朝向影响）
    const double q0 = std::atan2(target.position.y, target.position.x);
    const double cos_q0 = std::cos(q0);
    const double sin_q0 = std::sin(q0);
    const Eigen::Vector3d suction_target(
        target.position.x + tool_offset_x_ * cos_q0 - tool_offset_y_ * sin_q0,
        target.position.y + tool_offset_x_ * sin_q0 + tool_offset_y_ * cos_q0,
        target.position.z + object_height_ + tool_offset_z_);

    // 将吸盘目标点沿末端方向（由 pitch 和 q0 决定）往回退 tool_tip_length_，
    // 得到 Link_5 origin 的目标位置，这才是 IK 的实际求解目标。
    const Eigen::Vector3d tip_dir(
        std::cos(q0) * std::cos(pitch),
        std::sin(q0) * std::cos(pitch),
        std::sin(pitch));
    const Eigen::Vector3d ee_target = suction_target - tool_tip_length_ * tip_dir;

    const Eigen::Vector3d pre_target(
        ee_target.x(), ee_target.y(), ee_target.z() + pre_grasp_offset_);

    Eigen::VectorXd q_pre(5), q_grasp(5);
    if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[grasp_move] IK failed for pre-grasp (%.3f, %.3f, %.3f)",
                   pre_target.x(), pre_target.y(), pre_target.z());
      return false;
    }
    if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_grasp))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[grasp_move] IK failed for grasp (%.3f, %.3f, %.3f)",
                   ee_target.x(), ee_target.y(), ee_target.z());
      return false;
    }
    q_pre[0] += tool_yaw_offset_;
    q_grasp[0] += tool_yaw_offset_;

    RCLCPP_INFO(this->get_logger(),
                "[grasp_move] pre-z=%.3f grasp-z=%.3f pitch=%.2f roll=%.2f",
                pre_target.z(), ee_target.z(), pitch, tool_roll);

    if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_grasp}))
    {
      return false;
    }
    return wait_for_action_completion();
  }

  bool do_grasp_move(const geometry_msgs::msg::Pose &target)
  {
    return do_grasp_move(target, get_object_yaw(target));
  }

  /**
   * @brief 放置运动：pre-place（+pre_place_offset）→ place 双段 + 垂直后退。
   *        返回 false 表示 IK 失败或运动失败。
   */
  bool do_place_move(const geometry_msgs::msg::Pose &target)
  {
    const double tool_roll = get_object_yaw(target);
    const double pitch = grasp_pitch_ + tool_pitch_offset_;

    // 吸盘接触面目标点（XY 偏移沿臂方向旋转，不受 joint_0 朝向影响）
    const double q0 = std::atan2(target.position.y, target.position.x);
    const double cos_q0 = std::cos(q0);
    const double sin_q0 = std::sin(q0);
    const Eigen::Vector3d suction_target(
        target.position.x + tool_offset_x_ * cos_q0 - tool_offset_y_ * sin_q0,
        target.position.y + tool_offset_x_ * sin_q0 + tool_offset_y_ * cos_q0,
        target.position.z + object_height_ + tool_offset_z_);

    // 同 do_grasp_move：退回 tool_tip_length_ 得到 Link_5 origin 目标位置
    const Eigen::Vector3d tip_dir(
        std::cos(q0) * std::cos(pitch),
        std::sin(q0) * std::cos(pitch),
        std::sin(pitch));
    const Eigen::Vector3d ee_target = suction_target - tool_tip_length_ * tip_dir;

    const Eigen::Vector3d pre_target(
        ee_target.x(), ee_target.y(), ee_target.z() + pre_place_offset_);

    Eigen::VectorXd q_pre(5), q_place(5);
    if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[place_move] IK failed for pre-place (%.3f, %.3f, %.3f)",
                   pre_target.x(), pre_target.y(), pre_target.z());
      return false;
    }
    if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[place_move] IK failed for place (%.3f, %.3f, %.3f)",
                   ee_target.x(), ee_target.y(), ee_target.z());
      return false;
    }
    q_pre[0] += tool_yaw_offset_;
    q_place[0] += tool_yaw_offset_;

    RCLCPP_INFO(this->get_logger(),
                "[place_move] pre-z=%.3f place-z=%.3f pitch=%.2f roll=%.2f",
                pre_target.z(), ee_target.z(), pitch, tool_roll);

    if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place}))
    {
      return false;
    }
    if (!wait_for_action_completion())
    {
      return false;
    }

    // 垂直后退
    Eigen::VectorXd q_current_snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      q_current_snap = q_current_;
    }
    if (q_current_snap.size() == 5)
    {
      auto fk = kin_engine_->forwardKinematics(q_current_snap);
      Eigen::Vector3d retreat_pos = fk.translation();
      retreat_pos.z() += place_retreat_offset_;

      Eigen::VectorXd q_retreat(5);
      if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat))
      {
        q_retreat[0] += tool_yaw_offset_;
        RCLCPP_INFO(this->get_logger(),
                    "[place_move] retreating %.2fm upward", place_retreat_offset_);
        if (send_move_goal({q_retreat}))
        {
          wait_for_action_completion();
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[place_move] retreat IK failed, skipping.");
      }
    }
    return true;
  }

  /**
   * @brief 完整抓取序列：look_out → grasp → suction ON。
   */
  bool do_full_grasp(const geometry_msgs::msg::Pose &target)
  {
    target_pub_->publish(target);
    request_mode_switch("moving");
    do_look_out(target);
    wait_joints_still(0.02, 800);

    if (!do_grasp_move(target))
    {
      return false;
    }
    do_suction_on();
    return true;
  }

  /**
   * @brief 完整抓取序列（带朝向对齐）：look_out → grasp → suction ON。
   *        用 get_box_edge_roll 从感知 orientation 中提取 joint_4 roll。
   *        orientation 为 identity 时退化为 roll=0，与 do_full_grasp 等价。
   */
  bool do_full_grasp_aligned(const geometry_msgs::msg::Pose &target)
  {
    target_pub_->publish(target);
    request_mode_switch("moving");
    do_look_out(target);
    wait_joints_still(0.02, 800);

    const double roll = get_box_edge_roll(target);
    if (!do_grasp_move(target, roll))
    {
      return false;
    }
    do_suction_on();
    return true;
  }

  /**
   * @brief 完整放置序列：look_out → suction OFF → pre-place → place → retreat。
   */
  bool do_full_place(const geometry_msgs::msg::Pose &target)
  {
    target_pub_->publish(target);
    request_mode_switch("moving");
    do_look_out(target);

    RCLCPP_INFO(this->get_logger(), "[place] moving to pre-place then place...");
    if (!do_place_move(target))
    {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "[place] suction OFF");
    rclcpp::sleep_for(200ms);
    do_suction_off();
    rclcpp::sleep_for(300ms);

    RCLCPP_INFO(this->get_logger(), "[place] done.");
    return true;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 3 — 3-Phase Pipeline（流程状态机，使用 arm 接口和 arm 感知函数）
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief Phase 1：移到 look_out，获取粗定位目标。
   *        'r' = 调用 get_pick_pos 服务；
   *        'm' = 用户手动输入 world 坐标 x y z；
   *        'a' = 中止。
   */
  bool phase1_get_coarse_target(geometry_msgs::msg::Pose &target_world)
  {
    RCLCPP_INFO(get_logger(), "[Phase1] Scan: moving to look_out pose...");
    request_mode_switch("moving");

    // 先朝正前方瞭望
    geometry_msgs::msg::Pose fwd;
    fwd.position.x = 1.0;
    fwd.position.y = 0.0;
    fwd.position.z = 0.0;
    fwd.orientation.w = 1.0;
    do_look_out(fwd);
    wait_joints_still(0.02, 800);

    RCLCPP_INFO(get_logger(), "[Phase1] At look_out. Select input mode:");
    std::cout << "\n[Phase1/Scan] Sensor input: (r)eal sensor / (m)anual input / (a)bort: ";
    char c = 'a';
    std::cin >> c;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (c == 'a')
    {
      RCLCPP_WARN(get_logger(), "[Phase1] Aborted.");
      return false;
    }

    if (c == 'r')
    {
      // 使用 arm 已验证的感知接口
      return call_pick_service_sync(step6_pick_object_name_, &target_world);
    }

    // 手动输入 world 坐标
    double mx, my, mz;
    std::cout << "  Enter target position in world frame  x y z (m): ";
    if (!(std::cin >> mx >> my >> mz))
    {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      RCLCPP_WARN(get_logger(), "[Phase1] Invalid input.");
      return false;
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    target_world.position.x = mx;
    target_world.position.y = my;
    target_world.position.z = mz;
    target_world.orientation.w = 1.0;
    RCLCPP_INFO(get_logger(), "[Phase1] Manual target: world=(%.3f, %.3f, %.3f)", mx, my, mz);
    return true;
  }

  /**
   * @brief Phase 2：固定俯视高度下做 XY 闭环对齐。
   *        每轮调用 call_pick_service_sync 获取物体 world 坐标，
   *        计算与当前末端 XY 的误差，误差 < align_threshold 则收敛。
   *        整个过程 Z 保持不变（固定为进入 Phase 2 时的末端 Z）。
   */
  bool phase2_align(geometry_msgs::msg::Pose &target_world)
  {
    RCLCPP_INFO(get_logger(), "[Phase2] Look-down: moving to overhead (load) pose...");
    request_mode_switch("moving");

    // 移到俯瞰姿态，joint_0 朝向粗定位目标
    if (!presets_.count("load"))
    {
      RCLCPP_ERROR(get_logger(), "Preset 'load' not found!");
      return false;
    }
    auto q_overhead = presets_["load"];
    q_overhead[0] = std::atan2(target_world.position.y, target_world.position.x);
    q_overhead[4] = 0.0;
    if (!send_move_goal({q_overhead}))
    {
      return false;
    }
    if (!wait_for_action_completion())
    {
      return false;
    }
    wait_joints_still(0.02, 800);

    // 记录进入 Phase 2 时的末端 Z（全程固定，只调 XY）
    double fixed_z;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (q_current_.size() != 5)
      {
        RCLCPP_ERROR(get_logger(), "[Phase2] No valid joint state.");
        return false;
      }
      auto fk0 = kin_engine_->forwardKinematics(q_current_);
      fixed_z = fk0.translation().z();
    }
    RCLCPP_INFO(get_logger(), "[Phase2] Overhead Z fixed at %.3f m. Starting alignment loop...", fixed_z);

    for (int iter = 0; iter < align_max_iters_; ++iter)
    {
      // 1. 感知：获取物体当前 world 坐标（arm 已验证接口）
      geometry_msgs::msg::Pose detected;
      if (!call_pick_service_sync(step6_pick_object_name_, &detected))
      {
        RCLCPP_WARN(get_logger(), "[Phase2] iter %d: perception failed, skipping.", iter + 1);
        continue;
      }

      // 2. FK：获取当前末端 XY
      double ex, ey;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        auto fk = kin_engine_->forwardKinematics(q_current_);
        ex = fk.translation().x();
        ey = fk.translation().y();
      }

      // 3. 计算误差
      const double dx = detected.position.x - ex;
      const double dy = detected.position.y - ey;
      const double error = std::hypot(dx, dy);

      RCLCPP_INFO(get_logger(),
                  "[Phase2] iter %d/%d | object=(%.3f,%.3f) EE=(%.3f,%.3f) err=%.4f m",
                  iter + 1, align_max_iters_,
                  detected.position.x, detected.position.y, ex, ey, error);

      // 4. 收敛判断
      if (error < align_threshold_)
      {
        RCLCPP_INFO(get_logger(),
                    "[Phase2] Converged (%.4f m < %.4f m threshold).", error, align_threshold_);
        // 更新最终 XY（Z 不变）
        target_world.position.x = detected.position.x;
        target_world.position.y = detected.position.y;
        return true;
      }

      // 5. 微调：只转 joint_0 朝向目标 XY，保持其余关节不动
      //    不用完整 IK 重解，避免每次迭代引起大幅度姿态跳变（"乱甩"）。
      target_world.position.x = detected.position.x;
      target_world.position.y = detected.position.y;

      Eigen::VectorXd q_new = q_overhead;
      q_new[0] = std::atan2(detected.position.y, detected.position.x);
      q_new[4] = 0.0;
      if (!send_move_goal({q_new}))
      {
        return false;
      }
      if (!wait_for_action_completion())
      {
        return false;
      }
      wait_joints_still(0.02, 600);
    }

    RCLCPP_WARN(get_logger(),
                "[Phase2] Max iters (%d) reached without convergence. Proceeding with last estimate.",
                align_max_iters_);
    return true; // 未收敛但不中止，用最后估计值继续
  }

  /**
   * @brief Phase 3（抓取）：joint_4 → -90° → 询问确认 → pre-grasp + grasp → suction ON → roll 归零。
   */
  bool phase3_grasp_descend(const geometry_msgs::msg::Pose &target_world)
  {
    RCLCPP_INFO(get_logger(), "[Phase3] Grasp: rotating joint_4 to -90°...");

    Eigen::VectorXd q_rot;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      q_rot = q_current_;
    }
    if (q_rot.size() == 5)
    {
      q_rot[4] = -M_PI / 2.0;
      if (!send_move_goal({q_rot}))
      {
        return false;
      }
      if (!wait_for_action_completion())
      {
        return false;
      }
      wait_joints_still(0.02, 600);
    }

    RCLCPP_INFO(get_logger(), "[Phase3] Joint_4 at -90°. Confirm to descend:");
    std::cout << "\n[Phase3/PreGrasp-confirm] Continue: (r)eal / (m)anual-confirm / (a)bort: ";
    char c = 'r';
    std::cin >> c;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (c == 'a')
    {
      RCLCPP_WARN(get_logger(), "[Phase3] Aborted before grasp.");
      return false;
    }

    // pre-grasp + grasp 双段 IK（roll 由 do_grasp_move 内部通过 get_object_yaw 设定）
    if (!do_grasp_move(target_world))
    {
      return false;
    }
    do_suction_on();

    return true;
  }

  /**
   * @brief Phase 3（放置）：joint_4 → -90° → 询问确认 → pre-place + place → suction OFF → 后退。
   */
  bool phase3_place_descend(const geometry_msgs::msg::Pose &target_world)
  {
    RCLCPP_INFO(get_logger(), "[Phase3P] Place: rotating joint_4 to -90°...");

    Eigen::VectorXd q_rot;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      q_rot = q_current_;
    }
    if (q_rot.size() == 5)
    {
      q_rot[4] = -M_PI / 2.0;
      if (!send_move_goal({q_rot}))
      {
        return false;
      }
      if (!wait_for_action_completion())
      {
        return false;
      }
      wait_joints_still(0.02, 600);
    }

    std::cout << "\n[Phase3P/PrePlace-confirm] Continue: (r)eal / (m)anual-confirm / (a)bort: ";
    char c = 'r';
    std::cin >> c;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (c == 'a')
    {
      RCLCPP_WARN(get_logger(), "[Phase3P] Aborted before place.");
      return false;
    }

    if (!do_place_move(target_world))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "[Phase3P] suction OFF");
    rclcpp::sleep_for(200ms);
    do_suction_off();
    rclcpp::sleep_for(300ms);

    RCLCPP_INFO(get_logger(), "[Phase3P] Place done.");
    return true;
  }

  /** 完整 3-Phase 抓取流程。*/
  bool do_3phase_grasp()
  {
    RCLCPP_INFO(get_logger(), "=== 3-Phase Grasp Pipeline ===");

    geometry_msgs::msg::Pose coarse_target;
    if (!phase1_get_coarse_target(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PGrasp] Phase1 failed.");
      return false;
    }

    if (!phase2_align(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PGrasp] Phase2 alignment failed.");
      return false;
    }

    target_pub_->publish(coarse_target);
    if (!phase3_grasp_descend(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PGrasp] Phase3 grasp failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "=== 3-Phase Grasp Complete ===");
    return true;
  }

  /** 完整 3-Phase 放置流程。*/
  bool do_3phase_place()
  {
    RCLCPP_INFO(get_logger(), "=== 3-Phase Place Pipeline ===");

    geometry_msgs::msg::Pose coarse_target;
    if (!phase1_get_coarse_target(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PPlace] Phase1 failed.");
      return false;
    }

    if (!phase2_align(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PPlace] Phase2 alignment failed.");
      return false;
    }

    target_pub_->publish(coarse_target);
    if (!phase3_place_descend(coarse_target))
    {
      RCLCPP_ERROR(get_logger(), "[3PPlace] Phase3 place failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "=== 3-Phase Place Complete ===");
    return true;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 4 — Main Task Loop
  // ═══════════════════════════════════════════════════════════════════════════

  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 5 — Remote Control (外部节点通过 /arm/cmd 控制)
  // ═══════════════════════════════════════════════════════════════════════════

  void publish_status(const std::string &s)
  {
    if (!status_pub_) return;
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[remote] status → \"%s\"", s.c_str());
  }

  /**
   * @brief 完整抓取序列（远程模式）：
   *        look_out → perception → grasp → suction ON
   *        → sleep 0.5s → "grasped"
   *        → carry preset → loaded → "stowed"
   */
  bool do_grasp_sequence()
  {
    geometry_msgs::msg::Pose target;

    // look_out 朝正前，再调感知
    geometry_msgs::msg::Pose fwd;
    fwd.position.x = 1.0;
    fwd.orientation.w = 1.0;
    request_mode_switch("moving");
    do_look_out(fwd);
    wait_joints_still(0.02, 800);

    if (!call_pick_service_sync(step6_pick_object_name_, &target))
    {
      publish_status("error:perception_failed");
      return false;
    }

    if (!do_full_grasp_aligned(target))
    {
      publish_status("error:grasp_failed");
      return false;
    }

    rclcpp::sleep_for(500ms);
    publish_status("grasped");

    // carry：moving → carry preset → loaded
    if (!request_mode_switch("moving"))
    {
      publish_status("error:mode_switch_failed");
      return false;
    }
    if (!presets_.count("carry"))
    {
      RCLCPP_ERROR(this->get_logger(), "[remote] Preset 'carry' not found!");
      publish_status("error:no_carry_preset");
      return false;
    }
    if (!send_move_goal({presets_["carry"]}) || !wait_for_action_completion())
    {
      publish_status("error:carry_failed");
      return false;
    }
    request_mode_switch("loaded");

    publish_status("stowed");
    return true;
  }

  /**
   * @brief 完整放置序列（远程模式）：
   *        perception → do_place_move_with_orientation（含 suction OFF + retreat）
   *        → "placed" → do_reset → "reset"
   */
  bool do_place_sequence()
  {
    geometry_msgs::msg::Pose frame_pose;

    if (!call_place_service_sync(place_frame_name_, &frame_pose))
    {
      publish_status("error:place_perception_failed");
      return false;
    }

    request_mode_switch("moving");
    if (!do_place_move_with_orientation(frame_pose))
    {
      publish_status("error:place_failed");
      return false;
    }

    publish_status("placed");

    do_reset();
    publish_status("reset");
    return true;
  }

  /**
   * @brief 远程控制主循环。
   *        启动后先 reset，广播 "reset"，然后等待 /arm/cmd 命令。
   *        支持 "grasp" / "place"；执行中拒绝新命令并广播 "error:busy"。
   */
  void run_remote_control()
  {
    if (!wait_for_system_ready())
    {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[remote] Ready. Performing initial reset...");
    do_reset();
    publish_status("reset");
    RCLCPP_INFO(this->get_logger(), "[remote] Waiting for commands on /arm/cmd ...");

    while (rclcpp::ok() && is_running_.load())
    {
      std::string cmd;
      {
        std::unique_lock<std::mutex> lk(cmd_mutex_);
        cmd_cv_.wait_for(lk, std::chrono::milliseconds(200),
                         [this] { return !pending_cmd_.empty(); });
        if (pending_cmd_.empty()) continue;
        cmd = pending_cmd_;
        pending_cmd_.clear();
      }

      if (remote_busy_.exchange(true))
      {
        publish_status("error:busy");
        continue;
      }

      if (cmd == "grasp")
      {
        RCLCPP_INFO(this->get_logger(), "[remote] Executing grasp sequence...");
        do_grasp_sequence();
      }
      else if (cmd == "place")
      {
        RCLCPP_INFO(this->get_logger(), "[remote] Executing place sequence...");
        do_place_sequence();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[remote] Unknown command: %s", cmd.c_str());
        publish_status("error:unknown_cmd:" + cmd);
      }

      remote_busy_.store(false);
    }
  }

  void run_task_sequence()
  {
    if (!wait_for_system_ready())
    {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Task Node Ready.");

    while (rclcpp::ok() && is_running_.load())
    {
      std::cout
          << "\n====== Task Control Panel ======\n"
          << "1:  Reset (suction OFF -> moving -> reset -> idle)\n"
          << "2:  Joint preset A (debug)\n"
          << "3:  Joint preset B (debug)\n"
          << "4:  Auto place   (dog camera perception -> pre-place -> place -> suction OFF -> retreat)\n"
          << "5:  Manual place (input frame x y z yaw -> pre-place -> place -> suction OFF -> retreat)\n"
          << "6:  Auto grasp   (perception/mock -> look_out -> grasp -> suction)\n"
          << "7:  Manual grasp (input world x y z -> look_out -> grasp -> suction)\n"
          << "8:  Release (suction OFF -> moving)\n"
          << "9:  Carry reset (suction ON, moving -> carry preset -> loaded)\n"
          << "10: Move to load preset\n"
          << "11: Estimate payload\n"
          << "12: 3-Phase Grasp (scan -> overhead align -> joint4 -90 -> grasp)\n"
          << "13: 3-Phase Place (scan -> overhead align -> joint4 -90 -> place)\n"
          << "14: Auto Stack   (dog camera -> pre-stack -> stack -> suction OFF -> retreat)\n"
          << "15: Manual Stack (input box top x y z yaw -> stack -> suction OFF -> retreat)\n"
          << "0:  Exit\n"
          << "cmd: " << std::flush;

      int input_cmd;
      if (!wait_for_user_command(&input_cmd))
      {
        return;
      }

      Eigen::VectorXd q_(5);

      switch (input_cmd)
      {
      case 1:
        RCLCPP_INFO(this->get_logger(), ">>> Reset");
        do_reset();
        break;

      case 2:
        RCLCPP_INFO(this->get_logger(), ">>> Preset A");
        request_mode_switch("moving");
        q_ << 0, 160, -130, 40, 0;
        q_ *= M_PI / 180.0;
        if (send_move_goal(q_))
        {
          wait_for_action_completion();
        }
        break;

      case 3:
        RCLCPP_INFO(this->get_logger(), ">>> Preset B");
        request_mode_switch("moving");
        q_ << 180, 90, -90, -90, 0;
        q_ *= M_PI / 180.0;
        if (send_move_goal(q_))
        {
          wait_for_action_completion();
        }
        break;

      case 4:
      {
        // Auto Place：调用狗头相机感知服务获取方框位姿，直接放置（无 look_out）
        RCLCPP_INFO(this->get_logger(), ">>> Auto Place (dog camera perception)");
        geometry_msgs::msg::Pose frame_pose;

        if (place_frame_use_mock_)
        {
          frame_pose.position.x = place_frame_mock_x_;
          frame_pose.position.y = place_frame_mock_y_;
          frame_pose.position.z = place_frame_mock_z_;
          frame_pose.orientation.x = 0.0;
          frame_pose.orientation.y = 0.0;
          frame_pose.orientation.z = std::sin(place_frame_mock_yaw_ / 2.0);
          frame_pose.orientation.w = std::cos(place_frame_mock_yaw_ / 2.0);
          RCLCPP_INFO(this->get_logger(),
                      "[case4] mock frame pos=(%.3f,%.3f,%.3f) yaw=%.3f rad",
                      place_frame_mock_x_, place_frame_mock_y_,
                      place_frame_mock_z_, place_frame_mock_yaw_);
        }
        else
        {
          if (!call_place_service_sync(place_frame_name_, &frame_pose))
          {
            RCLCPP_WARN(this->get_logger(), "[case4] Perception failed, abort.");
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(),
                    "[case4] frame world=(%.3f,%.3f,%.3f)",
                    frame_pose.position.x, frame_pose.position.y, frame_pose.position.z);

        if (!do_place_move_with_orientation(frame_pose))
        {
          RCLCPP_ERROR(this->get_logger(), "[case4] Place move failed.");
          break;
        }
        request_mode_switch("moving");
        RCLCPP_INFO(this->get_logger(), "[case4] Auto Place done.");
        break;
      }

      case 5:
      {
        // Manual Place：用户手动输入方框中心 x y z 和朝向 yaw，用于调试和验证
        RCLCPP_INFO(this->get_logger(), ">>> Manual Place (input frame pose)");
        double tx, ty, tz, tyaw;
        std::cout << "Enter frame center x y z (world, m) and yaw (rad): ";
        if (!(std::cin >> tx >> ty >> tz >> tyaw))
        {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(this->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        geometry_msgs::msg::Pose frame_pose;
        frame_pose.position.x = tx;
        frame_pose.position.y = ty;
        frame_pose.position.z = tz;
        frame_pose.orientation.x = 0.0;
        frame_pose.orientation.y = 0.0;
        frame_pose.orientation.z = std::sin(tyaw / 2.0);
        frame_pose.orientation.w = std::cos(tyaw / 2.0);

        RCLCPP_INFO(this->get_logger(),
                    "[case5] frame pos=(%.3f,%.3f,%.3f) yaw=%.3f",
                    tx, ty, tz, tyaw);

        request_mode_switch("moving");
        if (!do_place_move_with_orientation(frame_pose))
        {
          RCLCPP_ERROR(this->get_logger(), "[case5] Place move failed.");
          break;
        }
        request_mode_switch("moving");
        RCLCPP_INFO(this->get_logger(), "[case5] Manual Place done.");
        break;
      }

      case 6:
      {
        RCLCPP_INFO(this->get_logger(), ">>> Auto Grasp");
        geometry_msgs::msg::Pose target;

        if (step6_use_mock_target_)
        {
          target.position.x = step6_mock_x_;
          target.position.y = step6_mock_y_;
          target.position.z = step6_mock_z_;
          target.orientation.w = 1.0;
          RCLCPP_INFO(this->get_logger(), "Using mock target (%.3f, %.3f, %.3f)",
                      step6_mock_x_, step6_mock_y_, step6_mock_z_);
        }
        else
        {
          // 先移到 look_out，再调用感知服务
          geometry_msgs::msg::Pose fwd;
          fwd.position.x = 1.0;
          fwd.position.y = 0.0;
          fwd.position.z = 0.0;
          fwd.orientation.w = 1.0;
          request_mode_switch("moving");
          do_look_out(fwd);
          wait_joints_still(0.02, 800);

          if (!call_pick_service_sync(step6_pick_object_name_, &target))
          {
            RCLCPP_WARN(this->get_logger(), "Perception failed, abort.");
            break;
          }
        }
        do_full_grasp_aligned(target);
        break;
      }

      case 7:
      {
        double tx, ty, tz;
        std::cout << "Enter target x y z (world frame, meters): ";
        if (!(std::cin >> tx >> ty >> tz))
        {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(this->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        geometry_msgs::msg::Pose target;
        target.position.x = tx;
        target.position.y = ty;
        target.position.z = tz;
        target.orientation.w = 1.0;
        RCLCPP_INFO(this->get_logger(), ">>> Manual Grasp target=(%.3f,%.3f,%.3f)", tx, ty, tz);
        do_full_grasp(target);
        break;
      }

      case 8:
        do_suction_off();
        break;

      case 9:
      {
        RCLCPP_INFO(this->get_logger(), "[carry] moving -> carry preset -> loaded");
        if (!request_mode_switch("moving"))
        {
          break;
        }
        if (!presets_.count("carry"))
        {
          RCLCPP_ERROR(this->get_logger(), "Preset 'carry' not found!");
          break;
        }
        if (send_move_goal({presets_["carry"]}))
        {
          wait_for_action_completion();
        }
        request_mode_switch("loaded");
        break;
      }

      case 10:
        RCLCPP_INFO(this->get_logger(), ">>> Move to load preset");
        request_mode_switch("moving");
        do_load();
        break;

      case 11:
        if (request_payload_estimate())
        {
          RCLCPP_INFO(this->get_logger(),
                      "Estimated payload mass: %.3f kg", last_estimated_mass_);
        }
        break;

      case 12:
        RCLCPP_INFO(this->get_logger(), ">>> 3-Phase Grasp");
        do_3phase_grasp();
        break;

      case 13:
        RCLCPP_INFO(this->get_logger(), ">>> 3-Phase Place");
        do_3phase_place();
        break;

      case 14:
      {
        // Auto Stack：调用 get_stack_pos 感知服务获取目标箱子上表面位姿，叠放
        RCLCPP_INFO(this->get_logger(), ">>> Auto Stack (dog camera perception)");
        geometry_msgs::msg::Pose box_top_pose;

        if (stack_use_mock_)
        {
          box_top_pose.position.x = stack_mock_x_;
          box_top_pose.position.y = stack_mock_y_;
          box_top_pose.position.z = stack_mock_z_;
          box_top_pose.orientation.x = 0.0;
          box_top_pose.orientation.y = 0.0;
          box_top_pose.orientation.z = std::sin(stack_mock_yaw_ / 2.0);
          box_top_pose.orientation.w = std::cos(stack_mock_yaw_ / 2.0);
          RCLCPP_INFO(this->get_logger(),
                      "[case14] mock box_top pos=(%.3f,%.3f,%.3f) yaw=%.3f rad",
                      stack_mock_x_, stack_mock_y_, stack_mock_z_, stack_mock_yaw_);
        }
        else
        {
          if (!call_stack_service_sync(stack_service_name_, &box_top_pose))
          {
            RCLCPP_WARN(this->get_logger(), "[case14] Stack perception failed, abort.");
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(),
                    "[case14] box_top world=(%.3f,%.3f,%.3f)",
                    box_top_pose.position.x,
                    box_top_pose.position.y,
                    box_top_pose.position.z);

        request_mode_switch("moving");
        if (!do_stack_move_with_orientation(box_top_pose))
        {
          RCLCPP_ERROR(this->get_logger(), "[case14] Stack move failed.");
          break;
        }
        request_mode_switch("moving");
        RCLCPP_INFO(this->get_logger(), "[case14] Auto Stack done.");
        break;
      }

      case 15:
      {
        // Manual Stack：手动输入目标箱子上表面 x y z 和 yaw，用于调试
        RCLCPP_INFO(this->get_logger(), ">>> Manual Stack (input box top pose)");
        double tx, ty, tz, tyaw;
        std::cout << "Enter box top surface center x y z (world, m) and yaw (rad): ";
        if (!(std::cin >> tx >> ty >> tz >> tyaw))
        {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(this->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        geometry_msgs::msg::Pose box_top_pose;
        box_top_pose.position.x = tx;
        box_top_pose.position.y = ty;
        box_top_pose.position.z = tz;
        box_top_pose.orientation.x = 0.0;
        box_top_pose.orientation.y = 0.0;
        box_top_pose.orientation.z = std::sin(tyaw / 2.0);
        box_top_pose.orientation.w = std::cos(tyaw / 2.0);

        RCLCPP_INFO(this->get_logger(),
                    "[case15] box_top pos=(%.3f,%.3f,%.3f) yaw=%.3f",
                    tx, ty, tz, tyaw);

        request_mode_switch("moving");
        if (!do_stack_move_with_orientation(box_top_pose))
        {
          RCLCPP_ERROR(this->get_logger(), "[case15] Stack move failed.");
          break;
        }
        request_mode_switch("moving");
        RCLCPP_INFO(this->get_logger(), "[case15] Manual Stack done.");
        break;
      }

      case 0:
        RCLCPP_INFO(this->get_logger(), "Exit.");
        is_running_.store(false);
        rclcpp::shutdown();
        return;

      default:
        RCLCPP_WARN(this->get_logger(), "Unknown command: %d", input_cmd);
        break;
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Member Variables
  // ═══════════════════════════════════════════════════════════════════════════

  // State
  arm2_task::TaskState state_;
  std::atomic<bool> has_robot_data_{false};
  Eigen::VectorXd q_current_;
  Eigen::VectorXd dq_current_;
  std::mutex mtx_;
  std::atomic<bool> driver_ready_{false};

  // Action state
  std::atomic<bool> is_action_running_{false};
  std::atomic<bool> action_finished_{false};
  bool last_action_succeeded_{false};
  std::string last_action_message_;
  std::mutex action_result_mutex_;

  // Trajectory parameters
  double max_v_{1.0};
  double max_a_{2.0};
  double dist_threshold_{0.05};

  // Task flags
  bool require_payload_service_{false};
  bool require_suction_service_{false};

  // Grasp / place parameters
  double grasp_pitch_{-1.57};
  double tool_pitch_offset_{0.0};
  double tool_yaw_offset_{0.0};
  double object_height_{0.05};
  double pre_grasp_offset_{0.10};
  double pre_place_offset_{0.12};
  double place_retreat_offset_{0.15};
  double tool_offset_x_{0.0};
  double tool_offset_y_{0.0};
  double tool_offset_z_{0.0};
  double tool_tip_length_{0.0};

  // Step 6 auto-grasp parameters
  bool step6_use_mock_target_{false};
  double step6_mock_x_{0.35};
  double step6_mock_y_{0.0};
  double step6_mock_z_{0.12};
  std::string step6_pick_object_name_{"box"};

  // Place-frame parameters (狗头相机放置，task_place_frame)
  double place_frame_hover_height_{0.25};
  double place_frame_contact_offset_{0.0};
  std::string place_frame_name_{"target_frame"};
  double place_frame_roll_sign_{1.0};
  bool   place_frame_use_mock_{false};
  double place_frame_mock_x_{0.35};
  double place_frame_mock_y_{0.0};
  double place_frame_mock_z_{0.0};
  double place_frame_mock_yaw_{0.0};

  // Stack parameters (箱子叠放，task_stack)
  double stack_hover_height_{0.05};
  double stack_contact_offset_{0.25};
  std::string stack_service_name_{"get_stack_pos"};
  double stack_roll_sign_{1.0};
  bool   stack_use_mock_{false};
  double stack_mock_x_{0.35};
  double stack_mock_y_{0.0};
  double stack_mock_z_{0.1};
  double stack_mock_yaw_{0.0};

  // Phase-2 alignment parameters
  double align_threshold_{0.005}; // 5 mm
  int align_max_iters_{5};

  // Payload estimation
  float last_estimated_mass_{0.0f};
  std::atomic<bool> is_mass_updated_{false};
  std::atomic<bool> is_pose_updated_{false};

  // Presets
  std::map<std::string, Eigen::VectorXd> presets_;

  // Kinematics
  std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;

  // TF (arm 已验证的 shared_ptr 风格)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers / Subscribers / Clients
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client_;
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client_;
  rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client_;

  // Remote control
  bool remote_mode_{false};
  std::atomic<bool> remote_busy_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;
  std::string pending_cmd_;

  // Thread
  std::atomic<bool> is_running_{true};
  std::thread task_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->start();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
