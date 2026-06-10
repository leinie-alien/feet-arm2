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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "arm2_task/common_units.hpp"
#include "arm2_task/kinematics_engine.hpp"

using namespace std::chrono_literals;

class FollowTargetNode : public rclcpp::Node
{
public:
    FollowTargetNode() : Node("follow_target_node")
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
        std::string urdf = share_dir + "/" + rel_urdf_path;
        double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
        double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
        double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
        double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

        state_topic_ =
            this->declare_parameter("follow_target.state_topic", "/arm2/_lowState/joint");
        target_topic_ =
            this->declare_parameter("follow_target.target_topic", "/joint_target_state");
        ready_topic_ =
            this->declare_parameter("follow_target.ready_topic", "/robot_driver/ready");
        pick_service_name_ =
            this->declare_parameter("follow_target.pick_service", std::string("get_pick_pos"));
        object_name_ =
            this->declare_parameter("follow_target.object_name", std::string("box"));
        mode_service_name_ =
            this->declare_parameter("follow_target.mode_service", std::string("set_controller_mode"));
        mode_name_ =
            this->declare_parameter("follow_target.mode_name", std::string("tracking"));
        request_mode_on_startup_ =
            this->declare_parameter("follow_target.request_mode_on_startup", true);
        acquisition_hz_ = this->declare_parameter("follow_target.acquisition_hz", 10.0);
        publish_hz_ = this->declare_parameter("follow_target.publish_hz", 50.0);
        target_qos_depth_ = this->declare_parameter("follow_target.target_qos_depth", 10);
        pick_wait_service_sec_ =
            this->declare_parameter("follow_target.pick_wait_service_sec", 1.0);
        pick_response_timeout_sec_ =
            this->declare_parameter("follow_target.pick_response_timeout_sec", 0.5);
        transform_timeout_sec_ =
            this->declare_parameter("follow_target.transform_timeout_sec", 0.5);
        target_stale_timeout_sec_ =
            this->declare_parameter("follow_target.target_stale_timeout_sec", 0.35);
        radial_offset_ = this->declare_parameter("follow_target.radial_offset", -0.08);
        height_offset_ = this->declare_parameter("follow_target.height_offset", 0.20);
        follow_tool_pitch_ =
            this->declare_parameter("follow_target.follow_tool_pitch", -1.57);
        follow_roll_ = this->declare_parameter("follow_target.follow_roll", 0.0);
        camera_pitch_blend_ =
            this->declare_parameter("follow_target.camera_pitch_blend", 1.0);
        camera_pitch_lower_ =
            this->declare_parameter("follow_target.camera_pitch_lower", -2.0);
        camera_pitch_upper_ =
            this->declare_parameter("follow_target.camera_pitch_upper", 1.57);
        position_filter_alpha_ =
            this->declare_parameter("follow_target.position_filter_alpha", 0.25);
        max_abs_velocity_ =
            this->declare_parameter("follow_target.max_abs_velocity", 1.5);
        max_abs_acceleration_ =
            this->declare_parameter("follow_target.max_abs_acceleration", 8.0);
        log_target_ = this->declare_parameter("follow_target.log_target", false);
        angle_window_lower_ = this->declare_parameter<std::vector<double>>(
            "follow_target.angle_window_lower",
            std::vector<double>{-M_PI, -0.5, -3.0, -2.0, -M_PI});
        angle_window_upper_ = this->declare_parameter<std::vector<double>>(
            "follow_target.angle_window_upper",
            std::vector<double>{M_PI, 3.5, 0.15, 1.57, M_PI});

        normalize_angle_windows();
        acquisition_hz_ = std::max(1.0, acquisition_hz_);
        publish_hz_ = std::max(1.0, publish_hz_);
        target_qos_depth_ = std::max(1, target_qos_depth_);
        pick_wait_service_sec_ = std::max(0.0, pick_wait_service_sec_);
        pick_response_timeout_sec_ = std::max(0.0, pick_response_timeout_sec_);
        transform_timeout_sec_ = std::max(0.0, transform_timeout_sec_);
        target_stale_timeout_sec_ = std::max(0.0, target_stale_timeout_sec_);
        camera_pitch_blend_ = std::clamp(camera_pitch_blend_, 0.0, 1.0);
        position_filter_alpha_ = std::clamp(position_filter_alpha_, 0.0, 1.0);
        max_abs_velocity_ = std::max(0.0, max_abs_velocity_);
        max_abs_acceleration_ = std::max(0.0, max_abs_acceleration_);

        current_q_ = Eigen::VectorXd::Zero(kDof);
        target_q_ = Eigen::VectorXd::Zero(kDof);
        target_dq_ = Eigen::VectorXd::Zero(kDof);
        target_ddq_ = Eigen::VectorXd::Zero(kDof);
        prev_target_dq_ = Eigen::VectorXd::Zero(kDof);
        last_continuous_target_q_ = Eigen::VectorXd::Zero(kDof);

        load_look_at_preset();

        kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(
            urdf, arm2_task::RobotGeometry(l1, l2, l3, l4));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto target_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(target_qos_depth_)))
                              .reliable()
                              .durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        target_pub_ = this->create_publisher<robot_msgs::msg::RobotState>(target_topic_, target_qos);
        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            state_qos,
            std::bind(&FollowTargetNode::state_callback, this, std::placeholders::_1));
        driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&FollowTargetNode::driver_ready_callback, this, std::placeholders::_1));

        pick_client_ =
            this->create_client<robot_msgs::srv::GetPickPos>(pick_service_name_);
        mode_client_ =
            this->create_client<robot_msgs::srv::SetControllerMode>(mode_service_name_);

        const auto acquisition_period = std::chrono::duration<double>(1.0 / acquisition_hz_);
        acquisition_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(acquisition_period),
            std::bind(&FollowTargetNode::acquire_target, this));

        const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
            std::bind(&FollowTargetNode::publish_target, this));

        if (request_mode_on_startup_) {
            mode_request_timer_ = this->create_wall_timer(
                1s, std::bind(&FollowTargetNode::request_mode_if_needed, this));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Follow target node started. object=%s pick_service=%s state_topic=%s target_topic=%s mode=%s acquisition_hz=%.1f publish_hz=%.1f radial_offset=%.3f height_offset=%.3f",
            object_name_.c_str(),
            pick_service_name_.c_str(),
            state_topic_.c_str(),
            target_topic_.c_str(),
            mode_name_.c_str(),
            acquisition_hz_,
            publish_hz_,
            radial_offset_,
            height_offset_);
    }

private:
    static constexpr int kDof = 5;
    static constexpr double kAngleWrapLimit = M_PI;
    static constexpr double kFullTurnRad = 2.0 * M_PI;

    static std::string format_vector(const Eigen::VectorXd &values)
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

    void normalize_angle_windows()
    {
        const std::array<double, kDof> default_lower = {-M_PI, -0.5, -3.0, -2.0, -M_PI};
        const std::array<double, kDof> default_upper = {M_PI, 3.5, 0.15, 1.57, M_PI};

        if (angle_window_lower_.size() != static_cast<size_t>(kDof)) {
            angle_window_lower_.assign(default_lower.begin(), default_lower.end());
        }
        if (angle_window_upper_.size() != static_cast<size_t>(kDof)) {
            angle_window_upper_.assign(default_upper.begin(), default_upper.end());
        }

        for (int i = 0; i < kDof; ++i) {
            if (!std::isfinite(angle_window_lower_[static_cast<size_t>(i)]) ||
                !std::isfinite(angle_window_upper_[static_cast<size_t>(i)]) ||
                angle_window_lower_[static_cast<size_t>(i)] >=
                    angle_window_upper_[static_cast<size_t>(i)])
            {
                angle_window_lower_[static_cast<size_t>(i)] = default_lower[static_cast<size_t>(i)];
                angle_window_upper_[static_cast<size_t>(i)] = default_upper[static_cast<size_t>(i)];
            }
        }
    }

    void load_look_at_preset()
    {
        const auto angles_deg =
            this->declare_parameter("presets.look_at", std::vector<double>(kDof, 0.0));
        if (angles_deg.size() != static_cast<size_t>(kDof)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Preset 'look_at' size=%zu, expected=%d. Fallback look-at pose disabled.",
                angles_deg.size(),
                kDof);
            has_look_at_preset_ = false;
            return;
        }

        look_at_preset_ = Eigen::VectorXd::Zero(kDof);
        for (int i = 0; i < kDof; ++i) {
            look_at_preset_[i] = angles_deg[static_cast<size_t>(i)] * M_PI / 180.0;
        }
        has_look_at_preset_ = true;
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

        return std::clamp(*best_it, lower, upper);
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        if (!msg || msg->motor_state.size() < static_cast<size_t>(kDof)) {
            return;
        }

        for (int i = 0; i < kDof; ++i) {
            const auto &motor = msg->motor_state[static_cast<size_t>(i)];
            if (!motor.valid || !std::isfinite(motor.q) || !std::isfinite(motor.dq)) {
                return;
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < kDof; ++i) {
            current_q_[i] = msg->motor_state[static_cast<size_t>(i)].q;
        }
        if (!has_target_) {
            last_continuous_target_q_ = current_q_;
        }
        has_state_ = true;
    }

    void driver_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        if (driver_ready_ != msg->data) {
            RCLCPP_INFO(
                this->get_logger(),
                "Follow target driver ready changed: %s -> %s",
                driver_ready_ ? "true" : "false",
                msg->data ? "true" : "false");
        }
        driver_ready_ = msg->data;
    }

    bool call_pick_service_sync(const std::string &object_name, geometry_msgs::msg::Pose *out_pose)
    {
        const auto service_wait_timeout =
            std::chrono::duration<double>(pick_wait_service_sec_);
        if (!pick_client_->wait_for_service(service_wait_timeout)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for pick service %s.",
                pick_service_name_.c_str());
            return false;
        }

        auto request = std::make_shared<robot_msgs::srv::GetPickPos::Request>();
        request->object_name = object_name;

        auto result_future = pick_client_->async_send_request(request);
        if (result_future.wait_for(std::chrono::duration<double>(pick_response_timeout_sec_)) !=
            std::future_status::ready)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Pick service call for [%s] timed out.",
                object_name.c_str());
            return false;
        }

        const auto response = result_future.get();
        if (!response || response->pick_pose.header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Pick service returned an empty pose for [%s].",
                object_name.c_str());
            return false;
        }

        try {
            const rclcpp::Time capture_time = response->pick_pose.header.stamp;
            if (!tf_buffer_->canTransform(
                    "world",
                    response->pick_pose.header.frame_id,
                    capture_time,
                    tf2::durationFromSec(transform_timeout_sec_)))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "TF lookup failed for %s -> world.",
                    response->pick_pose.header.frame_id.c_str());
                return false;
            }

            const geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
                "world", response->pick_pose.header.frame_id, capture_time);
            if (out_pose != nullptr) {
                tf2::doTransform(response->pick_pose.pose, *out_pose, t_stamped);
            }
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TF2 Error while tracking [%s]: %s",
                object_name.c_str(),
                ex.what());
            return false;
        }
    }

    Eigen::Vector3d build_follow_point(const geometry_msgs::msg::Pose &target_world_pose) const
    {
        Eigen::Vector3d follow_point(
            target_world_pose.position.x,
            target_world_pose.position.y,
            target_world_pose.position.z + height_offset_);

        const double radial_distance =
            std::hypot(target_world_pose.position.x, target_world_pose.position.y);
        if (radial_distance > 1e-6) {
            follow_point.x() += radial_offset_ * target_world_pose.position.x / radial_distance;
            follow_point.y() += radial_offset_ * target_world_pose.position.y / radial_distance;
        }

        return follow_point;
    }

    void apply_camera_look_at(
        const geometry_msgs::msg::Pose &target_world_pose,
        Eigen::VectorXd &q_goal)
    {
        q_goal[0] =
            std::atan2(target_world_pose.position.y, target_world_pose.position.x);

        const pinocchio::SE3 link4_pose = kin_engine_->forwardKinematics(q_goal);
        const double horizontal_distance = std::hypot(
            target_world_pose.position.x - link4_pose.translation().x(),
            target_world_pose.position.y - link4_pose.translation().y());
        const double target_link4_pitch = std::atan2(
            target_world_pose.position.z - link4_pose.translation().z(),
            horizontal_distance);
        const double desired_pitch_3 = target_link4_pitch - q_goal[1] - q_goal[2];
        const double clamped_pitch =
            std::clamp(desired_pitch_3, camera_pitch_lower_, camera_pitch_upper_);

        q_goal[3] =
            (1.0 - camera_pitch_blend_) * q_goal[3] + camera_pitch_blend_ * clamped_pitch;
        q_goal[3] = std::clamp(q_goal[3], camera_pitch_lower_, camera_pitch_upper_);
    }

    bool compute_follow_goal(
        const geometry_msgs::msg::Pose &target_world_pose,
        Eigen::VectorXd &q_goal)
    {
        const Eigen::Vector3d follow_point = build_follow_point(target_world_pose);
        bool used_look_at_fallback = false;

        if (!kin_engine_->solveIK(follow_point, follow_tool_pitch_, q_goal)) {
            if (!has_look_at_preset_) {
                return false;
            }
            q_goal = look_at_preset_;
            used_look_at_fallback = true;
        }

        q_goal[4] = follow_roll_;
        apply_camera_look_at(target_world_pose, q_goal);

        if (!q_goal.allFinite()) {
            return false;
        }

        if (used_look_at_fallback) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Follow IK failed; using look_at preset fallback while keeping the target in camera view.");
        }
        return true;
    }

    void update_target_state(const Eigen::VectorXd &desired_q, const rclcpp::Time &stamp)
    {
        Eigen::VectorXd previous_continuous_q = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd previous_target_dq = Eigen::VectorXd::Zero(kDof);
        rclcpp::Time previous_stamp(0, 0, RCL_ROS_TIME);
        bool has_previous_target = false;
        bool has_reference = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            has_previous_target = has_target_;
            if (has_previous_target) {
                previous_continuous_q = last_continuous_target_q_;
                previous_target_dq = prev_target_dq_;
                previous_stamp = last_target_stamp_;
                has_reference = true;
            } else if (has_state_) {
                previous_continuous_q = current_q_;
                has_reference = true;
            }
        }

        Eigen::VectorXd continuous_q = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd windowed_q = Eigen::VectorXd::Zero(kDof);
        for (int i = 0; i < kDof; ++i) {
            continuous_q[i] = unwrap_continuous_angle(
                desired_q[i], has_reference, previous_continuous_q[i]);
            windowed_q[i] = choose_windowed_angle(i, continuous_q[i]);
        }

        Eigen::VectorXd dq = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd ddq = Eigen::VectorXd::Zero(kDof);
        if (has_previous_target && stamp > previous_stamp) {
            const double dt = (stamp - previous_stamp).seconds();
            if (dt > std::numeric_limits<double>::epsilon()) {
                dq = (continuous_q - previous_continuous_q) / dt;
                for (int i = 0; i < kDof; ++i) {
                    dq[i] = std::clamp(dq[i], -max_abs_velocity_, max_abs_velocity_);
                }

                ddq = (dq - previous_target_dq) / dt;
                for (int i = 0; i < kDof; ++i) {
                    ddq[i] = std::clamp(
                        ddq[i], -max_abs_acceleration_, max_abs_acceleration_);
                }
            }
        }

        for (int i = 0; i < kDof; ++i) {
            const double lower = angle_window_lower_[static_cast<size_t>(i)];
            const double upper = angle_window_upper_[static_cast<size_t>(i)];
            const bool at_lower_limit = std::abs(windowed_q[i] - lower) < 1e-9;
            const bool at_upper_limit = std::abs(windowed_q[i] - upper) < 1e-9;
            if ((at_lower_limit && dq[i] < 0.0) || (at_upper_limit && dq[i] > 0.0)) {
                dq[i] = 0.0;
                ddq[i] = 0.0;
            }
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            target_q_ = windowed_q;
            target_dq_ = dq;
            target_ddq_ = ddq;
            prev_target_dq_ = dq;
            last_continuous_target_q_ = continuous_q;
            last_target_stamp_ = stamp;
            last_observation_time_ = std::chrono::steady_clock::now();
            target_stale_latched_ = false;
            has_target_ = true;
        }
    }

    void acquire_target()
    {
        if (!driver_ready_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Follow target waiting for /robot_driver/ready.");
            return;
        }

        if (!has_state_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Follow target waiting for the first valid robot joint state.");
            return;
        }

        geometry_msgs::msg::Pose target_world_pose;
        if (!call_pick_service_sync(object_name_, &target_world_pose)) {
            return;
        }

        Eigen::Vector3d measured_position(
            target_world_pose.position.x,
            target_world_pose.position.y,
            target_world_pose.position.z);
        if (!has_filtered_target_) {
            filtered_target_position_ = measured_position;
            has_filtered_target_ = true;
        } else {
            filtered_target_position_ =
                (1.0 - position_filter_alpha_) * filtered_target_position_ +
                position_filter_alpha_ * measured_position;
        }

        geometry_msgs::msg::Pose filtered_pose = target_world_pose;
        filtered_pose.position.x = filtered_target_position_.x();
        filtered_pose.position.y = filtered_target_position_.y();
        filtered_pose.position.z = filtered_target_position_.z();

        Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
        if (!compute_follow_goal(filtered_pose, q_goal)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed to build a valid follow goal for [%s].",
                object_name_.c_str());
            return;
        }

        update_target_state(q_goal, this->get_clock()->now());

        if (log_target_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "Follow target pose=[%.3f, %.3f, %.3f] q=%s",
                filtered_pose.position.x,
                filtered_pose.position.y,
                filtered_pose.position.z,
                format_vector(q_goal).c_str());
        }
    }

    void publish_target()
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd ddq = Eigen::VectorXd::Zero(kDof);
        bool should_publish = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!has_target_) {
                return;
            }

            const double observation_age_sec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_observation_time_).count();
            if (observation_age_sec > target_stale_timeout_sec_) {
                if (!target_stale_latched_) {
                    target_stale_latched_ = true;
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Follow target observation is stale (age=%.3fs). Stop publishing to let downstream hold position.",
                        observation_age_sec);
                }
                return;
            }

            q = target_q_;
            dq = target_dq_;
            ddq = target_ddq_;
            should_publish = true;
        }

        if (!should_publish) {
            return;
        }

        robot_msgs::msg::RobotState target_msg;
        target_msg.motor_state.resize(kDof);
        for (int i = 0; i < kDof; ++i) {
            auto &motor = target_msg.motor_state[static_cast<size_t>(i)];
            motor.q = static_cast<float>(q[i]);
            motor.dq = static_cast<float>(dq[i]);
            motor.ddq = static_cast<float>(ddq[i]);
            motor.tau_est = 0.0f;
            motor.cur = 0.0f;
            motor.valid = true;
        }
        target_pub_->publish(target_msg);

        if (log_target_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Published follow target q=%s dq=%s ddq=%s",
                format_vector(q).c_str(),
                format_vector(dq).c_str(),
                format_vector(ddq).c_str());
        }
    }

    void request_mode_if_needed()
    {
        if (!request_mode_on_startup_ || mode_requested_) {
            return;
        }

        if (!mode_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for mode service %s before enabling follow mode.",
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
                    mode_requested_ = true;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Follow mode enabled via %s: %s",
                        mode_service_name_.c_str(),
                        response->message.c_str());
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Follow mode request rejected: %s",
                        response->message.c_str());
                }
            });
    }

    std::mutex data_mutex_;
    bool driver_ready_{false};
    bool has_state_{false};
    bool has_target_{false};
    bool has_filtered_target_{false};
    bool has_look_at_preset_{false};
    bool mode_requested_{false};
    bool request_mode_on_startup_{true};
    bool log_target_{false};
    bool target_stale_latched_{false};

    double acquisition_hz_{10.0};
    double publish_hz_{50.0};
    double pick_wait_service_sec_{1.0};
    double pick_response_timeout_sec_{0.5};
    double transform_timeout_sec_{0.5};
    double target_stale_timeout_sec_{0.35};
    double radial_offset_{-0.08};
    double height_offset_{0.20};
    double follow_tool_pitch_{-1.57};
    double follow_roll_{0.0};
    double camera_pitch_blend_{1.0};
    double camera_pitch_lower_{-2.0};
    double camera_pitch_upper_{1.57};
    double position_filter_alpha_{0.25};
    double max_abs_velocity_{1.5};
    double max_abs_acceleration_{8.0};
    int target_qos_depth_{10};

    std::string state_topic_;
    std::string target_topic_;
    std::string ready_topic_;
    std::string pick_service_name_;
    std::string object_name_;
    std::string mode_service_name_;
    std::string mode_name_;

    std::vector<double> angle_window_lower_;
    std::vector<double> angle_window_upper_;

    Eigen::VectorXd current_q_;
    Eigen::VectorXd target_q_;
    Eigen::VectorXd target_dq_;
    Eigen::VectorXd target_ddq_;
    Eigen::VectorXd prev_target_dq_;
    Eigen::VectorXd last_continuous_target_q_;
    Eigen::VectorXd look_at_preset_{Eigen::VectorXd::Zero(kDof)};
    Eigen::Vector3d filtered_target_position_{Eigen::Vector3d::Zero()};
    rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_observation_time_{};

    std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr target_pub_;
    rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
    rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
    rclcpp::TimerBase::SharedPtr acquisition_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr mode_request_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTargetNode>());
    rclcpp::shutdown();
    return 0;
}
