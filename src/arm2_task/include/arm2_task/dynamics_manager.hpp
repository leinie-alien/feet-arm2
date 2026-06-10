#pragma once
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "arm2_task/common_units.hpp"

namespace arm2_task {

class DynamicsManager {
public:
    explicit DynamicsManager(const std::string& urdf_path);

    /**
     * @brief 初始化摩擦力与传动比参数
     */
    void initParams(const std::vector<double>& fc, 
                    const std::vector<double>& fv, 
                    const std::vector<double>& ratios,      
                    double alpha);

    /**
     * @brief 动态设置负载状态（500g正方体）
     */
    void setPayloadState(bool has_load, double mass = 0.5, const Eigen::Vector3d& com = Eigen::Vector3d(0, 0, 0.2219));

    /**
     * @brief 抓取后静态检测：根据力矩残差估计负载质量
     */
    double estimatePayloadMass(const JointState& actual_state);

    /**
     * @brief 核心计算接口
     */
    Eigen::VectorXd computeInverseDynamics(const JointState& state);
    Eigen::VectorXd computeFriction(const Eigen::VectorXd& dq);
    Eigen::VectorXd getFeedForwardTorque(const JointState& des_state);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    
    std::vector<arm2_task::FrictionParams> friction_configs_;
    
    // 负载管理私有变量
    pinocchio::FrameIndex last_link_body_frame_idx_; // 末端连杆对应的 BODY frame 索引
    pinocchio::JointIndex payload_joint_idx_;        // 承载末端负载惯量的父关节索引
    pinocchio::Inertia original_inertia_;            // 缓存原始 URDF 惯量
};

} // namespace arm2_task
