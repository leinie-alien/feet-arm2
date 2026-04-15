#include "arm2_task/dynamics_manager.hpp"
#include <pinocchio/algorithm/rnea.hpp> 
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <iostream>
#include <stdexcept>

namespace arm2_task {

DynamicsManager::DynamicsManager(const std::string& urdf_path) {
    // 加载 URDF，此时惯量已包含转子补偿
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    
    // 负载的物理惯量需要挂到 Link_5 对应 BODY frame 的父关节上。
    last_link_body_frame_idx_ = model_.getBodyId("Link_5");
    if (last_link_body_frame_idx_ >= model_.frames.size()) {
        throw std::runtime_error("Invalid BODY frame index for Link_5.");
    }
    payload_joint_idx_ = model_.frames[last_link_body_frame_idx_].parentJoint;
    if (payload_joint_idx_ <= 0 || payload_joint_idx_ >= model_.inertias.size()) {
        throw std::runtime_error("Invalid parent joint index for Link_5 payload inertia.");
    }
    original_inertia_ = model_.inertias[payload_joint_idx_];

    friction_configs_.resize(model_.nv);
}

// 在 src/dynamics_manager.cpp 中
void DynamicsManager::initParams(const std::vector<double>& fc, 
                                 const std::vector<double>& fv, 
                                 const std::vector<double>& ratios,
                                 double alpha) { // 补全第四个参数
    for (int i = 0; i < (int)model_.nv; ++i) {
        friction_configs_[i].fc = fc[i];
        friction_configs_[i].fv = fv[i]; 
        friction_configs_[i].GearRatio = ratios[i];
        friction_configs_[i].alpha = alpha; // 确保 alpha 被赋值
    }
}

void DynamicsManager::setPayloadState(bool has_load, double mass, const Eigen::Vector3d& com) {
    if (has_load) {
        // 计算正方体转动惯量: I = 1/6 * m * a^2 (a=0.25m)
        double i_val = (1.0/6.0) * mass * std::pow(0.25, 2);
        Eigen::Matrix3d inertia_mat = Eigen::Matrix3d::Identity() * i_val;
        
        pinocchio::Inertia load_inertia(mass, com, inertia_mat);
        // 惯量叠加
        model_.inertias[payload_joint_idx_] = original_inertia_ + load_inertia;
    } else {
        model_.inertias[payload_joint_idx_] = original_inertia_;
    }
}

double DynamicsManager::estimatePayloadMass(const JointState& actual_state) {
    // 1. 计算当前位姿下的空载理论重力矩 g(q)
    // 采用静态计算，dq 和 ddq 设为 0
    Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model_.nv);
    
    // 临时恢复空载惯量模型以计算基准重力矩
    pinocchio::Inertia current_tmp = model_.inertias[payload_joint_idx_];
    model_.inertias[payload_joint_idx_] = original_inertia_;
    
    // 使用 RNEA 计算空载状态下的重力项
    Eigen::VectorXd tau_gravity_empty = pinocchio::rnea(model_, data_, actual_state.q, zero_v, zero_v);
    
    // 2. 立即恢复模型状态，确保后续计算一致性
    model_.inertias[payload_joint_idx_] = current_tmp;

    // 3. 计算力矩残差
    // 残差 = 实际观测到的力矩 - 预测的空载重力矩 - 预测的摩擦力矩
    Eigen::VectorXd tau_residual = actual_state.tau - computeFriction(actual_state.dq) - tau_gravity_empty;

    // 4. 基于解析几何的质量估计 (以 Joint 2 为主参考轴)
    // 获取重力加速度常量 (Pinocchio 默认为 9.81)
    const double g_acc = std::abs(model_.gravity.linear().z());
    
    // 动态获取 L2 长度：通过 Link_2 相对于 Joint 2 的偏移自动获取，避免硬编码 0.35
    // 假设 Joint 1 的位置在坐标原点，Joint 2 的位置由 URDF 决定
    double l_arm = model_.jointPlacements[model_.getJointId("Pitch_2")].translation().norm();
    if (l_arm < 1e-3) l_arm = 0.35; // 如果获取失败，回退到默认值

    // 计算有效力臂：l_eff = L2 * cos(q[1])
    // 注意：q[1] 为大臂俯仰角，0度通常代表水平方向
    double l_eff = l_arm * std::cos(actual_state.q[1]);

    // 5. 奇异位姿保护
    // 如果机械臂处于垂直状态 (cos(q1) -> 0)，力矩对质量不敏感，此时计算会产生巨大误差
    if (std::abs(l_eff) < 0.05) {
        // 返回 0.0 或保留上次估计值，防止由于除以极小值导致质量估计值飙升
        return 0.0; 
    }

    // 质量 = 力矩残差 / (g * 有效力臂)
    double estimated_m = tau_residual[1] / (g_acc * l_eff);

    // 6. 结果平滑与限幅
    return std::max(0.0, std::abs(estimated_m));
}

Eigen::VectorXd DynamicsManager::computeInverseDynamics(const JointState& state) {
    return pinocchio::rnea(model_, data_, state.q, state.dq, state.ddq);
}

Eigen::VectorXd DynamicsManager::computeFriction(const Eigen::VectorXd& dq) {
    Eigen::VectorXd tau_f = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < (int)model_.nv; ++i) {
        const auto& p = friction_configs_[i];
        // 考虑传动比对粘性摩擦的平方倍影响
        
        tau_f[i] = p.fc * std::tanh(p.alpha * dq[i]) + p.fv * dq[i] * std::pow(p.GearRatio, 2);
    }
    return tau_f;
}

Eigen::VectorXd DynamicsManager::getFeedForwardTorque(const JointState& des_state) {
    return computeInverseDynamics(des_state) + computeFriction(des_state.dq);
}

} // namespace arm2_task
