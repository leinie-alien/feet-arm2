#include "arm2_task/kinematics_engine.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <cmath>

namespace arm2_task {

KinematicsEngine::KinematicsEngine(const std::string& urdf_path, const RobotGeometry& geom)
    : geom_(geom) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    
    link4_frame_id_ = model_.getFrameId("Link_4");
    tip_frame_id_ = model_.getFrameId("Link_5");
    if (tip_frame_id_ >= static_cast<int>(model_.frames.size())) {
        tip_frame_id_ = link4_frame_id_;
    }
}

bool KinematicsEngine::solveIK(const Eigen::Vector3d& target_p_world, double pitch, Eigen::VectorXd& q_out) {
    q_out = Eigen::VectorXd::Zero(5);

    // 1. 计算底座旋转角 (Joint 0)
    // 假设 base 位于 (0,0,0)，q0 将平面对准目标点
    q_out[0] = std::atan2(target_p_world.y(), target_p_world.x());

    // 2. 将 3D 目标投影到 2D 平面 (r, z)
    // r 是水平距离，z 是相对于第一个水平关节(L1偏移后)的高度
    double r = std::sqrt(target_p_world.x() * target_p_world.x() + target_p_world.y() * target_p_world.y());
    double z_rel = target_p_world.z() - geom_.L1;

    // 3. 调用平面 3-Link 解析算法
    // 在该平面内，x 轴即为 r 方向，y 轴即为 z_rel 方向
    Eigen::Vector3d q_planar;
    if (!solvePlanar3Link(r, z_rel, pitch, geom_.L2, geom_.L3, geom_.L4, q_planar)) {
        return false; // 目标点超程或几何不可达
    }

    // 4. 映射回 5 自由度关节空间
    // q_planar 分别对应大臂、小臂、手腕的关节角
    q_out[1] = q_planar[0];
    q_out[2] = q_planar[1];
    q_out[3] = q_planar[2];
    q_out[4] = 0.0; // 默认 Roll 角为 0

    return true;
}

bool KinematicsEngine::solveIK(const Eigen::Vector3d& target_p_world, double pitch, double roll,
                             Eigen::VectorXd& q_out) {
    if (!solveIK(target_p_world, pitch, q_out)) {
        return false;
    }
    q_out[4] = roll;
    return true;
}

bool KinematicsEngine::solveIK(const Eigen::Vector3d& target_p_world,
                             Eigen::VectorXd& q_out,
                             double r_offset,
                             double z_offset) {

     q_out = Eigen::VectorXd::Zero(5);

    // 1. 计算底座旋转角 (Joint 0)
    // 假设 base 位于 (0,0,0)，q0 将平面对准目标点
    q_out[0] = std::atan2(target_p_world.y(), target_p_world.x());

    // 2. 将 3D 目标投影到 2D 平面 (r, z)
    // r 是水平距离，z 是相对于第一个水平关节(L1偏移后)的高度
    double pitch_look = - std::atan2(z_offset, -r_offset);
    double r = std::sqrt(target_p_world.x() * target_p_world.x() + target_p_world.y() * target_p_world.y());
    double z_rel = target_p_world.z() - geom_.L1;

    // 3. 调用平面 3-Link 解析算法
    // 在该平面内，x 轴即为 r 方向，y 轴即为 z_rel 方向
    Eigen::Vector3d q_planar;
    if (!solvePlanar3Link(r + r_offset, z_rel + z_offset, pitch_look, geom_.L2, geom_.L3, geom_.L4, q_planar)) {
        return false; // 目标点超程或几何不可达
    }

    // 4. 映射回 5 自由度关节空间
    // q_planar 分别对应大臂、小臂、手腕的关节角
    q_out[1] = q_planar[0];
    q_out[2] = q_planar[1];
    q_out[3] = q_planar[2];
    q_out[4] = 0.0; // 默认 Roll 角为 0

    return true;
}

bool KinematicsEngine::solvePlanar3Link(double x, double y, double phi, 
                                       double l1, double l2, double l3, 
                                       Eigen::Vector3d& q_planar) {
    // 寻找肘部（Joint 3）的位置
    double x_w = x - l3 * std::cos(phi);
    double y_w = y - l3 * std::sin(phi);

    // 转化为 2-Link (l1, l2) 到达 (x_w, y_w) 的问题
    double r_sq = x_w * x_w + y_w * y_w;
    double r = std::sqrt(r_sq);

    // 余弦定理计算中间关节 q2 (对应你的 Link 3)
    double cos_q2 = (r_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (std::abs(cos_q2) > 1.0) return false; // 无法构成的三角形

    // 习惯上选择“下肘”位姿（减小重心偏移）
    double q2 = -std::acos(cos_q2); 

    // 计算第一个关节 q1
    double theta = std::atan2(y_w, x_w);
    double alpha = std::atan2(l2 * std::sin(q2), l1 + l2 * std::cos(q2));
    double q1 = theta - alpha;

    // 计算第三个关节 q3 保证末端姿态 phi
    // phi = q1 + q2 + q3
    double q3 = phi - q1 - q2;

    q_planar << q1, q2, q3;
    return true;
}

/**
 * @brief 计算正向运动学
 */
pinocchio::SE3 KinematicsEngine::forwardKinematics(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    return data_.oMf[link4_frame_id_];
}

bool KinematicsEngine::solveJointVelocity(double dx, double dy, double dz,
                                          double d_pitch, double d_roll,
                                          const Eigen::VectorXd& q_current,
                                          Eigen::VectorXd& dq_out,
                                          double damping) {
    if (q_current.size() != model_.nv) {
        return false;
    }

    pinocchio::forwardKinematics(model_, data_, q_current);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_current);

    Eigen::MatrixXd jacobian(6, model_.nv);
    pinocchio::getFrameJacobian(model_,
                                data_,
                                tip_frame_id_,
                                pinocchio::ReferenceFrame::WORLD,
                                jacobian);

    const Eigen::Matrix3d& rotation_world_tip = data_.oMf[tip_frame_id_].rotation();

    Eigen::Matrix<double, 6, 1> desired_twist = Eigen::Matrix<double, 6, 1>::Zero();
    desired_twist.tail<3>() << dx, dy, dz;

    // 用户输入的 pitch / roll 视为末端局部坐标系下的角速度命令。
    const Eigen::Vector3d omega_tip(d_roll, d_pitch, 0.0);
    desired_twist.head<3>() = rotation_world_tip * omega_tip;

    const double lambda2 = damping * damping;
    const Eigen::Matrix<double, 6, 6> regularized =
        jacobian * jacobian.transpose() + lambda2 * Eigen::Matrix<double, 6, 6>::Identity();

    dq_out = jacobian.transpose() * regularized.ldlt().solve(desired_twist);
    return dq_out.allFinite();
}


} // namespace arm2_task
