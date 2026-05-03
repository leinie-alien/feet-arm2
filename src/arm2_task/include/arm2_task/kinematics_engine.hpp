#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>
#include "arm2_task/common_units.hpp"

namespace arm2_task {

class KinematicsEngine {
public:
    /**
     * @brief 构造函数，初始化模型
     * @param urdf_path URDF文件的绝对路径
     * @param geom 机器人几何参数（L1-L4）
     */
    KinematicsEngine(const std::string& urdf_path, const RobotGeometry& geom);

    /**
     * @brief 正运动学：关节角 -> 末端执行器位姿
     */
    pinocchio::SE3 forwardKinematics(const Eigen::VectorXd& q);

    /**
     * @brief 末端笛卡尔速度映射到关节角速度
     * @param dx 基座坐标系下末端 x 方向线速度 (m/s)
     * @param dy 基座坐标系下末端 y 方向线速度 (m/s)
     * @param dz 基座坐标系下末端 z 方向线速度 (m/s)
     * @param d_pitch 末端局部坐标系下绕 y 轴的角速度 (rad/s)
     * @param d_roll 末端局部坐标系下绕 x 轴的角速度 (rad/s)
     * @param q_current 当前关节角
     * @param dq_out [out] 输出关节角速度
     * @param damping 阻尼最小二乘系数，用于奇异位姿附近稳定求解
     * @return bool 输入合法且求解成功时返回 true
     */
    bool solveJointVelocity(double dx, double dy, double dz,
                            double d_pitch, double d_roll,
                            const Eigen::VectorXd& q_current,
                            Eigen::VectorXd& dq_out,
                            double damping = 1e-4);

    /**
     * @brief 逆运动学：世界坐标系目标 -> 关节角
     * @param target_p_world 目标在基座坐标系下的位置 (x, y, z)
     * @param pitch 期望末端俯仰角 (rad)
     * @param q_out [out] 输出 5 自由度关节角
     * @return bool 是否有解
     */
    bool solveIK(const Eigen::Vector3d& target_p_world, double pitch, Eigen::VectorXd& q_out);
    bool solveIK(const Eigen::Vector3d& target_p_world, double pitch, double roll,
                 Eigen::VectorXd& q_out);
    bool solveIK(const Eigen::Vector3d& target_p_world,
             Eigen::VectorXd& q_out,
             double r_offset,
             double z_offset);
    /**
     * @brief 坐标系转换：将 Link_4 坐标系下的相对偏移转换为基座坐标系下的绝对位置
     * 常用于处理视觉检测到的物体相对位置
     */
    Eigen::Vector3d transformRelativeToWorld(const Eigen::VectorXd& q_current, const Eigen::Vector3d& p_relative);

private:
    /**
     * @brief 内部平面几何解算（原 planar_ik_engine 逻辑）
     */
    bool solvePlanar3Link(double x, double y, double phi, 
                          double l1, double l2, double l3, 
                          Eigen::Vector3d& q_planar);

    pinocchio::Model model_;
    pinocchio::Data data_;
    RobotGeometry geom_;
    
    // 缓存 Frame Index 以提高查找效率
    int link4_frame_id_;
    int tip_frame_id_;
};

} // namespace arm2_task
