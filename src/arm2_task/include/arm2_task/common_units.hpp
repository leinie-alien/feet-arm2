#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace arm2_task {

enum class TaskState {
    IDLE, LOOKOUT, OVERLOOK, GRASPING, HOLDING, PLACING, FAULT
};

enum class ArmEvent {
    START_TASK, TARGET_SPOTTED, IN_POSITION, ACTION_SUCCESS, ACTION_FAILURE, EMERGENCY_STOP
};
/**
 * @brief 机器人几何尺寸参数 (单位: m)
 * 用于解析几何逆解，必须与 URDF 中的坐标偏移量严格一致
 */
struct RobotGeometry {
    double L1; // 基座中心到 Joint1 的高度 (Z方向)
    double L2; // 大臂长度 (Joint1 到 Joint2)
    double L3; // 小臂长度 (Joint2 到 Joint3)
    double L4; // 腕部到末端执行器中心的长度 (Joint3 到 Tip)

    RobotGeometry(double l1 = 0.0, double l2 = 0.0, double l3 = 0.0, double l4 = 0.0)
        : L1(l1), L2(l2), L3(l3), L4(l4) {}
};

/**
 * @brief 摩擦力补偿参数 (对应每个关节)
 * 模型: tau_f = fc * tanh(alpha * dq) + fv * dq
 */
struct FrictionParams {
    double fc = 0.0;    // 库伦摩擦系数 (Coulomb friction)
    double fv = 0.0;    // 粘滞摩擦系数 (Viscous friction)
    double alpha = 60.0; // 连续化平滑系数，防止零速附近的力矩震荡
    double GearRatio = 1.0; // 传动比 (如果有的话)
};

/**
 * @brief 机器人全状态结构体
 * 封装关节空间的所有物理量，便于在算法类之间高效传递
 */
struct JointState {
    Eigen::VectorXd q;   // 位置 (rad)
    Eigen::VectorXd dq;  // 速度 (rad/s)
    Eigen::VectorXd ddq; // 加速度 (rad/s^2)
    Eigen::VectorXd tau; // 力矩 (N·m)

    explicit JointState(int dof = 5) {
        q = dq = ddq = tau = Eigen::VectorXd::Zero(dof);
    }
};

/**
 * @brief 笛卡尔空间位姿定义
 */
struct CartesianPose {
    Eigen::Vector3d pos;      // 位置 (x, y, z)
    Eigen::Vector3d rpy;      // 姿态 (Roll, Pitch, Yaw)
    Eigen::Quaterniond quat;  // 四元数表示的姿态
};

/**
 * @brief 系统静态常量配置
 */
namespace constants {
    const int ARM_DOF = 5;
    const double CONTROL_PERIOD = 0.002; // 500Hz 控制周期 (s)
}

} // namespace arm2_task
