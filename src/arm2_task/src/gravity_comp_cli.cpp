#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "arm2_task/common_units.hpp"
#include "arm2_task/dynamics_manager.hpp"

namespace {

constexpr double kDegToRad = M_PI / 180.0;

void print_usage(const char *prog)
{
    std::cout
        << "Usage:\n"
        << "  " << prog << " <urdf_path> [step_deg]\n\n"
        << "Behavior:\n"
        << "  q1=q3=q4=q5=0, q2 sweeps from 0 deg to 180 deg.\n"
        << "  Prints gravity compensation torque for each sample.\n\n"
        << "Example:\n"
        << "  " << prog << " src/arm2_task/urdf/arm2.urdf 5\n";
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        print_usage(argv[0]);
        return argc == 1 ? 0 : 1;
    }

    const std::string urdf_path = argv[1];
    double step_deg = 10.0;
    if (argc == 3) {
        step_deg = std::stod(argv[2]);
    }

    if (step_deg <= 0.0 || step_deg > 180.0) {
        std::cerr << "step_deg must be in (0, 180].\n";
        return 1;
    }

    arm2_task::DynamicsManager dyn_manager(urdf_path);
    arm2_task::JointState state(5);
    state.q.setZero();
    state.dq.setZero();
    state.ddq.setZero();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Gravity compensation sweep loaded URDF: " << urdf_path << "\n";
    std::cout << "q1=q3=q4=q5=0, q2: 0 deg -> 180 deg, step=" << step_deg << " deg\n\n";
    std::cout << "q2_deg,q1_tau,q2_tau,q3_tau,q4_tau,q5_tau\n";

    for (double q2_deg = 0.0; q2_deg <= 180.0 + 1e-9; q2_deg += step_deg) {
        state.q.setZero();
        state.q[1] = q2_deg * kDegToRad;

        const Eigen::VectorXd tau_g = dyn_manager.computeInverseDynamics(state);

        std::cout
            << q2_deg << ","
            << tau_g[0] << ","
            << tau_g[1] << ","
            << tau_g[2] << ","
            << tau_g[3] << ","
            << tau_g[4] << "\n";
    }

    return 0;
}
