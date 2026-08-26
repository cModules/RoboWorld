#include "robot.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>

namespace orolp {

class RobotModel::Impl {
public:
    pinocchio::Model model;
    pinocchio::Data data;
    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_base = Eigen::Matrix4d::Identity();
    std::string model_name;
    std::vector<std::string> joint_names;
};

RobotModel::RobotModel(const std::string& urdf_path)
    : pImpl(std::make_unique<Impl>())
{
    pinocchio::urdf::buildModel(urdf_path, pImpl->model);
    pImpl->data = pinocchio::Data(pImpl->model);
    pImpl->model_name = pImpl->model.name;
    pImpl->joint_names = pImpl->model.names;
}

RobotModel::~RobotModel() = default;

Eigen::Matrix4d RobotModel::fk(const std::vector<double>& q) const {
    Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    pinocchio::forwardKinematics(pImpl->model, pImpl->data, q_vec);
    pinocchio::updateFramePlacements(pImpl->model, pImpl->data);

    // End-effector frame (last operational frame)
    const auto& frame = pImpl->model.frames[pImpl->model.nframes - 1];
    Eigen::Matrix4d T = pImpl->data.oMf[frame.parent].toHomogeneousMatrix();

    // Apply base and tool transforms
    return pImpl->T_base * T * pImpl->T_tool;
}

std::vector<std::vector<double>> RobotModel::ik(
    const Eigen::Matrix4d& T_target,
    const std::vector<double>& q_seed,
    int max_solutions) const
{
    std::vector<std::vector<double>> solutions;

    // Remove base/tool offset to get target in model frame
    Eigen::Matrix4d T_model = pImpl->T_base.inverse() * T_target * pImpl->T_tool.inverse();

    // Use Pinocchio's built-in IK (Levenberg-Marquardt style)
    Eigen::VectorXd q_seed_vec;
    if (!q_seed.empty()) {
        q_seed_vec = Eigen::Map<const Eigen::VectorXd>(q_seed.data(), q_seed.size());
    } else {
        q_seed_vec = pinocchio::neutral(pImpl->model);
    }

    // Simple numerical IK: iterate with Jacobian pseudo-inverse
    const int max_iter = 100;
    const double tol = 1e-4;
    Eigen::VectorXd q = q_seed_vec;

    for (int iter = 0; iter < max_iter; ++iter) {
        pinocchio::forwardKinematics(pImpl->model, pImpl->data, q);
        pinocchio::updateFramePlacements(pImpl->model, pImpl->data);

        const auto& frame = pImpl->model.frames[pImpl->model.nframes - 1];
        Eigen::Matrix4d T_current = pImpl->data.oMf[frame.parent].toHomogeneousMatrix();

        // Compute error in SE(3)
        Eigen::Matrix4d T_err = T_model.inverse() * T_current;
        Eigen::VectorXd err = pinocchio::log6(pinocchio::SE3(T_err)).toVector();

        if (err.norm() < tol) {
            std::vector<double> sol(q.data(), q.data() + q.size());
            solutions.push_back(sol);
            break;
        }

        // Jacobian
        Eigen::MatrixXd J(6, pImpl->model.nv);
        pinocchio::computeFrameJacobian(pImpl->model, pImpl->data, q,
                                        pImpl->model.nframes - 1,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);

        // Damped least squares
        double damping = 1e-3;
        Eigen::VectorXd dq = -J.transpose() * (J * J.transpose() + damping * Eigen::MatrixXd::Identity(6,6)).inverse() * err;
        q += dq;

        // Clamp to limits
        for (int i = 0; i < pImpl->model.nq; ++i) {
            q(i) = std::max(pImpl->model.lowerPositionLimit(i),
                            std::min(pImpl->model.upperPositionLimit(i), q(i)));
        }
    }

    // For now return single solution; multi-solution requires analytic solver or global search
    return solutions;
}

Eigen::MatrixXd RobotModel::jacobian(
    const std::vector<double>& q,
    const std::string& frame) const
{
    Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    pinocchio::computeJointJacobians(pImpl->model, pImpl->data, q_vec);

    Eigen::MatrixXd J(6, pImpl->model.nv);
    if (frame.empty()) {
        pinocchio::getFrameJacobian(pImpl->model, pImpl->data,
                                    pImpl->model.nframes - 1,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    } else {
        // TODO: lookup frame by name
        pinocchio::getFrameJacobian(pImpl->model, pImpl->data,
                                    pImpl->model.nframes - 1,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    }
    return J;
}

int RobotModel::dof() const { return pImpl->model.nq; }

JointLimits RobotModel::limits() const {
    JointLimits lim;
    lim.min.resize(pImpl->model.nq);
    lim.max.resize(pImpl->model.nq);
    lim.vel_max.resize(pImpl->model.nv);
    lim.acc_max.resize(pImpl->model.nv);

    for (int i = 0; i < pImpl->model.nq; ++i) {
        lim.min[i] = pImpl->model.lowerPositionLimit(i);
        lim.max[i] = pImpl->model.upperPositionLimit(i);
    }
    for (int i = 0; i < pImpl->model.nv; ++i) {
        lim.vel_max[i] = pImpl->model.velocityLimit(i);
        // Acceleration limits not always present in URDF; use heuristic
        lim.acc_max[i] = lim.vel_max[i] * 5.0;
    }
    return lim;
}

std::string RobotModel::name() const { return pImpl->model_name; }

std::vector<std::string> RobotModel::joint_names() const {
    return pImpl->joint_names;
}

void RobotModel::setToolFrame(const Eigen::Matrix4d& T_tcf) { pImpl->T_tool = T_tcf; }
void RobotModel::setBaseFrame(const Eigen::Matrix4d& T_wcf) { pImpl->T_base = T_wcf; }
Eigen::Matrix4d RobotModel::toolFrame() const { return pImpl->T_tool; }
Eigen::Matrix4d RobotModel::baseFrame() const { return pImpl->T_base; }

} // namespace orolp
