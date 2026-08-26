#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace orolp {

struct JointLimits {
    std::vector<double> min;
    std::vector<double> max;
    std::vector<double> vel_max;
    std::vector<double> acc_max;
};

/**
 * @brief Unified robot model based on URDF + Pinocchio.
 *        Supports arbitrary kinematic trees (single/multi-arm).
 */
class RobotModel {
public:
    explicit RobotModel(const std::string& urdf_path);
    ~RobotModel();

    // Forward Kinematics: joint angles -> end-effector SE(3) pose
    Eigen::Matrix4d fk(const std::vector<double>& q) const;

    // Inverse Kinematics: SE(3) pose -> joint angle solutions
    // Returns up to max_solutions; uses q_seed for numerical continuation.
    std::vector<std::vector<double>> ik(
        const Eigen::Matrix4d& T_target,
        const std::vector<double>& q_seed = {},
        int max_solutions = 8
    ) const;

    // Geometric Jacobian (6 x n_dof) at given frame
    Eigen::MatrixXd jacobian(
        const std::vector<double>& q,
        const std::string& frame = ""  // empty = end-effector
    ) const;

    // Model properties
    int dof() const;
    JointLimits limits() const;
    std::string name() const;
    std::vector<std::string> joint_names() const;

    // Tool / Base frame offsets (brand-specific calibration)
    void setToolFrame(const Eigen::Matrix4d& T_tcf);
    void setBaseFrame(const Eigen::Matrix4d& T_wcf);
    Eigen::Matrix4d toolFrame() const;
    Eigen::Matrix4d baseFrame() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace orolp
