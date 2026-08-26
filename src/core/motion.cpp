#include "motion.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

namespace orolp {

double Trajectory::duration() const {
    if (samples.empty()) return 0.0;
    return samples.back().t;
}

Trajectory MotionPlanner::planJointMove(
    const RobotModel& robot,
    const Waypoint& start,
    const Waypoint& end,
    double time_step)
{
    Trajectory traj;
    const int n_dof = robot.dof();
    const auto lim = robot.limits();

    // Compute max required motion per joint
    double max_time = 0.0;
    std::vector<double> delta(n_dof);
    for (int i = 0; i < n_dof; ++i) {
        delta[i] = end.target[i] - start.target[i];
        double t_acc = lim.vel_max[i] / lim.acc_max[i];  // time to accelerate to max vel
        double d_acc = 0.5 * lim.acc_max[i] * t_acc * t_acc;
        double t_cruise = 0.0;
        if (std::abs(delta[i]) > 2.0 * d_acc) {
            t_cruise = (std::abs(delta[i]) - 2.0 * d_acc) / lim.vel_max[i];
        }
        double t_total = 2.0 * t_acc + t_cruise;
        max_time = std::max(max_time, t_total);
    }

    // Synchronize all joints to max_time (slowest joint dictates cycle)
    int num_samples = static_cast<int>(std::ceil(max_time / time_step)) + 1;
    traj.samples.reserve(num_samples);

    for (int k = 0; k < num_samples; ++k) {
        double t = k * time_step;
        double s = std::min(1.0, t / max_time);  // normalized time [0,1]

        // S-curve blending (smooth acceleration)
        double blend = 0.1;  // 10% of motion for acceleration/deceleration
        double s_eff;
        if (s < blend) {
            s_eff = 0.5 * (s / blend) * (s / blend);
        } else if (s > 1.0 - blend) {
            double s2 = (1.0 - s) / blend;
            s_eff = 1.0 - 0.5 * s2 * s2;
        } else {
            s_eff = (s - 0.5 * blend) / (1.0 - blend);
        }

        Trajectory::Sample sample;
        sample.t = t;
        sample.q.resize(n_dof);
        sample.dq.resize(n_dof);
        sample.ddq.resize(n_dof);

        for (int i = 0; i < n_dof; ++i) {
            sample.q[i] = start.target[i] + s_eff * delta[i];
            // Numerical derivatives for now
            sample.dq[i] = 0.0;
            sample.ddq[i] = 0.0;
        }
        traj.samples.push_back(sample);
    }

    return traj;
}

Trajectory MotionPlanner::planLinearMove(
    const RobotModel& robot,
    const Waypoint& start,
    const Waypoint& end,
    double time_step)
{
    // For now: convert to joint move via IK at each step
    // Full implementation would interpolate in SE(3) and solve IK along path
    return planJointMove(robot, start, end, time_step);
}

Trajectory MotionPlanner::optimizeTime(
    const Trajectory& path,
    const JointLimits& limits)
{
    // Placeholder: return original path
    // TODO: integrate TOPP-RA library for time-optimal reparameterization
    return path;
}

} // namespace orolp
