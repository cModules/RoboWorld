#pragma once

#include "robot.h"
#include <vector>
#include <memory>

namespace orolp {

enum class MoveType { JOINT, LINEAR, CIRCULAR, SPLINE };

struct Waypoint {
    MoveType type = MoveType::JOINT;
    std::vector<double> target;   // Joint angles OR flattened pose matrix
    double speed = 50.0;          // % or mm/s depending on context
    double blend_radius = 0.0;    // mm
    std::string frame_id;         // Reference frame name
};

/**
 * @brief Discretized trajectory with time, position, velocity, acceleration.
 */
struct Trajectory {
    struct Sample {
        double t;
        std::vector<double> q;      // Joint positions
        std::vector<double> dq;     // Joint velocities
        std::vector<double> ddq;    // Joint accelerations
    };
    std::vector<Sample> samples;
    double duration() const;
};

class MotionPlanner {
public:
    // Joint-space trapezoidal/S-curve interpolation
    static Trajectory planJointMove(
        const RobotModel& robot,
        const Waypoint& start,
        const Waypoint& end,
        double time_step = 0.004    // 250 Hz servo loop
    );

    // Cartesian linear move with joint-space optimization (singularity avoidance)
    static Trajectory planLinearMove(
        const RobotModel& robot,
        const Waypoint& start,
        const Waypoint& end,
        double time_step = 0.004
    );

    // Time-optimal path parameterization (TOPP-RA placeholder)
    static Trajectory optimizeTime(
        const Trajectory& path,
        const JointLimits& limits
    );
};

} // namespace orolp
