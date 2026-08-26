#pragma once

#include "robot.h"
#include "motion.h"
#include <string>
#include <vector>
#include <memory>

namespace orolp {

struct Mesh {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
};

struct Obstacle {
    Mesh geometry;
    Eigen::Matrix4d pose;
    std::string name;
};

/**
 * @brief Collision detection world using FCL (Flexible Collision Library).
 */
class CollisionWorld {
public:
    CollisionWorld();
    ~CollisionWorld();

    void addRobot(const RobotModel& robot, const std::string& name);
    void addObstacle(const Mesh& mesh, const Eigen::Matrix4d& pose, const std::string& name);
    void removeObstacle(const std::string& name);

    // Discrete collision checking along trajectory
    bool checkPath(const Trajectory& traj, const std::string& robot_name, double safety_margin = 0.0);

    // Distance query between two named objects
    double computeDistance(const std::string& obj_a, const std::string& obj_b);

    // Update robot configuration for continuous collision detection
    void updateRobot(const std::string& name, const std::vector<double>& q);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace orolp
