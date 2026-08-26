#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "robot.h"
#include "motion.h"
#include "collision.h"

namespace py = pybind11;
using namespace orolp;

PYBIND11_MODULE(orolp_core, m) {
    m.doc() = "OpenRoboOLP Core Engine - Python Bindings";

    // JointLimits
    py::class_<JointLimits>(m, "JointLimits")
        .def_readwrite("min", &JointLimits::min)
        .def_readwrite("max", &JointLimits::max)
        .def_readwrite("vel_max", &JointLimits::vel_max)
        .def_readwrite("acc_max", &JointLimits::acc_max);

    // RobotModel
    py::class_<RobotModel>(m, "RobotModel")
        .def(py::init<const std::string&>(), py::arg("urdf_path"))
        .def("fk", &RobotModel::fk, py::arg("q"),
             "Forward kinematics: joint angles -> 4x4 SE(3) pose")
        .def("ik", &RobotModel::ik,
             py::arg("T_target"),
             py::arg("q_seed") = std::vector<double>{},
             py::arg("max_solutions") = 8,
             "Inverse kinematics: SE(3) pose -> list of joint solutions")
        .def("jacobian", &RobotModel::jacobian,
             py::arg("q"), py::arg("frame") = "",
             "Geometric Jacobian at specified frame")
        .def("dof", &RobotModel::dof)
        .def("limits", &RobotModel::limits)
        .def("name", &RobotModel::name)
        .def("joint_names", &RobotModel::joint_names)
        .def("setToolFrame", &RobotModel::setToolFrame)
        .def("setBaseFrame", &RobotModel::setBaseFrame)
        .def("toolFrame", &RobotModel::toolFrame)
        .def("baseFrame", &RobotModel::baseFrame);

    // MoveType
    py::enum_<MoveType>(m, "MoveType")
        .value("JOINT", MoveType::JOINT)
        .value("LINEAR", MoveType::LINEAR)
        .value("CIRCULAR", MoveType::CIRCULAR)
        .value("SPLINE", MoveType::SPLINE);

    // Waypoint
    py::class_<Waypoint>(m, "Waypoint")
        .def(py::init<>())
        .def_readwrite("type", &Waypoint::type)
        .def_readwrite("target", &Waypoint::target)
        .def_readwrite("speed", &Waypoint::speed)
        .def_readwrite("blend_radius", &Waypoint::blend_radius)
        .def_readwrite("frame_id", &Waypoint::frame_id);

    // Trajectory::Sample
    py::class_<Trajectory::Sample>(m, "TrajectorySample")
        .def_readwrite("t", &Trajectory::Sample::t)
        .def_readwrite("q", &Trajectory::Sample::q)
        .def_readwrite("dq", &Trajectory::Sample::dq)
        .def_readwrite("ddq", &Trajectory::Sample::ddq);

    // Trajectory
    py::class_<Trajectory>(m, "Trajectory")
        .def_readwrite("samples", &Trajectory::samples)
        .def("duration", &Trajectory::duration);

    // MotionPlanner
    py::class_<MotionPlanner>(m, "MotionPlanner")
        .def_static("planJointMove", &MotionPlanner::planJointMove,
                    py::arg("robot"), py::arg("start"), py::arg("end"),
                    py::arg("time_step") = 0.004)
        .def_static("planLinearMove", &MotionPlanner::planLinearMove,
                    py::arg("robot"), py::arg("start"), py::arg("end"),
                    py::arg("time_step") = 0.004)
        .def_static("optimizeTime", &MotionPlanner::optimizeTime,
                    py::arg("path"), py::arg("limits"));

    // CollisionWorld
    py::class_<CollisionWorld>(m, "CollisionWorld")
        .def(py::init<>())
        .def("addRobot", &CollisionWorld::addRobot)
        .def("addObstacle", &CollisionWorld::addObstacle)
        .def("removeObstacle", &CollisionWorld::removeObstacle)
        .def("checkPath", &CollisionWorld::checkPath,
             py::arg("traj"), py::arg("robot_name"), py::arg("safety_margin") = 0.0)
        .def("computeDistance", &CollisionWorld::computeDistance)
        .def("updateRobot", &CollisionWorld::updateRobot);

    // Mesh (for obstacles)
    py::class_<Mesh>(m, "Mesh")
        .def(py::init<>())
        .def_readwrite("vertices", &Mesh::vertices)
        .def_readwrite("triangles", &Mesh::triangles);
}
