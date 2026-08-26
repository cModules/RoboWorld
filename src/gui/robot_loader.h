#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <vector>
#include <string>
#include <memory>
#include <map>

// Forward declarations from core
namespace orolp { class RobotModel; }

namespace orolp_gui {

/**
 * @brief Simplified robot visual representation.
 * Generates primitive geometries (cylinders, boxes) for each link
 * based on URDF joint origins and axes. Does not require external mesh files.
 */
struct LinkGeometry {
    QOpenGLBuffer vbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer ibo{QOpenGLBuffer::IndexBuffer};
    QOpenGLVertexArrayObject vao;
    int index_count = 0;
    QMatrix4x4 local_transform;  // Joint origin from URDF
    QVector3D color{0.7f, 0.7f, 0.75f};  // Default industrial gray
    std::string name;
    int joint_idx = -1;  // Index in q vector, -1 for fixed/base
    QVector3D joint_axis{0, 0, 1};  // Rotation axis for revolute joints
};

class RobotLoader {
public:
    RobotLoader();
    ~RobotLoader();

    // Load robot from URDF path (uses core::RobotModel for kinematics)
    bool loadURDF(const std::string& urdf_path);

    // Update joint values and compute global transforms
    void setJointValues(const std::vector<double>& q);

    // Render all links
    void render(QOpenGLFunctions* gl);

    // Access
    const std::vector<LinkGeometry>& links() const { return links_; }
    std::vector<std::string> jointNames() const;
    std::vector<double> jointLimitsMin() const;
    std::vector<double> jointLimitsMax() const;
    int dof() const;

private:
    void generateCylinder(LinkGeometry& link, float radius, float length, int segments = 32);
    void generateBox(LinkGeometry& link, float w, float h, float d);
    void generateSphere(LinkGeometry& link, float radius, int segments = 16);

    std::vector<LinkGeometry> links_;
    std::unique_ptr<orolp::RobotModel> robot_model_;
    std::vector<QMatrix4x4> global_transforms_;
    bool initialized_ = false;
};

} // namespace orolp_gui
