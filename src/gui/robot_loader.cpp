#include "robot_loader.h"
#include "robot.h"
#include <QOpenGLContext>
#include <cmath>
#include <iostream>

namespace orolp_gui {

RobotLoader::RobotLoader() = default;
RobotLoader::~RobotLoader() = default;

bool RobotLoader::loadURDF(const std::string& urdf_path) {
    try {
        robot_model_ = std::make_unique<orolp::RobotModel>(urdf_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load URDF: " << e.what() << std::endl;
        return false;
    }

    int n_dof = robot_model_->dof();
    std::cout << "Loaded robot: " << robot_model_->name() << " with " << n_dof << " DOF" << std::endl;

    // Create simplified geometries for each joint/link
    // For demo purposes, we create a cylinder for each revolute joint
    auto jnames = robot_model_->joint_names();
    auto limits = robot_model_->limits();

    for (int i = 0; i < n_dof; ++i) {
        LinkGeometry link;
        link.name = jnames[i];
        link.joint_idx = i;
        // Alternate colors for visual distinction
        float hue = static_cast<float>(i) / std::max(1, n_dof);
        link.color = QVector3D(
            0.5f + 0.5f * std::sin(hue * 6.28f),
            0.5f + 0.5f * std::sin(hue * 6.28f + 2.09f),
            0.5f + 0.5f * std::sin(hue * 6.28f + 4.18f)
        );
        // Default cylinder: radius 30mm, length 150mm (scaled for visualization)
        generateCylinder(link, 0.03f, 0.15f);
        links_.push_back(std::move(link));
    }

    // Add base link
    {
        LinkGeometry base;
        base.name = "base_link";
        base.joint_idx = -1;
        base.color = QVector3D(0.2f, 0.2f, 0.25f);  // Dark base
        generateBox(base, 0.15f, 0.05f, 0.15f);
        links_.push_back(std::move(base));
    }

    global_transforms_.resize(links_.size(), QMatrix4x4());
    initialized_ = true;
    return true;
}

void RobotLoader::setJointValues(const std::vector<double>& q) {
    if (!robot_model_ || q.size() != static_cast<size_t>(robot_model_->dof())) return;

    // Compute FK for each joint frame
    auto T_ee = robot_model_->fk(q);  // End-effector

    // For demo: distribute joints along a simple chain
    // In full implementation, parse URDF tree structure
    QMatrix4x4 base;
    base.translate(0, 0, 0);
    global_transforms_[links_.size() - 1] = base;  // Base stays fixed

    float accum_z = 0.0f;
    for (size_t i = 0; i < links_.size() - 1; ++i) {  // Exclude base
        QMatrix4x4 t;
        // Simple serial chain visualization: each joint rotates and translates
        accum_z += 0.15f;  // Link length offset
        t.translate(0, 0, accum_z);
        t.rotate(static_cast<float>(q[i] * 180.0 / M_PI), 0, 0, 1);  // Z-axis rotation
        global_transforms_[i] = t;
    }
}

void RobotLoader::render(QOpenGLFunctions* gl) {
    for (size_t i = 0; i < links_.size(); ++i) {
        const auto& link = links_[i];
        if (!link.vao.isCreated()) continue;

        link.vao.bind();
        gl->glDrawElements(GL_TRIANGLES, link.index_count, GL_UNSIGNED_INT, nullptr);
        link.vao.release();
    }
}

std::vector<std::string> RobotLoader::jointNames() const {
    if (!robot_model_) return {};
    return robot_model_->joint_names();
}

std::vector<double> RobotLoader::jointLimitsMin() const {
    if (!robot_model_) return {};
    return robot_model_->limits().min;
}

std::vector<double> RobotLoader::jointLimitsMax() const {
    if (!robot_model_) return {};
    return robot_model_->limits().max;
}

int RobotLoader::dof() const {
    if (!robot_model_) return 0;
    return robot_model_->dof();
}

void RobotLoader::generateCylinder(LinkGeometry& link, float radius, float length, int segments) {
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // Top and bottom center vertices
    int topCenter = segments * 2;
    int bottomCenter = segments * 2 + 1;

    for (int i = 0; i < segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float c = std::cos(theta);
        float s = std::sin(theta);

        // Top circle
        vertices.push_back(radius * c);
        vertices.push_back(radius * s);
        vertices.push_back(length / 2);
        vertices.push_back(c);  // normal x
        vertices.push_back(s);  // normal y
        vertices.push_back(0);  // normal z

        // Bottom circle
        vertices.push_back(radius * c);
        vertices.push_back(radius * s);
        vertices.push_back(-length / 2);
        vertices.push_back(c);
        vertices.push_back(s);
        vertices.push_back(0);
    }

    // Center vertices for caps
    vertices.push_back(0); vertices.push_back(0); vertices.push_back(length / 2);
    vertices.push_back(0); vertices.push_back(0); vertices.push_back(1);
    vertices.push_back(0); vertices.push_back(0); vertices.push_back(-length / 2);
    vertices.push_back(0); vertices.push_back(0); vertices.push_back(-1);

    // Side faces
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int top_i = i * 2;
        int top_next = next * 2;
        int bot_i = i * 2 + 1;
        int bot_next = next * 2 + 1;

        indices.push_back(top_i); indices.push_back(bot_i); indices.push_back(top_next);
        indices.push_back(top_next); indices.push_back(bot_i); indices.push_back(bot_next);
    }

    // Top cap
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        indices.push_back(topCenter);
        indices.push_back(i * 2);
        indices.push_back(next * 2);
    }

    // Bottom cap
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        indices.push_back(bottomCenter);
        indices.push_back(next * 2 + 1);
        indices.push_back(i * 2 + 1);
    }

    link.index_count = static_cast<int>(indices.size());

    link.vbo.create();
    link.vbo.bind();
    link.vbo.allocate(vertices.data(), vertices.size() * sizeof(float));

    link.ibo.create();
    link.ibo.bind();
    link.ibo.allocate(indices.data(), indices.size() * sizeof(unsigned int));

    link.vao.create();
    link.vao.bind();
    link.vbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    link.ibo.bind();
    link.vao.release();
}

void RobotLoader::generateBox(LinkGeometry& link, float w, float h, float d) {
    float x = w / 2, y = h / 2, z = d / 2;
    float vertices[] = {
        // Front face
        -x, -y,  z,  0, 0, 1,
         x, -y,  z,  0, 0, 1,
         x,  y,  z,  0, 0, 1,
        -x,  y,  z,  0, 0, 1,
        // Back face
        -x, -y, -z,  0, 0, -1,
        -x,  y, -z,  0, 0, -1,
         x,  y, -z,  0, 0, -1,
         x, -y, -z,  0, 0, -1,
        // Top face
        -x,  y, -z,  0, 1, 0,
        -x,  y,  z,  0, 1, 0,
         x,  y,  z,  0, 1, 0,
         x,  y, -z,  0, 1, 0,
        // Bottom face
        -x, -y, -z,  0, -1, 0,
         x, -y, -z,  0, -1, 0,
         x, -y,  z,  0, -1, 0,
        -x, -y,  z,  0, -1, 0,
        // Right face
         x, -y, -z,  1, 0, 0,
         x,  y, -z,  1, 0, 0,
         x,  y,  z,  1, 0, 0,
         x, -y,  z,  1, 0, 0,
        // Left face
        -x, -y, -z,  -1, 0, 0,
        -x, -y,  z,  -1, 0, 0,
        -x,  y,  z,  -1, 0, 0,
        -x,  y, -z,  -1, 0, 0,
    };

    unsigned int indices[] = {
        0, 1, 2,  0, 2, 3,       // front
        4, 5, 6,  4, 6, 7,       // back
        8, 9, 10, 8, 10, 11,     // top
        12, 13, 14, 12, 14, 15,  // bottom
        16, 17, 18, 16, 18, 19,  // right
        20, 21, 22, 20, 22, 23   // left
    };

    link.index_count = 36;

    link.vbo.create();
    link.vbo.bind();
    link.vbo.allocate(vertices, sizeof(vertices));

    link.ibo.create();
    link.ibo.bind();
    link.ibo.allocate(indices, sizeof(indices));

    link.vao.create();
    link.vao.bind();
    link.vbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    link.ibo.bind();
    link.vao.release();
}

void RobotLoader::generateSphere(LinkGeometry& link, float radius, int segments) {
    // Simplified: generate a low-poly sphere or use icosphere
    // For demo, just use a small box as placeholder
    generateBox(link, radius * 2, radius * 2, radius * 2);
}

} // namespace orolp_gui
