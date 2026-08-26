#include "viewport_3d.h"
#include "shader_manager.h"
#include "robot_loader.h"
#include <QMouseEvent>
#include <cmath>

namespace orolp_gui {

Viewport3D::Viewport3D(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    anim_timer_ = new QTimer(this);
    connect(anim_timer_, &QTimer::timeout, this, [this]() {
        // Smooth joint interpolation for animation
        if (robot_ && !current_q_.empty()) {
            for (size_t i = 0; i < current_q_.size(); ++i) {
                current_q_[i] += (target_q_[i] - current_q_[i]) * 0.1;
            }
            robot_->setJointValues(current_q_);
            update();
        }
    });
    anim_timer_->start(16);  // ~60 FPS
}

Viewport3D::~Viewport3D() = default;

void Viewport3D::setRobot(std::shared_ptr<RobotLoader> robot) {
    robot_ = robot;
    if (robot_) {
        current_q_.resize(robot_->dof(), 0.0);
        target_q_.resize(robot_->dof(), 0.0);
        robot_->setJointValues(current_q_);
    }
    update();
}

void Viewport3D::setJointValues(const std::vector<double>& q) {
    if (q.size() == target_q_.size()) {
        target_q_ = q;
    }
}

void Viewport3D::resetCamera() {
    camera_dist_ = 2.0f;
    camera_azimuth_ = 45.0f;
    camera_elevation_ = 30.0f;
    updateProjection();
    update();
}

void Viewport3D::setCameraDistance(float dist) {
    camera_dist_ = std::max(0.5f, dist);
    updateProjection();
    update();
}

void Viewport3D::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);  // Dark industrial background
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    shader_ = std::make_unique<ShaderManager>();
    if (!shader_->initialize()) {
        qDebug() << "Failed to initialize shaders";
    }

    updateProjection();
}

void Viewport3D::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!shader_ || !shader_->program()) return;

    shader_->bind();

    // Compute view matrix from orbit camera
    float az = camera_azimuth_ * M_PI / 180.0f;
    float el = camera_elevation_ * M_PI / 180.0f;
    float cx = camera_dist_ * std::cos(el) * std::cos(az);
    float cy = camera_dist_ * std::cos(el) * std::sin(az);
    float cz = camera_dist_ * std::sin(el);

    QMatrix4x4 view;
    view.lookAt(QVector3D(cx, cy, cz), QVector3D(0, 0, 0.5f), QVector3D(0, 0, 1));
    view_matrix_ = view;

    shader_->setProjectionMatrix(proj_matrix_);
    shader_->setLightPos(QVector3D(5, 5, 10));

    // Render grid
    shader_->setColor(QVector3D(0.3f, 0.3f, 0.35f));
    renderGrid(2.0f, 20);

    // Render axes
    renderAxes(0.3f);

    // Render robot
    if (robot_) {
        const auto& links = robot_->links();
        for (const auto& link : links) {
            if (!link.vao.isCreated()) continue;

            QMatrix4x4 mv = view_matrix_ * link.local_transform;
            QMatrix3x3 normal = mv.normalMatrix();

            shader_->setModelViewMatrix(mv);
            shader_->setNormalMatrix(normal);
            shader_->setColor(link.color);

            link.vao.bind();
            glDrawElements(GL_TRIANGLES, link.index_count, GL_UNSIGNED_INT, nullptr);
            link.vao.release();
        }
    }

    shader_->release();
}

void Viewport3D::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    updateProjection();
}

void Viewport3D::updateProjection() {
    float aspect = width() / std::max(1.0f, static_cast<float>(height()));
    proj_matrix_.perspective(45.0f, aspect, 0.01f, 100.0f);
}

void Viewport3D::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        last_mouse_pos_ = event->pos();
    }
}

void Viewport3D::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_) {
        QPoint delta = event->pos() - last_mouse_pos_;
        camera_azimuth_ -= delta.x() * 0.5f;
        camera_elevation_ += delta.y() * 0.5f;
        camera_elevation_ = std::clamp(camera_elevation_, -89.0f, 89.0f);
        last_mouse_pos_ = event->pos();
        update();
        emit cameraChanged();
    }
}

void Viewport3D::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = false;
    }
}

void Viewport3D::wheelEvent(QWheelEvent* event) {
    float delta = event->angleDelta().y() / 120.0f;
    camera_dist_ *= std::pow(0.9f, delta);
    camera_dist_ = std::clamp(camera_dist_, 0.3f, 10.0f);
    update();
}

void Viewport3D::renderGrid(float size, int divisions) {
    float step = size / divisions;
    float half = size / 2;

    // Use a simple line rendering approach
    // For proper grid, we'd need a line shader; for demo, use thin boxes
    // Simplified: skip detailed grid to avoid shader complexity
    QMatrix4x4 mv = view_matrix_;
    mv.translate(0, 0, 0);
    shader_->setModelViewMatrix(mv);
    shader_->setNormalMatrix(mv.normalMatrix());
}

void Viewport3D::renderAxes(float length) {
    // Simplified axes rendering
    QMatrix4x4 mv = view_matrix_;
    shader_->setModelViewMatrix(mv);
    shader_->setNormalMatrix(mv.normalMatrix());
}

} // namespace orolp_gui
