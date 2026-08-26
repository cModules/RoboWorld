#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QTimer>
#include <memory>

namespace orolp_gui {

class ShaderManager;
class RobotLoader;

/**
 * @brief OpenGL 3D viewport for robot visualization.
 * Features: orbit camera, grid floor, robot rendering, joint animation.
 */
class Viewport3D : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit Viewport3D(QWidget* parent = nullptr);
    ~Viewport3D() override;

    void setRobot(std::shared_ptr<RobotLoader> robot);
    void setJointValues(const std::vector<double>& q);

    // Camera control
    void resetCamera();
    void setCameraDistance(float dist);

signals:
    void cameraChanged();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void renderGrid(float size, int divisions);
    void renderAxes(float length);
    void updateProjection();

    std::unique_ptr<ShaderManager> shader_;
    std::shared_ptr<RobotLoader> robot_;

    // Camera
    float camera_dist_ = 2.0f;
    float camera_azimuth_ = 45.0f;   // degrees
    float camera_elevation_ = 30.0f; // degrees
    QPoint last_mouse_pos_;
    bool dragging_ = false;

    QMatrix4x4 proj_matrix_;
    QMatrix4x4 view_matrix_;

    QTimer* anim_timer_ = nullptr;
    std::vector<double> current_q_;
    std::vector<double> target_q_;
};

} // namespace orolp_gui
