#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <memory>

namespace orolp_gui {

/**
 * @brief Manages OpenGL shaders for robot rendering.
 * Provides Phong shading with configurable colors.
 */
class ShaderManager {
public:
    ShaderManager();
    ~ShaderManager();

    bool initialize();
    void bind();
    void release();

    void setModelViewMatrix(const QMatrix4x4& mv);
    void setProjectionMatrix(const QMatrix4x4& proj);
    void setNormalMatrix(const QMatrix3x3& normal);
    void setColor(const QVector3D& color);
    void setLightPos(const QVector3D& light);

    QOpenGLShaderProgram* program() { return program_.get(); }

private:
    std::unique_ptr<QOpenGLShaderProgram> program_;
    bool initialized_ = false;

    // Uniform locations
    int u_mv_ = -1;
    int u_proj_ = -1;
    int u_normal_ = -1;
    int u_color_ = -1;
    int u_light_pos_ = -1;
};

} // namespace orolp_gui
