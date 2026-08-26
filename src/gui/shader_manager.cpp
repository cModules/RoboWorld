#include "shader_manager.h"
#include <QOpenGLShader>

namespace orolp_gui {

ShaderManager::ShaderManager() = default;
ShaderManager::~ShaderManager() = default;

bool ShaderManager::initialize() {
    program_ = std::make_unique<QOpenGLShaderProgram>();

    // Vertex shader
    const char* vs_src = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;

        uniform mat4 uMV;
        uniform mat4 uProj;
        uniform mat3 uNormal;

        out vec3 vNormal;
        out vec3 vPos;

        void main() {
            vec4 mvPos = uMV * vec4(aPos, 1.0);
            vPos = mvPos.xyz;
            vNormal = uNormal * aNormal;
            gl_Position = uProj * mvPos;
        }
    )";

    // Fragment shader
    const char* fs_src = R"(
        #version 330 core
        in vec3 vNormal;
        in vec3 vPos;

        uniform vec3 uColor;
        uniform vec3 uLightPos;

        out vec4 FragColor;

        void main() {
            vec3 normal = normalize(vNormal);
            vec3 lightDir = normalize(uLightPos - vPos);
            float diff = max(dot(normal, lightDir), 0.0);

            vec3 ambient = 0.3 * uColor;
            vec3 diffuse = 0.7 * diff * uColor;
            vec3 specular = 0.2 * pow(max(dot(normalize(-vPos), reflect(-lightDir, normal)), 0.0), 32.0) * vec3(1.0);

            FragColor = vec4(ambient + diffuse + specular, 1.0);
        }
    )";

    if (!program_->addShaderFromSourceCode(QOpenGLShader::Vertex, vs_src)) {
        qDebug() << "Vertex shader error:" << program_->log();
        return false;
    }
    if (!program_->addShaderFromSourceCode(QOpenGLShader::Fragment, fs_src)) {
        qDebug() << "Fragment shader error:" << program_->log();
        return false;
    }
    if (!program_->link()) {
        qDebug() << "Shader link error:" << program_->log();
        return false;
    }

    u_mv_ = program_->uniformLocation("uMV");
    u_proj_ = program_->uniformLocation("uProj");
    u_normal_ = program_->uniformLocation("uNormal");
    u_color_ = program_->uniformLocation("uColor");
    u_light_pos_ = program_->uniformLocation("uLightPos");

    initialized_ = true;
    return true;
}

void ShaderManager::bind() {
    if (program_) program_->bind();
}

void ShaderManager::release() {
    if (program_) program_->release();
}

void ShaderManager::setModelViewMatrix(const QMatrix4x4& mv) {
    if (u_mv_ >= 0) program_->setUniformValue(u_mv_, mv);
}

void ShaderManager::setProjectionMatrix(const QMatrix4x4& proj) {
    if (u_proj_ >= 0) program_->setUniformValue(u_proj_, proj);
}

void ShaderManager::setNormalMatrix(const QMatrix3x3& normal) {
    if (u_normal_ >= 0) program_->setUniformValue(u_normal_, normal);
}

void ShaderManager::setColor(const QVector3D& color) {
    if (u_color_ >= 0) program_->setUniformValue(u_color_, color);
}

void ShaderManager::setLightPos(const QVector3D& light) {
    if (u_light_pos_ >= 0) program_->setUniformValue(u_light_pos_, light);
}

} // namespace orolp_gui
