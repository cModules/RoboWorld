#pragma once

#include <QMainWindow>
#include <memory>

namespace orolp_gui {

class Viewport3D;
class JointPanel;
class RobotLoader;

/**
 * @brief Main application window.
 * Layout: 3D viewport (center) + joint panel (right) + toolbar (top).
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    bool loadRobot(const std::string& urdf_path);

private:
    void setupUI();
    void setupToolbar();
    void onJointChanged(const std::vector<double>& q);

    Viewport3D* viewport_ = nullptr;
    JointPanel* joint_panel_ = nullptr;
    std::shared_ptr<RobotLoader> robot_;
};

} // namespace orolp_gui
