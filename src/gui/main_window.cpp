#include "main_window.h"
#include "viewport_3d.h"
#include "joint_panel.h"
#include "robot_loader.h"
#include <QSplitter>
#include <QToolBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QDebug>

namespace orolp_gui {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUI();
    setupToolbar();
    setWindowTitle("OpenRoboOLP 3D Viewer v0.1.0");
    resize(1200, 800);
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUI() {
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    setCentralWidget(splitter);

    // 3D Viewport
    viewport_ = new Viewport3D(this);
    splitter->addWidget(viewport_);

    // Joint Panel
    joint_panel_ = new JointPanel(this);
    joint_panel_->setMaximumWidth(350);
    splitter->addWidget(joint_panel_);

    splitter->setSizes({900, 300});

    // Connect joint changes to viewport
    connect(joint_panel_, &JointPanel::jointValuesChanged,
            this, &MainWindow::onJointChanged);
}

void MainWindow::setupToolbar() {
    auto* toolbar = addToolBar("Main");

    auto* load_btn = new QPushButton("Load URDF", this);
    load_btn->setStyleSheet("QPushButton { padding: 6px 12px; }");
    toolbar->addWidget(load_btn);

    auto* reset_cam_btn = new QPushButton("Reset Camera", this);
    toolbar->addWidget(reset_cam_btn);

    toolbar->addSeparator();

    auto* info_label = new QLabel(" |  Left-drag: Orbit  |  Wheel: Zoom  |  Right: Pan", this);
    toolbar->addWidget(info_label);

    connect(load_btn, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getOpenFileName(this,
            "Load Robot URDF", "", "URDF Files (*.urdf);;All Files (*)");
        if (!path.isEmpty()) {
            loadRobot(path.toStdString());
        }
    });

    connect(reset_cam_btn, &QPushButton::clicked, this, [this]() {
        if (viewport_) viewport_->resetCamera();
    });
}

bool MainWindow::loadRobot(const std::string& urdf_path) {
    robot_ = std::make_shared<RobotLoader>();
    if (!robot_->loadURDF(urdf_path)) {
        QMessageBox::critical(this, "Error",
            QString("Failed to load URDF:
%1").arg(QString::fromStdString(urdf_path)));
        return false;
    }

    viewport_->setRobot(robot_);
    joint_panel_->setRobot(robot_);

    // Set initial home pose
    if (robot_->dof() >= 6) {
        std::vector<double> home = {0, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0};
        joint_panel_->jointValues();  // trigger update
        // Set slider values to home
        // (simplified: emit signal after panel setup)
    }

    setWindowTitle(QString("OpenRoboOLP - %1")
        .arg(QString::fromStdString(urdf_path)));
    return true;
}

void MainWindow::onJointChanged(const std::vector<double>& q) {
    if (viewport_) {
        viewport_->setJointValues(q);
    }
}

} // namespace orolp_gui
