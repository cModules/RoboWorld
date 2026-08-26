#include "joint_panel.h"
#include "robot_loader.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <cmath>

namespace orolp_gui {

JointPanel::JointPanel(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setSpacing(10);
    layout->setContentsMargins(10, 10, 10, 10);

    // Title
    auto* title = new QLabel("<h2>Joint Control</h2>", this);
    title->setAlignment(Qt::AlignCenter);
    layout->addWidget(title);

    // Sliders container
    auto* sliders_box = new QGroupBox("Joint Values (degrees)", this);
    auto* sliders_layout = new QVBoxLayout(sliders_box);
    layout->addWidget(sliders_box);

    // Buttons
    auto* btn_layout = new QHBoxLayout();
    auto* zero_btn = new QPushButton("Zero All", this);
    auto* home_btn = new QPushButton("Home Pose", this);
    btn_layout->addWidget(zero_btn);
    btn_layout->addWidget(home_btn);
    layout->addLayout(btn_layout);

    connect(zero_btn, &QPushButton::clicked, this, [this]() {
        for (auto* s : sliders_) s->setValue(0);
    });
    connect(home_btn, &QPushButton::clicked, this, [this]() {
        if (sliders_.size() >= 6) {
            sliders_[0]->setValue(0);
            sliders_[1]->setValue(-90);
            sliders_[2]->setValue(90);
            sliders_[3]->setValue(-90);
            sliders_[4]->setValue(90);
            sliders_[5]->setValue(0);
        }
    });

    layout->addStretch();
}

JointPanel::~JointPanel() = default;

void JointPanel::setRobot(std::shared_ptr<RobotLoader> robot) {
    robot_ = robot;

    // Clear existing widgets from the group box
    // (simplified: just clear vectors, in real app would delete old widgets)
    sliders_.clear();
    labels_.clear();
    limits_min_.clear();
    limits_max_.clear();

    if (!robot_) return;

    auto names = robot_->jointNames();
    auto mins = robot_->jointLimitsMin();
    auto maxs = robot_->jointLimitsMax();
    int n = robot_->dof();

    // Find the QGroupBox and its layout
    auto* group = findChild<QGroupBox*>("Joint Values (degrees)");
    if (!group) return;

    // Clear old widgets
    QLayoutItem* child;
    while ((child = group->layout()->takeAt(0)) != nullptr) {
        if (child->widget()) delete child->widget();
        delete child;
    }

    for (int i = 0; i < n; ++i) {
        double min_deg = mins[i] * 180.0 / M_PI;
        double max_deg = maxs[i] * 180.0 / M_PI;
        limits_min_.push_back(min_deg);
        limits_max_.push_back(max_deg);

        auto* row = new QWidget(group);
        auto* row_layout = new QHBoxLayout(row);
        row_layout->setContentsMargins(0, 0, 0, 0);

        QString label_text = QString("J%1: %2").arg(i + 1).arg(QString::fromStdString(names[i]));
        auto* label = new QLabel(label_text, row);
        label->setMinimumWidth(100);
        row_layout->addWidget(label);

        auto* slider = new QSlider(Qt::Horizontal, row);
        slider->setRange(static_cast<int>(min_deg * 10), static_cast<int>(max_deg * 10));
        slider->setValue(0);
        slider->setProperty("joint_idx", i);
        row_layout->addWidget(slider, 1);

        auto* value_label = new QLabel("0.0°", row);
        value_label->setMinimumWidth(60);
        value_label->setAlignment(Qt::AlignRight);
        row_layout->addWidget(value_label);

        group->layout()->addWidget(row);
        sliders_.push_back(slider);
        labels_.push_back(value_label);

        connect(slider, &QSlider::valueChanged, this, [this, value_label](int val) {
            double deg = val / 10.0;
            value_label->setText(QString("%1°").arg(deg, 0, 'f', 1));
            emit jointValuesChanged(jointValues());
        });
    }
}

std::vector<double> JointPanel::jointValues() const {
    std::vector<double> q;
    for (size_t i = 0; i < sliders_.size(); ++i) {
        double deg = sliders_[i]->value() / 10.0;
        q.push_back(deg * M_PI / 180.0);
    }
    return q;
}

} // namespace orolp_gui
