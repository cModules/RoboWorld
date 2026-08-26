#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <vector>
#include <string>
#include <memory>

namespace orolp_gui {

class RobotLoader;

/**
 * @brief Side panel with sliders for each robot joint.
 * Emits signals when joint values change for real-time viewport update.
 */
class JointPanel : public QWidget {
    Q_OBJECT

public:
    explicit JointPanel(QWidget* parent = nullptr);
    ~JointPanel() override;

    void setRobot(std::shared_ptr<RobotLoader> robot);
    std::vector<double> jointValues() const;

signals:
    void jointValuesChanged(const std::vector<double>& q);

private:
    void createSliders();
    void onSliderChanged(int value);

    std::shared_ptr<RobotLoader> robot_;
    std::vector<QSlider*> sliders_;
    std::vector<QLabel*> labels_;
    std::vector<double> limits_min_;
    std::vector<double> limits_max_;
};

} // namespace orolp_gui
