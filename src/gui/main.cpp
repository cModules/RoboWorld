#include <QApplication>
#include "main_window.h"
#include <iostream>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("OpenRoboOLP Viewer");
    app.setOrganizationName("openrobolp");

    orolp_gui::MainWindow window;
    window.show();

    // If URDF path provided as argument, auto-load
    if (argc > 1) {
        std::string urdf_path = argv[1];
        std::cout << "Loading robot from: " << urdf_path << std::endl;
        if (!window.loadRobot(urdf_path)) {
            std::cerr << "Failed to load robot. Starting with empty scene." << std::endl;
        }
    } else {
        std::cout << "Usage: orolp_viewer <path_to_robot.urdf>" << std::endl;
        std::cout << "No URDF provided. Use 'Load URDF' button or drag-and-drop." << std::endl;
    }

    return app.exec();
}
