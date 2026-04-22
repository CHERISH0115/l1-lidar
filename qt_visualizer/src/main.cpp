#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "main_window.hpp"
#include "cloud_widget.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    app.setApplicationName("SLAM Visualizer");

    qRegisterMetaType<std::vector<Point3D>>("std::vector<Point3D>");

    MainWindow win(argc, argv);
    win.show();
    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
