#pragma once
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include "cloud_widget.hpp"

// 在独立线程中运行 rclcpp::spin，通过信号将数据传递给 Qt 主线程
class RosWorker : public QObject {
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);
    ~RosWorker();

public slots:
    void start_ros(int argc, char **argv);
    void stop_ros();

signals:
    void cloud_received(std::vector<Point3D> pts);
    void pose_received(float x, float y, float yaw);
    void status_received(QString status);

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};
