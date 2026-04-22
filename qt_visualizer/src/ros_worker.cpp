#include "ros_worker.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

RosWorker::RosWorker(QObject *parent) : QObject(parent) {}

RosWorker::~RosWorker() { stop_ros(); }

void RosWorker::start_ros(int argc, char **argv) {
    node_ = std::make_shared<rclcpp::Node>("qt_visualizer_node");

    pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scan", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            on_pointcloud(msg);
        });

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            on_odometry(msg);
        });

    rclcpp::spin(node_);
}

void RosWorker::stop_ros() {
    rclcpp::shutdown();
}

void RosWorker::on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<Point3D> pts;
    pts.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
        if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
        pts.push_back({*it_x, *it_y, *it_z, 0.0f});
    }

    emit cloud_received(std::move(pts));
}

void RosWorker::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    float x = static_cast<float>(msg->pose.pose.position.x);
    float y = static_cast<float>(msg->pose.pose.position.y);

    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    emit pose_received(x, y, static_cast<float>(yaw));

    QString s = QString("X: %1 m  Y: %2 m  Yaw: %3°")
        .arg(x, 0, 'f', 2)
        .arg(y, 0, 'f', 2)
        .arg(yaw * 180.0 / M_PI, 0, 'f', 1);
    emit status_received(s);
}
