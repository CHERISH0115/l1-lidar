#include "ros_worker.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

RosWorker::RosWorker(QObject *parent) : QObject(parent) {}
RosWorker::~RosWorker() { stop_ros(); }

void RosWorker::start_ros(int argc, char **argv) {
    node_ = std::make_shared<rclcpp::Node>("qt_visualizer_node");

    // 订阅 FAST-LIO2 输出的全局注册点云
    pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10,
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

    // 检查 intensity 字段是否存在
    bool has_intensity = false;
    for (const auto &f : msg->fields)
        if (f.name == "intensity") { has_intensity = true; break; }

    if (has_intensity) {
        sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
            if (!std::isfinite(*it_x)) continue;
            pts.push_back({*it_x, *it_y, *it_z, *it_i});
        }
    } else {
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            if (!std::isfinite(*it_x)) continue;
            pts.push_back({*it_x, *it_y, *it_z, 0.0f});
        }
    }

    emit cloud_received(std::move(pts));
}

void RosWorker::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    float x = static_cast<float>(msg->pose.pose.position.x);
    float y = static_cast<float>(msg->pose.pose.position.y);
    float z = static_cast<float>(msg->pose.pose.position.z);

    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    emit pose_received(x, y, z, static_cast<float>(yaw));

    emit status_received(QString("X:%1 Y:%2 Z:%3\nYaw:%4°")
        .arg(x,0,'f',2).arg(y,0,'f',2).arg(z,0,'f',2)
        .arg(yaw * 180.0 / M_PI, 0,'f',1));
}
