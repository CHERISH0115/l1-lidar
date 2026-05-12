#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "l1_driver/l1_parser.hpp"

#include <thread>
#include <memory>
#include <string>
#include <vector>

class L1DriverNode : public rclcpp::Node {
public:
    L1DriverNode() : Node("l1_driver_node") {
        declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        declare_parameter("baudrate", 2000000);
        declare_parameter("lidar_frame", std::string("lidar_link"));
        declare_parameter("imu_frame",   std::string("imu_link"));
        declare_parameter("scan_period_sec", 0.1);   // 累积窗口，默认 100ms → 10 Hz
        declare_parameter("max_points_per_scan", 8192);

        auto port     = get_parameter("serial_port").as_string();
        auto baudrate = get_parameter("baudrate").as_int();
        scan_period_  = get_parameter("scan_period_sec").as_double();
        max_pts_      = static_cast<size_t>(get_parameter("max_points_per_scan").as_int());

        lidar_frame_ = get_parameter("lidar_frame").as_string();
        imu_frame_   = get_parameter("imu_frame").as_string();

        pc_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        acc_.reserve(max_pts_);

        parser_ = std::make_unique<L1Parser>(port, baudrate);
        parser_->set_pointcloud_callback(
            [this](const PointCloudFrame &f) { on_parser_points(f); });
        parser_->set_imu_callback(
            [this](const ImuFrame &f) { publish_imu(f); });

        spin_thread_ = std::thread([this] { parser_->spin(); });

        RCLCPP_INFO(get_logger(),
                    "L1 driver started on %s @ %d baud (scan_period=%.3fs)",
                    port.c_str(), (int)baudrate, scan_period_);
    }

    ~L1DriverNode() {
        parser_->stop();
        if (spin_thread_.joinable()) spin_thread_.join();
    }

private:
    // 累积 parser 的小帧到一个完整扫描周期再发布
    void on_parser_points(const PointCloudFrame &f) {
        if (f.points.empty()) return;

        if (acc_.empty()) {
            acc_start_ts_ = f.timestamp;
        }

        // 防止异常情况下无限累积
        if (acc_.size() + f.points.size() > max_pts_) {
            flush_scan();
            acc_start_ts_ = f.timestamp;
        }
        acc_.insert(acc_.end(), f.points.begin(), f.points.end());

        if (f.timestamp - acc_start_ts_ >= scan_period_) {
            flush_scan();
        }
    }

    void flush_scan() {
        if (acc_.empty()) return;

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp    = rclcpp::Time(static_cast<uint64_t>(acc_start_ts_ * 1e9));
        msg->header.frame_id = lidar_frame_;
        msg->height   = 1;
        msg->width    = static_cast<uint32_t>(acc_.size());
        msg->is_dense = true;

        sensor_msgs::PointCloud2Modifier mod(*msg);
        mod.setPointCloud2Fields(5,
            "x",         1, sensor_msgs::msg::PointField::FLOAT32,
            "y",         1, sensor_msgs::msg::PointField::FLOAT32,
            "z",         1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "time",      1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(acc_.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(*msg, "intensity");
        sensor_msgs::PointCloud2Iterator<float> it_t(*msg, "time");

        for (const auto &pt : acc_) {
            *it_x = pt.x;        ++it_x;
            *it_y = pt.y;        ++it_y;
            *it_z = pt.z;        ++it_z;
            *it_i = pt.intensity;++it_i;
            *it_t = static_cast<float>(pt.timestamp - acc_start_ts_); ++it_t;
        }

        pc_pub_->publish(std::move(msg));
        acc_.clear();
    }

    void publish_imu(const ImuFrame &f) {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp    = rclcpp::Time(static_cast<uint64_t>(f.timestamp * 1e9));
        msg->header.frame_id = imu_frame_;

        msg->linear_acceleration.x = f.acc_x;
        msg->linear_acceleration.y = f.acc_y;
        msg->linear_acceleration.z = f.acc_z;

        msg->angular_velocity.x = f.gyro_x;
        msg->angular_velocity.y = f.gyro_y;
        msg->angular_velocity.z = f.gyro_z;

        msg->linear_acceleration_covariance[0] = 0.01;
        msg->linear_acceleration_covariance[4] = 0.01;
        msg->linear_acceleration_covariance[8] = 0.01;
        msg->angular_velocity_covariance[0] = 0.001;
        msg->angular_velocity_covariance[4] = 0.001;
        msg->angular_velocity_covariance[8] = 0.001;
        msg->orientation.w = f.qw;
        msg->orientation.x = f.qx;
        msg->orientation.y = f.qy;
        msg->orientation.z = f.qz;
        msg->orientation_covariance[0] = 0.001;
        msg->orientation_covariance[4] = 0.001;
        msg->orientation_covariance[8] = 0.001;

        imu_pub_->publish(std::move(msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::unique_ptr<L1Parser> parser_;
    std::thread spin_thread_;

    std::string lidar_frame_;
    std::string imu_frame_;

    // 扫描帧累积缓冲（仅由 parser spin 线程访问，无需锁）
    std::vector<LidarPoint> acc_;
    double                  acc_start_ts_{0.0};
    double                  scan_period_{0.1};
    size_t                  max_pts_{8192};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<L1DriverNode>());
    rclcpp::shutdown();
    return 0;
}
