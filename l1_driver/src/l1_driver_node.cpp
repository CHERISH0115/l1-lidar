#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "l1_driver/l1_parser.hpp"

#include <thread>
#include <memory>
#include <string>

class L1DriverNode : public rclcpp::Node {
public:
    L1DriverNode() : Node("l1_driver_node") {
        declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        declare_parameter("baudrate", 2000000);
        declare_parameter("lidar_frame", std::string("lidar_link"));
        declare_parameter("imu_frame",   std::string("imu_link"));

        auto port     = get_parameter("serial_port").as_string();
        auto baudrate = get_parameter("baudrate").as_int();

        lidar_frame_ = get_parameter("lidar_frame").as_string();
        imu_frame_   = get_parameter("imu_frame").as_string();

        pc_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        parser_ = std::make_unique<L1Parser>(port, baudrate);
        parser_->set_pointcloud_callback(
            [this](const PointCloudFrame &f) { publish_pointcloud(f); });
        parser_->set_imu_callback(
            [this](const ImuFrame &f) { publish_imu(f); });

        spin_thread_ = std::thread([this] { parser_->spin(); });

        RCLCPP_INFO(get_logger(), "L1 driver node started on %s @ %d baud",
                    port.c_str(), (int)baudrate);
    }

    ~L1DriverNode() {
        parser_->stop();
        if (spin_thread_.joinable()) spin_thread_.join();
    }

private:
    void publish_pointcloud(const PointCloudFrame &frame) {
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp    = rclcpp::Time(static_cast<uint64_t>(frame.timestamp * 1e9));
        msg->header.frame_id = lidar_frame_;
        msg->height = 1;
        msg->width  = static_cast<uint32_t>(frame.points.size());
        msg->is_dense = true;

        sensor_msgs::PointCloud2Modifier mod(*msg);
        mod.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "time", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(frame.points.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(*msg, "intensity");
        sensor_msgs::PointCloud2Iterator<float> it_t(*msg, "time");

        double t0 = frame.points.empty() ? frame.timestamp : frame.points[0].timestamp;
        for (const auto &pt : frame.points) {
            *it_x = pt.x; ++it_x;
            *it_y = pt.y; ++it_y;
            *it_z = pt.z; ++it_z;
            *it_i = pt.intensity; ++it_i;
            *it_t = static_cast<float>(pt.timestamp - t0); ++it_t;
        }

        pc_pub_->publish(std::move(msg));
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

        // 协方差设为对角阵（噪声参数可从雷达手册获取）
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<L1DriverNode>());
    rclcpp::shutdown();
    return 0;
}
