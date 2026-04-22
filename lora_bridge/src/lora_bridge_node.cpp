#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <thread>
#include <atomic>
#include <cmath>

/* 与STM32端相同的协议定义 */
static constexpr uint8_t FRAME_HEAD       = 0xAA;
static constexpr uint8_t FRAME_TAIL       = 0xBB;
static constexpr uint8_t CMD_START_SLAM   = 0x01;
static constexpr uint8_t CMD_STOP_SLAM    = 0x02;
static constexpr uint8_t CMD_RESET        = 0x03;
static constexpr uint8_t CMD_QUERY_STATE  = 0x04;
static constexpr uint8_t CMD_STATE_RESP   = 0x10;

static uint8_t xor_check(const uint8_t *data, int len) {
    uint8_t v = 0;
    for (int i = 0; i < len; i++) v ^= data[i];
    return v;
}

class LoraBridgeNode : public rclcpp::Node {
public:
    LoraBridgeNode() : Node("lora_bridge_node"), slam_running_(false) {
        declare_parameter("serial_port", std::string("/dev/ttyS0"));
        declare_parameter("baudrate", 9600);

        auto port     = get_parameter("serial_port").as_string();
        auto baudrate = get_parameter("baudrate").as_int();

        fd_ = open_serial(port, baudrate);
        if (fd_ < 0)
            RCLCPP_ERROR(get_logger(), "Cannot open LoRa serial: %s", port.c_str());
        else
            RCLCPP_INFO(get_logger(), "LoRa bridge on %s @ %d baud", port.c_str(), (int)baudrate);

        // 订阅里程计，用于回传位姿
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                cur_x_   = msg->pose.pose.position.x;
                cur_y_   = msg->pose.pose.position.y;
            });

        // 发布启停指令供 Point-LIO 或 launch manager 订阅
        start_pub_ = create_publisher<std_msgs::msg::Bool>("/slam/start", 10);
        stop_pub_  = create_publisher<std_msgs::msg::Bool>("/slam/stop",  10);

        // 定期回传状态（1Hz）
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { send_state_response(); });

        // 串口接收线程
        recv_thread_ = std::thread([this]() { recv_loop(); });

        RCLCPP_INFO(get_logger(), "LoRa bridge node started");
    }

    ~LoraBridgeNode() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (fd_ >= 0) close(fd_);
    }

private:
    void recv_loop() {
        uint8_t buf[128];
        size_t  idx = 0;
        bool    in_frame = false;

        while (running_) {
            uint8_t byte;
            ssize_t n = read(fd_, &byte, 1);
            if (n <= 0) { usleep(1000); continue; }

            if (!in_frame) {
                if (byte == FRAME_HEAD) { idx = 0; buf[idx++] = byte; in_frame = true; }
                continue;
            }

            buf[idx++] = byte;
            if (idx >= sizeof(buf)) { in_frame = false; continue; }

            if (byte == FRAME_TAIL && idx >= 5) {
                // buf: HEAD | CMD | LEN | DATA... | XOR | TAIL
                uint8_t cmd = buf[1];
                uint8_t len = buf[2];
                uint8_t xor_calc = xor_check(buf + 1, len + 2); // CMD+LEN+DATA
                uint8_t xor_recv = buf[3 + len];
                if (xor_calc == xor_recv)
                    dispatch_cmd(cmd);
                in_frame = false;
            }
        }
    }

    void dispatch_cmd(uint8_t cmd) {
        switch (cmd) {
            case CMD_START_SLAM: {
                slam_running_ = true;
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                start_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "[LoRa] CMD: START SLAM");
                break;
            }
            case CMD_STOP_SLAM: {
                slam_running_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                stop_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "[LoRa] CMD: STOP SLAM");
                break;
            }
            case CMD_RESET:
                slam_running_ = false;
                cur_x_ = cur_y_ = fps_ = 0.0;
                RCLCPP_INFO(get_logger(), "[LoRa] CMD: RESET");
                break;
            case CMD_QUERY_STATE:
                send_state_response();
                break;
            default:
                RCLCPP_WARN(get_logger(), "[LoRa] Unknown CMD: 0x%02X", cmd);
        }
    }

    void send_state_response() {
        if (fd_ < 0) return;
        // 载荷: fps(4B) + x(4B) + y(4B) = 12字节
        constexpr uint8_t DATA_LEN = 12;
        uint8_t frame[5 + DATA_LEN + 2];
        float fps_f = static_cast<float>(fps_);
        float x_f   = static_cast<float>(cur_x_);
        float y_f   = static_cast<float>(cur_y_);

        frame[0] = FRAME_HEAD;
        frame[1] = CMD_STATE_RESP;
        frame[2] = DATA_LEN;
        memcpy(frame + 3 + 0, &fps_f, 4);
        memcpy(frame + 3 + 4, &x_f,   4);
        memcpy(frame + 3 + 8, &y_f,   4);
        frame[3 + DATA_LEN] = xor_check(frame + 1, DATA_LEN + 2);
        frame[4 + DATA_LEN] = FRAME_TAIL;

        write(fd_, frame, sizeof(frame));
    }

    int open_serial(const std::string &port, int baudrate) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) return -1;
        fcntl(fd, F_SETFL, 0);
        struct termios tio{};
        cfmakeraw(&tio);
        speed_t spd;
        switch (baudrate) {
            case 9600:  spd = B9600;  break;
            case 115200:spd = B115200;break;
            default:    spd = B9600;  break;
        }
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);
        tio.c_cflag |= (CLOCAL | CREAD | CS8);
        tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tio);
        return fd;
    }

    int fd_{-1};
    std::atomic<bool> running_{true};
    std::atomic<bool> slam_running_{false};
    std::thread recv_thread_;

    double cur_x_{0}, cur_y_{0}, fps_{0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
