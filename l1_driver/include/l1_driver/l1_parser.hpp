#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <functional>

struct LidarPoint {
    float x, y, z;
    float intensity;
    double timestamp;
};

struct ImuFrame {
    double timestamp;
    float qw, qx, qy, qz;          // 四元数
    float gyro_x, gyro_y, gyro_z;  // rad/s
    float acc_x,  acc_y,  acc_z;   // m/s²
};

struct PointCloudFrame {
    double timestamp;
    std::vector<LidarPoint> points;
};

using PointCloudCallback = std::function<void(const PointCloudFrame &)>;
using ImuCallback        = std::function<void(const ImuFrame &)>;

class L1Parser {
public:
    explicit L1Parser(const std::string &port, int baudrate = 2000000);
    ~L1Parser();

    void set_pointcloud_callback(PointCloudCallback cb);
    void set_imu_callback(ImuCallback cb);

    void spin();
    void stop();

private:
    int  fd_{-1};
    bool running_{false};

    PointCloudCallback pc_cb_;
    ImuCallback        imu_cb_;

    std::vector<uint8_t> buf_;

    // 暂存辅助帧，等距离帧到达后配对转换 XYZ
    struct AuxFrame {
        bool   valid{false};
        uint16_t packet_id{0};
        float  com_horizontal_angle_start;
        float  com_horizontal_angle_step;
        float  sys_vertical_angle_start;
        float  sys_vertical_angle_span;
        float  b_axis_dist;
        float  theta_angle;
        float  ksi_angle;
        uint8_t reflect_data[120];
    } aux_;

    void dispatch(uint32_t msgid, const uint8_t *payload, uint8_t len, double ts);
    void handle_imu(const uint8_t *p, double ts);
    void handle_aux(const uint8_t *p);
    void handle_dist(const uint8_t *p, double ts);

    int open_serial(const std::string &port, int baudrate);
};
