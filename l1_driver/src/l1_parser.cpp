#include "l1_driver/l1_parser.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <chrono>

// ── MAVLink v2 帧格式 ─────────────────────────────────────────────
// STX(0xFD) | LEN | INCOMPAT | COMPAT | SEQ | SYSID | COMPID | MSGID[3] | PAYLOAD[LEN] | CRC[2]
// 帧头共 10 字节，帧尾 CRC 2 字节

static constexpr uint8_t  MAVLINK_STX      = 0xFD;
static constexpr uint32_t MSGID_DISTANCE   = 16;   // RET_LIDAR_DISTANCE_DATA_PACKET
static constexpr uint32_t MSGID_AUXILIARY  = 17;   // RET_LIDAR_AUXILIARY_DATA_PACKET
static constexpr uint32_t MSGID_IMU        = 19;   // RET_IMU_ATTITUDE_DATA_PACKET

// CRC_EXTRA（由 MAVLink 消息定义文件给出）
static constexpr uint8_t CRC_EXTRA_DISTANCE  = 74;
static constexpr uint8_t CRC_EXTRA_AUXILIARY = 99;
static constexpr uint8_t CRC_EXTRA_IMU       = 110;

// ── CRC-16/MCRF4XX ───────────────────────────────────────────────
static uint16_t crc_accum(uint16_t crc, uint8_t b) {
    uint8_t tmp = b ^ (crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

// 计算范围 buf[1..9+payload_len]（跳过 STX，不含末尾 CRC），再混入 CRC_EXTRA
static uint16_t mavlink_crc(const uint8_t *frame, uint8_t payload_len, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    // 从 buf[1] 开始：LEN INCOMPAT COMPAT SEQ SYSID COMPID MSGID[3] PAYLOAD[payload_len]
    for (size_t i = 1; i < (size_t)(10 + payload_len); i++)
        crc = crc_accum(crc, frame[i]);
    crc = crc_accum(crc, crc_extra);
    return crc;
}

// ── 读 float / uint16 小端 ───────────────────────────────────────
static inline float     lef(const uint8_t *p) { float v;     memcpy(&v, p, 4); return v; }
static inline uint16_t  leu16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t  leu32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

static double now_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

// ── 构造 / 析构 ──────────────────────────────────────────────────
L1Parser::L1Parser(const std::string &port, int baudrate) {
    fd_ = open_serial(port, baudrate);
    if (fd_ < 0)
        throw std::runtime_error("L1Parser: cannot open " + port);
    buf_.reserve(4096);
}

L1Parser::~L1Parser() { stop(); if (fd_ >= 0) close(fd_); }

void L1Parser::set_pointcloud_callback(PointCloudCallback cb) { pc_cb_ = cb; }
void L1Parser::set_imu_callback(ImuCallback cb)               { imu_cb_ = cb; }
void L1Parser::stop() { running_ = false; }

// ── 主循环 ───────────────────────────────────────────────────────
void L1Parser::spin() {
    running_ = true;
    uint8_t tmp[512];

    while (running_) {
        ssize_t n = read(fd_, tmp, sizeof(tmp));
        if (n <= 0) continue;
        buf_.insert(buf_.end(), tmp, tmp + n);

        while (buf_.size() >= 12) {  // 最小帧 = 10头 + 0载荷 + 2CRC
            // 搜 STX
            size_t start = buf_.size();
            for (size_t i = 0; i < buf_.size(); i++) {
                if (buf_[i] == MAVLINK_STX) { start = i; break; }
            }
            if (start > 0) buf_.erase(buf_.begin(), buf_.begin() + start);
            if (buf_.size() < 12) break;

            uint8_t  payload_len = buf_[1];
            size_t   total       = 10 + payload_len + 2;
            if (buf_.size() < total) break;

            // 取 msgid（3 字节小端，偏移 7-9）
            uint32_t msgid = (uint32_t)buf_[7] | ((uint32_t)buf_[8] << 8) | ((uint32_t)buf_[9] << 16);

            // 选 CRC_EXTRA
            uint8_t crc_extra = 0;
            bool known = true;
            if      (msgid == MSGID_IMU)       crc_extra = CRC_EXTRA_IMU;
            else if (msgid == MSGID_DISTANCE)  crc_extra = CRC_EXTRA_DISTANCE;
            else if (msgid == MSGID_AUXILIARY) crc_extra = CRC_EXTRA_AUXILIARY;
            else    known = false;

            if (known) {
                // 验证 CRC
                uint16_t expected = mavlink_crc(buf_.data(), payload_len, crc_extra);
                uint16_t actual   = leu16(buf_.data() + 10 + payload_len);
                if (expected == actual) {
                    dispatch(msgid, buf_.data() + 10, payload_len, now_sec());
                }
            }

            buf_.erase(buf_.begin(), buf_.begin() + total);
        }
    }
}

// ── 分发 ─────────────────────────────────────────────────────────
void L1Parser::dispatch(uint32_t msgid, const uint8_t *p, uint8_t len, double ts) {
    if (msgid == MSGID_IMU       && len >= 42) handle_imu(p, ts);
    if (msgid == MSGID_AUXILIARY && len >= 89) handle_aux(p);
    if (msgid == MSGID_DISTANCE  && len >= 6)  handle_dist(p, ts);
}

// ── IMU 帧（msgid=19, 42字节）────────────────────────────────────
// wire layout: quat[4×float] gyro[3×float] acc[3×float] packet_id[uint16]
void L1Parser::handle_imu(const uint8_t *p, double ts) {
    if (!imu_cb_) return;
    ImuFrame f;
    f.timestamp = ts;
    f.qw = lef(p +  0);  f.qx = lef(p +  4);
    f.qy = lef(p +  8);  f.qz = lef(p + 12);
    f.gyro_x = lef(p + 16); f.gyro_y = lef(p + 20); f.gyro_z = lef(p + 24);
    f.acc_x  = lef(p + 28); f.acc_y  = lef(p + 32); f.acc_z  = lef(p + 36);
    imu_cb_(f);
}

// ── 辅助帧（msgid=17, 209字节）──────────────────────────────────
void L1Parser::handle_aux(const uint8_t *p) {
    aux_.valid                     = true;
    aux_.packet_id                 = leu16(p + 84);
    aux_.com_horizontal_angle_start = lef(p + 20);
    aux_.com_horizontal_angle_step  = lef(p + 24);
    aux_.sys_vertical_angle_start   = lef(p + 28);
    aux_.sys_vertical_angle_span    = lef(p + 32);
    aux_.b_axis_dist                = lef(p + 72);
    aux_.theta_angle                = lef(p + 76);
    aux_.ksi_angle                  = lef(p + 80);
    memcpy(aux_.reflect_data, p + 89, 120);
}

// ── 距离帧（msgid=16, 246字节）──────────────────────────────────
// wire: packet_id[2] packet_cnt[2] payload_size[2] point_data[240]
void L1Parser::handle_dist(const uint8_t *p, double ts) {
    if (!pc_cb_) return;

    uint16_t pid = leu16(p);
    const uint8_t *point_data = p + 6;

    // 使用最近一次有效辅助帧进行 XYZ 转换（无需严格 packet_id 匹配）
    (void)pid;
    if (aux_.valid) {
        const float range_scale = 0.001f;
        const float z_bias      = 0.0445f;
        const int   N           = 120;

        float bias  = aux_.b_axis_dist / 1000.0f;
        float sin_t = sinf(aux_.theta_angle);
        float cos_t = cosf(aux_.theta_angle);
        float sin_k = sinf(aux_.ksi_angle);
        float cos_k = cosf(aux_.ksi_angle);

        float pitch = aux_.sys_vertical_angle_start  * (float)M_PI / 180.0f;
        float dpitch = aux_.sys_vertical_angle_span  * (float)M_PI / 180.0f;
        float yaw   = aux_.com_horizontal_angle_start * (float)M_PI / 180.0f;
        float dyaw  = aux_.com_horizontal_angle_step / N * (float)M_PI / 180.0f;

        PointCloudFrame frame;
        frame.timestamp = ts;
        frame.points.reserve(N);

        for (int i = 0; i < N; i++, pitch += dpitch, yaw += dyaw) {
            uint16_t raw = (uint16_t)point_data[2*i] | ((uint16_t)point_data[2*i+1] << 8);
            if (raw == 0) continue;

            float r       = raw * range_scale;
            float sin_a   = sinf(pitch), cos_a = cosf(pitch);
            float sin_b   = sinf(yaw),   cos_b = cosf(yaw);

            float A = (-cos_t * sin_k + sin_t * sin_a * cos_k) * r + bias;
            float B = cos_a * cos_k * r;

            LidarPoint pt;
            pt.x         = cos_b * A - sin_b * B;
            pt.y         = sin_b * A + cos_b * B;
            pt.z         = (sin_t * sin_k + cos_t * sin_a * cos_k) * r + z_bias;
            pt.intensity = (float)aux_.reflect_data[i];
            pt.timestamp = ts;
            frame.points.push_back(pt);
        }
        pc_cb_(frame);
    }
}

// ── 串口初始化 ───────────────────────────────────────────────────
int L1Parser::open_serial(const std::string &port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);

    speed_t spd;
    switch (baudrate) {
        case 115200:  spd = B115200;  break;
        case 460800:  spd = B460800;  break;
        case 921600:  spd = B921600;  break;
        case 2000000: spd = B2000000; break;
        default:      spd = B2000000; break;
    }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tio);
    return fd;
}
