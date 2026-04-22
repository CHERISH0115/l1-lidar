/*
 * 宇树 L1 MAVLink 协议验证程序
 * 依赖: unitree_lidar_sdk MAVLink 头文件
 */
#include "mavlink/SysMavlink/mavlink.h"
#include "parse_range_auxiliary_data_to_cloud.h"

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <string.h>

#define PORT     "/dev/ttyUSB0"
#define BAUDRATE B2000000

static volatile int running = 1;
static void on_signal(int s) { (void)s; running = 0; }

static int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return -1; }
    struct termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, BAUDRATE);
    cfsetospeed(&tio, BAUDRATE);
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tio);
    return fd;
}

int main(void) {
    signal(SIGINT, on_signal);

    int fd = open_serial(PORT);
    if (fd < 0) return 1;
    printf("Opened %s @ 2000000 baud (MAVLink v2 parser)\n\n", PORT);

    mavlink_message_t msg;
    mavlink_status_t  status;

    /* 暂存辅助帧，等距离帧到来后合并算 XYZ */
    mavlink_ret_lidar_auxiliary_data_packet_t aux = {0};
    int has_aux = 0;

    long imu_cnt = 0, scan_cnt = 0;
    uint8_t buf[512];

    while (running) {
        ssize_t n = read(fd, buf, sizeof(buf));
        for (ssize_t i = 0; i < n; i++) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                continue;

            /* --- IMU 帧 (msgid=19) --- */
            if (msg.msgid == MAVLINK_MSG_ID_RET_IMU_ATTITUDE_DATA_PACKET) {
                mavlink_ret_imu_attitude_data_packet_t imu;
                mavlink_msg_ret_imu_attitude_data_packet_decode(&msg, &imu);
                imu_cnt++;
                if (imu_cnt % 200 == 1) {
                    printf("[IMU #%ld] q=(%.3f,%.3f,%.3f,%.3f) "
                           "gyro=(%.3f,%.3f,%.3f) acc=(%.3f,%.3f,%.3f)\n",
                           imu_cnt,
                           imu.quaternion[0], imu.quaternion[1],
                           imu.quaternion[2], imu.quaternion[3],
                           imu.angular_velocity[0], imu.angular_velocity[1],
                           imu.angular_velocity[2],
                           imu.linear_acceleration[0], imu.linear_acceleration[1],
                           imu.linear_acceleration[2]);
                }

            /* --- 辅助帧 (msgid=17) --- */
            } else if (msg.msgid == MAVLINK_MSG_ID_RET_LIDAR_AUXILIARY_DATA_PACKET) {
                mavlink_msg_ret_lidar_auxiliary_data_packet_decode(&msg, &aux);
                has_aux = 1;

            /* --- 距离帧 (msgid=16) --- */
            } else if (msg.msgid == MAVLINK_MSG_ID_RET_LIDAR_DISTANCE_DATA_PACKET) {
                mavlink_ret_lidar_distance_data_packet_t dist;
                mavlink_msg_ret_lidar_distance_data_packet_decode(&msg, &dist);
                scan_cnt++;

                if (has_aux && aux.packet_id == dist.packet_id) {
                    /* 用 SDK 提供的函数把距离+辅助数据转成 XYZ */
                    std::vector<std::array<float,4>> cloud;
                    parseRangeAuxiliaryDataToCloud(aux, dist, cloud);
                    if (scan_cnt % 50 == 1 && !cloud.empty()) {
                        printf("[SCAN #%ld] %zu pts, first=(%.3f,%.3f,%.3f) inten=%.0f\n",
                               scan_cnt, cloud.size(),
                               cloud[0][0], cloud[0][1], cloud[0][2], cloud[0][3]);
                    }
                } else if (scan_cnt % 50 == 1) {
                    /* 还没收到匹配的辅助帧，只打印原始距离 */
                    uint16_t d0 = dist.point_data[0] | (dist.point_data[1] << 8);
                    printf("[SCAN #%ld] raw first_dist=%u mm (no aux yet)\n",
                           scan_cnt, d0);
                }
            }
        }
    }

    printf("\nDone. IMU: %ld  Scan: %ld\n", imu_cnt, scan_cnt);
    close(fd);
    return 0;
}
