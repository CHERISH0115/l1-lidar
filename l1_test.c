#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#define PORT     "/dev/ttyUSB0"
#define BAUDRATE B2000000

#define HEADER0  0xAA
#define HEADER1  0xBB
#define TAIL0    0xCC
#define TAIL1    0xDD
#define TYPE_IMU  0x01
#define TYPE_SCAN 0x02

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
    printf("Opened %s @ 2000000 baud\n", PORT);

    uint8_t buf[8192];
    size_t  buf_len = 0;

    long imu_count  = 0;
    long scan_count = 0;
    long scan_pts   = 0;

    while (running) {
        ssize_t n = read(fd, buf + buf_len, sizeof(buf) - buf_len);
        if (n > 0) buf_len += n;

        /* 逐帧解析 */
        while (buf_len >= 8) {
            /* 搜帧头 */
            size_t start = buf_len;
            for (size_t i = 0; i + 1 < buf_len; i++) {
                if (buf[i] == HEADER0 && buf[i+1] == HEADER1) { start = i; break; }
            }
            if (start > 0) {
                memmove(buf, buf + start, buf_len - start);
                buf_len -= start;
            }
            if (buf_len < 8) break;

            uint16_t payload_len = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
            size_t total = 2 + 1 + 2 + payload_len + 1 + 2;
            if (buf_len < total) break;

            /* 帧尾校验 */
            if (buf[total-2] != TAIL0 || buf[total-1] != TAIL1) {
                memmove(buf, buf + 1, buf_len - 1);
                buf_len--;
                continue;
            }

            /* XOR 校验 */
            uint8_t crc = 0;
            for (size_t i = 5; i < 5 + payload_len; i++) crc ^= buf[i];
            if (crc != buf[5 + payload_len]) {
                memmove(buf, buf + total, buf_len - total);
                buf_len -= total;
                continue;
            }

            uint8_t type = buf[2];
            const uint8_t *payload = buf + 5;

            if (type == TYPE_IMU && payload_len >= 24) {
                float ax, ay, az, gx, gy, gz;
                memcpy(&ax, payload+0,  4);
                memcpy(&ay, payload+4,  4);
                memcpy(&az, payload+8,  4);
                memcpy(&gx, payload+12, 4);
                memcpy(&gy, payload+16, 4);
                memcpy(&gz, payload+20, 4);
                imu_count++;
                if (imu_count % 100 == 1)
                    printf("[IMU #%ld] acc=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f)\n",
                           imu_count, ax, ay, az, gx, gy, gz);

            } else if (type == TYPE_SCAN) {
                size_t npts = payload_len / 16;
                scan_count++;
                scan_pts += npts;
                if (scan_count % 20 == 1) {
                    float x, y, z, intensity;
                    memcpy(&x,         payload+0,  4);
                    memcpy(&y,         payload+4,  4);
                    memcpy(&z,         payload+8,  4);
                    memcpy(&intensity, payload+12, 4);
                    printf("[SCAN #%ld] %zu pts, first=(%.3f,%.3f,%.3f) inten=%.1f\n",
                           scan_count, npts, x, y, z, intensity);
                }
            }

            memmove(buf, buf + total, buf_len - total);
            buf_len -= total;
        }
    }

    printf("\nDone. IMU frames: %ld, Scan frames: %ld, total pts: %ld\n",
           imu_count, scan_count, scan_pts);
    close(fd);
    return 0;
}
