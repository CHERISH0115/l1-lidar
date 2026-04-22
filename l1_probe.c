#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

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

static float le_float(const uint8_t *p) {
    float v; memcpy(&v, p, 4); return v;
}
static uint16_t le_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int main(void) {
    signal(SIGINT, on_signal);
    int fd = open_serial(PORT);
    if (fd < 0) return 1;
    printf("Probing %s ...\n\n", PORT);

    uint8_t buf[16384];
    size_t  buf_len = 0;
    int frame_count = 0;

    while (running && frame_count < 30) {
        ssize_t n = read(fd, buf + buf_len, sizeof(buf) - buf_len);
        if (n > 0) buf_len += n;
        if (buf_len < 16) continue;

        /* 搜索 0xfd 作为帧起始字节 */
        size_t start = buf_len;
        for (size_t i = 0; i + 1 < buf_len; i++) {
            if (buf[i] == 0xfd) { start = i; break; }
        }
        if (start > 0) {
            memmove(buf, buf + start, buf_len - start);
            buf_len -= start;
        }
        if (buf_len < 16) continue;

        uint8_t  type2  = buf[1];                   /* 第2字节：帧类型 */
        uint16_t seq    = le_u16(buf + 4);          /* 字节4-5：序列号 */
        uint8_t  sub    = buf[6];                   /* 字节6 */
        uint8_t  count  = buf[7];                   /* 字节7：数据计数 */

        /* 估算帧长：头10字节 + count个元素 */
        size_t elem_size = (type2 == 0x2a) ? 4 : 2; /* 0x2a→float, 其他→uint16 */
        size_t total = 10 + (size_t)count * elem_size;
        if (total > sizeof(buf) - 4) total = 10 + 256; /* 保护 */
        if (buf_len < total + 2) { /* 等数据 */
            /* 如果 count=0 且数据很多，说明 count 字段不对，跳过一帧固定长度 */
            if (buf_len > 300) {
                memmove(buf, buf + 1, buf_len - 1);
                buf_len--;
            }
            continue;
        }

        printf("=== Frame #%d ===\n", ++frame_count);
        printf("  type2=0x%02x  seq=%u  sub=0x%02x  count=%u  est_total=%zu\n",
               type2, seq, sub, count, total);

        /* 打印前10字节头 */
        printf("  header: ");
        for (int i = 0; i < 10 && i < (int)buf_len; i++) printf("%02x ", buf[i]);
        printf("\n");

        const uint8_t *payload = buf + 10;
        size_t pay_len = (buf_len > total) ? total - 10 : buf_len - 10;

        if (type2 == 0x2a && pay_len >= 24) {
            /* IMU帧：尝试解析为浮点 */
            printf("  [IMU?] floats: ");
            for (size_t i = 0; i + 3 < pay_len; i += 4)
                printf("%.4f ", le_float(payload + i));
            printf("\n");
        } else {
            /* Scan帧：尝试解析为uint16 */
            printf("  [SCAN?] first 16 u16: ");
            for (size_t i = 0; i + 1 < pay_len && i < 32; i += 2)
                printf("%u ", le_u16(payload + i));
            printf("\n");
        }

        /* 跳过这一帧，继续找下一个 0xfd */
        size_t skip = 1;
        for (size_t i = 1; i + 1 < buf_len; i++) {
            if (buf[i] == 0xfd) { skip = i; break; }
            if (i == buf_len - 2) skip = buf_len - 1;
        }
        memmove(buf, buf + skip, buf_len - skip);
        buf_len -= skip;
    }

    close(fd);
    return 0;
}
