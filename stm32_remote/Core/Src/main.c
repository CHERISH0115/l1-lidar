/* main.c — STM32遥控终端主程序
 * 硬件: STM32F103C8T6 / 蓝桥杯F103系列
 * 外设: UART2(LoRa), I2C1(OLED), GPIOA(按键)
 */

#include "main.h"
#include "lora.h"
#include "oled.h"
#include "key.h"
#include <stdio.h>
#include <string.h>

/* HAL 句柄（由 CubeMX 生成，此处声明使用） */
UART_HandleTypeDef huart2;
I2C_HandleTypeDef  hi2c1;

/* 系统状态 */
typedef struct {
    uint8_t  slam_running;
    float    pos_x;
    float    pos_y;
    float    fps;
    char     info[32];
} SystemState;

static SystemState g_state = {0};

static void oled_show_state(void);
static void parse_response(const uint8_t *buf, uint16_t len);

int main(void) {
    HAL_Init();
    SystemClock_Config(); /* 由 CubeMX 生成 */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    key_init();
    oled_init();
    lora_init();

    oled_printf(0, "SLAM Remote v1.0");
    oled_printf(1, "Initializing...");
    HAL_Delay(500);
    oled_clear();
    oled_printf(0, "SLAM Remote v1.0");
    oled_printf(1, "Ready.");

    uint32_t last_key_tick   = 0;
    uint32_t last_query_tick = 0;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* 按键扫描（每10ms一次） */
        if (now - last_key_tick >= 10) {
            last_key_tick = now;
            KeyEvent ev = key_scan();
            switch (ev) {
                case KEY_START:
                    lora_send_cmd(CMD_START_SLAM);
                    g_state.slam_running = 1;
                    oled_printf(1, "CMD: START");
                    break;
                case KEY_STOP:
                    lora_send_cmd(CMD_STOP_SLAM);
                    g_state.slam_running = 0;
                    oled_printf(1, "CMD: STOP ");
                    break;
                case KEY_RESET:
                    lora_send_cmd(CMD_RESET);
                    memset(&g_state, 0, sizeof(g_state));
                    oled_printf(1, "CMD: RESET");
                    break;
                case KEY_QUERY:
                    lora_send_cmd(CMD_QUERY_STATE);
                    oled_printf(1, "CMD: QUERY");
                    break;
                default:
                    break;
            }
        }

        /* 定时主动查询（每2秒） */
        if (now - last_query_tick >= 2000) {
            last_query_tick = now;
            lora_send_cmd(CMD_QUERY_STATE);
        }

        /* 接收回包（非阻塞，50ms超时） */
        uint8_t  recv_buf[64];
        uint16_t recv_len = 0;
        if (lora_recv(recv_buf, &recv_len, 50)) {
            parse_response(recv_buf, recv_len);
            oled_show_state();
        }
    }
}

/* 解析树莓派回传的状态帧
 * 格式: HEAD | CMD_STATE_RESP | LEN | fps(4B float) | x(4B float) | y(4B float) | XOR | TAIL
 */
static void parse_response(const uint8_t *buf, uint16_t len) {
    if (len < 5) return;
    if (buf[0] != LORA_FRAME_HEAD || buf[1] != CMD_STATE_RESP) return;
    uint8_t data_len = buf[2];
    if (data_len >= 12 && len >= (uint16_t)(data_len + 5)) {
        memcpy(&g_state.fps,   buf + 3 + 0, 4);
        memcpy(&g_state.pos_x, buf + 3 + 4, 4);
        memcpy(&g_state.pos_y, buf + 3 + 8, 4);
    }
}

static void oled_show_state(void) {
    oled_printf(0, "SLAM Remote v1.0");
    oled_printf(1, g_state.slam_running ? "Status: RUNNING" : "Status: STOPPED");
    oled_printf(2, "FPS:%.1f", g_state.fps);
    oled_printf(3, "X:%.2fm Y:%.2fm", g_state.pos_x, g_state.pos_y);
}
