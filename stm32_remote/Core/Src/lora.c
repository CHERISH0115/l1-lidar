#include "lora.h"
#include <string.h>

void lora_init(void) {
    /* LoRa 模块上电后默认进入正常模式（M0=M1=0）
     * 此处不发送AT指令，直接使用出厂配置（433MHz，SF7，BW125K）
     * 若需修改信道，可在此处添加 AT+ADDRESS 等配置命令
     */
    HAL_Delay(100);
}

uint8_t lora_calc_xor(const uint8_t *data, uint16_t len) {
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) xor_val ^= data[i];
    return xor_val;
}

void lora_send_cmd(uint8_t cmd) {
    /* 帧格式: HEAD | CMD | LEN(0) | XOR | TAIL */
    uint8_t frame[5];
    frame[0] = LORA_FRAME_HEAD;
    frame[1] = cmd;
    frame[2] = 0x00;                       /* 无附加数据 */
    frame[3] = lora_calc_xor(&cmd, 1);
    frame[4] = LORA_FRAME_TAIL;
    HAL_UART_Transmit(&LORA_UART, frame, sizeof(frame), 100);
}

uint8_t lora_recv(uint8_t *buf, uint16_t *len, uint32_t timeout_ms) {
    uint8_t tmp;
    uint32_t start = HAL_GetTick();
    uint16_t idx   = 0;
    uint8_t  state = 0; /* 0=等帧头 1=接收中 */

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (HAL_UART_Receive(&LORA_UART, &tmp, 1, 5) != HAL_OK) continue;

        if (state == 0) {
            if (tmp == LORA_FRAME_HEAD) { idx = 0; buf[idx++] = tmp; state = 1; }
        } else {
            buf[idx++] = tmp;
            if (idx >= 64) { state = 0; continue; } /* 防溢出 */
            if (tmp == LORA_FRAME_TAIL && idx >= 5) {
                /* 校验 */
                uint8_t xor_calc = lora_calc_xor(buf + 1, buf[2] + 2);
                if (xor_calc == buf[idx - 2]) {
                    *len = idx;
                    return 1; /* 接收成功 */
                }
                state = 0;
            }
        }
    }
    return 0; /* 超时 */
}
