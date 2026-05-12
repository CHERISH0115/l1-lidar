#include "lora.h"
#include <string.h>

/* UART handle for LoRa: USART2, PA2=TX, PA3=RX */
#define LORA_USART USART2

void lora_init(void)
{
    /* Module power-up delay */
    volatile uint32_t d;
    for (d = 0; d < 720000; d++) { __NOP(); }
}

static uint8_t lora_calc_xor(const uint8_t *data, uint16_t len)
{
    uint8_t x = 0;
    uint16_t i;
    for (i = 0; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

void lora_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t frame[70];  /* 1 head + 3 header + max 64 payload + 1 xor + 1 tail */
    uint8_t idx = 0;

    frame[idx++] = LORA_FRAME_HEAD;
    frame[idx++] = cmd;
    frame[idx++] = len;
    if (len > 0 && data != NULL) {
        memcpy(frame + idx, data, len);
        idx += len;
    }
    frame[idx++] = lora_calc_xor(frame + 1, 2 + len); /* XOR over CMD+LEN+DATA */
    frame[idx++] = LORA_FRAME_TAIL;

    uint8_t i;
    for (i = 0; i < idx; i++) {
        while (USART_GetFlagStatus(LORA_USART, USART_FLAG_TXE) == RESET);
        USART_SendData(LORA_USART, frame[i]);
    }
    while (USART_GetFlagStatus(LORA_USART, USART_FLAG_TC) == RESET);
}

void lora_send_cmd(uint8_t cmd)
{
    lora_send(cmd, NULL, 0);
}

uint8_t lora_recv_nonblock(uint8_t *buf, uint16_t *len)
{
    static uint8_t state = 0;
    static uint16_t idx  = 0;
    static uint8_t frm[72];  /* 1 head + 3 header + 64 data + 1 xor + 1 tail + margin */

    while (USART_GetFlagStatus(LORA_USART, USART_FLAG_RXNE) != RESET) {
        uint8_t byte = USART_ReceiveData(LORA_USART);

        if (state == 0) {
            if (byte == LORA_FRAME_HEAD) {
                idx = 0;
                frm[idx++] = byte;
                state = 1;
            }
        } else {
            frm[idx++] = byte;
            if (idx >= sizeof(frm)) { state = 0; continue; }
            if (byte == LORA_FRAME_TAIL && idx >= 5) {
                uint8_t data_len = frm[2];
                uint8_t expected = lora_calc_xor(frm + 1, 2 + data_len);
                if (expected == frm[idx - 2]) {
                    memcpy(buf, frm, idx);
                    *len = idx;
                    state = 0;
                    return 1;
                }
                state = 0;
            }
        }
    }
    return 0;
}
