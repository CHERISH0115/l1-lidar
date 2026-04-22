#ifndef LORA_H
#define LORA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* LoRa 模块通过 UART 通信（SX1278 AT 指令模式）
 * 波特率: 9600, 8N1
 * M0=0, M1=0 → 正常收发模式
 */

#define LORA_UART   huart2
#define LORA_TX_PIN GPIO_PIN_2
#define LORA_RX_PIN GPIO_PIN_3
#define LORA_PORT   GPIOA

/* 自定义协议
 * 帧格式: 0xAA | CMD | LEN | DATA[LEN] | XOR_CHECKSUM | 0xBB
 */
#define LORA_FRAME_HEAD 0xAA
#define LORA_FRAME_TAIL 0xBB

/* 指令码 */
#define CMD_START_SLAM  0x01
#define CMD_STOP_SLAM   0x02
#define CMD_RESET       0x03
#define CMD_QUERY_STATE 0x04
#define CMD_STATE_RESP  0x10  /* 树莓派回传状态 */

void lora_init(void);
void lora_send_cmd(uint8_t cmd);
uint8_t lora_recv(uint8_t *buf, uint16_t *len, uint32_t timeout_ms);
uint8_t lora_calc_xor(const uint8_t *data, uint16_t len);

extern UART_HandleTypeDef LORA_UART;

#endif /* LORA_H */
