#ifndef __LORA_H
#define __LORA_H

#include "stm32f10x.h"
#include <stdint.h>

/* Protocol frame constants */
#define LORA_FRAME_HEAD  0xAA
#define LORA_FRAME_TAIL  0xBB

/* Commands (compatible with stm32_remote + extended LiDAR) */
#define CMD_START_SLAM     0x01
#define CMD_STOP_SLAM      0x02
#define CMD_RESET          0x03
#define CMD_QUERY_STATE    0x04
#define CMD_STATE_RESP     0x10

#define CMD_LIDAR_IMU      0x20
#define CMD_LIDAR_DIST     0x21
#define CMD_LIDAR_STATUS   0x22

void     lora_init(void);
void     lora_send(uint8_t cmd, const uint8_t *data, uint8_t len);
void     lora_send_cmd(uint8_t cmd);
uint8_t  lora_recv_nonblock(uint8_t *buf, uint16_t *len);

#endif
