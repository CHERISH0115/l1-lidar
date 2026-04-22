#ifndef KEY_H
#define KEY_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*
 * KEY1 → PA0  启动建图
 * KEY2 → PA1  停止建图
 * KEY3 → PA4  复位系统
 * KEY4 → PA5  查询状态
 */

typedef enum {
    KEY_NONE  = 0,
    KEY_START = 1,
    KEY_STOP  = 2,
    KEY_RESET = 3,
    KEY_QUERY = 4,
} KeyEvent;

void key_init(void);
KeyEvent key_scan(void);  /* 带消抖，调用间隔建议 10ms */

#endif /* KEY_H */
