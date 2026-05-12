#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

typedef enum {
    KEY_NONE  = 0,
    KEY_START = 1,
    KEY_STOP  = 2,
    KEY_RESET = 3,
    KEY_QUERY = 4,
} KeyEvent;

void     key_init(void);
KeyEvent key_scan(void);

#endif
