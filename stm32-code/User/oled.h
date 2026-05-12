#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"
#include <stdint.h>

#define OLED_ADDR       0x3C  /* 7-bit I2C address */
#define OLED_COLS       128
#define OLED_ROWS       64
#define OLED_PAGES      8

void oled_init(void);
void oled_clear(void);
void oled_printf(uint8_t row, const char *fmt, ...);
void oled_refresh(void);

#endif
