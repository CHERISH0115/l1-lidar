#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* SSD1306 OLED，I2C接口，地址0x78（7位0x3C） */
#define OLED_I2C        hi2c1
#define OLED_ADDR       0x78
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

extern I2C_HandleTypeDef OLED_I2C;

void oled_init(void);
void oled_clear(void);
void oled_set_cursor(uint8_t col, uint8_t row);  /* row: 0-7 (每行8像素) */
void oled_printf(uint8_t row, const char *fmt, ...);
void oled_refresh(void);

#endif /* OLED_H */
