#include "key.h"

#define KEY_PORT  GPIOA
#define KEY1_PIN  GPIO_Pin_0
#define KEY2_PIN  GPIO_Pin_1
#define KEY3_PIN  GPIO_Pin_4
#define KEY4_PIN  GPIO_Pin_5
#define KEY_MASK  (KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN)

void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = KEY_MASK;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY_PORT, &GPIO_InitStructure);
}

KeyEvent key_scan(void)
{
    static uint8_t last_state = 0xFF;
    uint16_t port_val = GPIO_ReadInputData(KEY_PORT);
    uint8_t cur;

    cur  = (port_val & KEY1_PIN) ? 0x01 : 0;
    cur |= (port_val & KEY2_PIN) ? 0x02 : 0;
    cur |= (port_val & KEY3_PIN) ? 0x04 : 0;
    cur |= (port_val & KEY4_PIN) ? 0x08 : 0;

    uint8_t pressed = (~cur) & last_state;
    last_state = cur;

    if (pressed & 0x01) return KEY_START;
    if (pressed & 0x02) return KEY_STOP;
    if (pressed & 0x04) return KEY_RESET;
    if (pressed & 0x08) return KEY_QUERY;

    return KEY_NONE;
}
