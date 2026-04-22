#include "key.h"

#define KEY1_PORT GPIOA
#define KEY1_PIN  GPIO_PIN_0
#define KEY2_PORT GPIOA
#define KEY2_PIN  GPIO_PIN_1
#define KEY3_PORT GPIOA
#define KEY3_PIN  GPIO_PIN_4
#define KEY4_PORT GPIOA
#define KEY4_PIN  GPIO_PIN_5

void key_init(void) {
    /* GPIO 时钟在 CubeMX 中配置，此处仅作说明
     * 所有按键引脚配置为输入上拉（GPIO_MODE_INPUT, GPIO_PULLUP）
     * 按键按下时引脚拉低 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef init = {0};
    init.Mode  = GPIO_MODE_INPUT;
    init.Pull  = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Pin   = KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN;
    HAL_GPIO_Init(GPIOA, &init);
}

KeyEvent key_scan(void) {
    static uint8_t last_state = 0xFF;
    uint8_t cur = 0xFF;

    if (!HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN)) cur &= ~0x01;
    if (!HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN)) cur &= ~0x02;
    if (!HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN)) cur &= ~0x04;
    if (!HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN)) cur &= ~0x08;

    uint8_t pressed = (~cur) & last_state; /* 检测下降沿 */
    last_state = cur;

    if (pressed & 0x01) return KEY_START;
    if (pressed & 0x02) return KEY_STOP;
    if (pressed & 0x04) return KEY_RESET;
    if (pressed & 0x08) return KEY_QUERY;
    return KEY_NONE;
}
