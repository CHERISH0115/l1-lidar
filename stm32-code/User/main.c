#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "key.h"
#include "oled.h"
#include "lora.h"
#include "lidar.h"
#include <string.h>
#include <stdio.h>

/* System state tracking */
typedef struct {
    uint8_t  slam_running;
    float    pos_x;
    float    pos_y;
    float    fps;
    uint16_t imu_pkt_count;
    uint16_t dist_pkt_count;
} SystemState;

static SystemState g_state = {0};

/* Forward declarations */
static void RCC_Config(void);
static void GPIO_Config(void);
static void USART1_Config(uint32_t baudrate);
static void USART2_Config(uint32_t baudrate);
static void I2C1_Config(void);
static void NVIC_Config(void);
static void parse_lora_response(const uint8_t *buf, uint16_t len);
static void update_oled(void);
static void lora_send_lidar_imu(void);
static void lora_send_lidar_dist(void);
static void lora_send_lidar_status(void);

/* ======================== Main Entry =============================== */
int main(void)
{
    /* SysTick at 1ms (SystemCoreClock = 72MHz) */
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);
    }

    /* Peripheral initialization */
    RCC_Config();
    GPIO_Config();
    USART1_Config(921600);
    USART2_Config(9600);
    I2C1_Config();
    NVIC_Config();

    /* Module initialization */
    oled_init();
    lora_init();
    key_init();

    /* Boot screen */
    oled_printf(0, "L1-LiDAR v1.0");
    oled_printf(1, "Starting...");
    oled_refresh();
    delay_ms(500);
    oled_clear();
    oled_printf(0, "L1-LiDAR v1.0");
    oled_printf(1, "Ready.");
    oled_refresh();

    /* Main loop timing */
    uint32_t last_key_ms      = 0;
    uint32_t last_imu_tx_ms   = 0;
    uint32_t last_dist_tx_ms  = 0;
    uint32_t last_status_ms   = 0;
    uint32_t last_oled_ms     = 0;
    uint32_t last_watchdog_ms = 0;
    uint32_t last_query_ms    = 0;

    uint16_t imu_count  = 0;
    uint16_t dist_count = 0;

    while (1) {
        uint32_t now = millis();

        /* Service USART1 ring buffer (LiDAR data) */
        {
            uint8_t byte;
            while (lidar_ring_pop(&byte)) {
                mavlink_feed_byte(byte);
            }
        }

        /* Update packet counters */
        if (g_imu.updated) {
            g_imu.updated = 0;
            imu_count++;
        }
        if (g_dist.updated) {
            g_dist.updated = 0;
            dist_count++;
        }

        /* LoRa non-blocking receive */
        {
            uint8_t  rx_buf[64];
            uint16_t rx_len = 0;
            if (lora_recv_nonblock(rx_buf, &rx_len)) {
                parse_lora_response(rx_buf, rx_len);
            }
        }

        /* Key scan (10ms) */
        if (now - last_key_ms >= 10) {
            last_key_ms = now;
            KeyEvent ev = key_scan();
            switch (ev) {
            case KEY_START:
                lora_send_cmd(CMD_START_SLAM);
                g_state.slam_running = 1;
                break;
            case KEY_STOP:
                lora_send_cmd(CMD_STOP_SLAM);
                g_state.slam_running = 0;
                break;
            case KEY_RESET:
                lora_send_cmd(CMD_RESET);
                memset(&g_state, 0, sizeof(g_state));
                break;
            case KEY_QUERY:
                lora_send_cmd(CMD_QUERY_STATE);
                break;
            default:
                break;
            }
        }

        /* Auto query (2s) */
        if (now - last_query_ms >= 2000) {
            last_query_ms = now;
            lora_send_cmd(CMD_QUERY_STATE);
        }

        /* Send IMU via LoRa (10Hz) */
        if (now - last_imu_tx_ms >= 100) {
            last_imu_tx_ms = now;
            lora_send_lidar_imu();
        }

        /* Send distance via LoRa (5Hz) */
        if (now - last_dist_tx_ms >= 200) {
            last_dist_tx_ms = now;
            lora_send_lidar_dist();
        }

        /* Status heartbeat (1Hz) */
        if (now - last_status_ms >= 1000) {
            last_status_ms = now;
            g_state.imu_pkt_count  = imu_count;
            g_state.dist_pkt_count = dist_count;
            imu_count  = 0;
            dist_count = 0;
            lora_send_lidar_status();
        }

        /* Update OLED (5Hz) */
        if (now - last_oled_ms >= 200) {
            last_oled_ms = now;
            update_oled();
            oled_refresh();
        }

        /* LiDAR connection watchdog (1s check, 3s timeout) */
        if (now - last_watchdog_ms >= 1000) {
            last_watchdog_ms = now;
            if (now - g_last_lidar_byte_ms > 3000) {
                g_lidar_connected = 0;
            } else {
                g_lidar_connected = 1;
            }
        }

        /* Toggle LED at 1Hz as heartbeat */
        {
            static uint32_t last_led_ms = 0;
            if (now - last_led_ms >= 500) {
                last_led_ms = now;
                static uint8_t led_state = 0;
                led_state ^= 1;
                if (led_state) {
                    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
                } else {
                    GPIO_SetBits(GPIOC, GPIO_Pin_13);
                }
            }
        }
    }
}

/* ===================== Peripheral Init ============================== */

static void RCC_Config(void)
{
    /* SystemInit() already configured 72MHz HSE+PLL in startup.
       Enable peripheral clocks here. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_USART1 |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |
                           RCC_APB1Periph_I2C1, ENABLE);
}

static void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PC13: Status LED (push-pull output) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    /* USART1: PA9=TX (AF push-pull) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1: PA10=RX (floating input) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2: PA2=TX (AF push-pull) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2: PA3=RX (floating input) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* I2C1: PB6=SCL, PB7=SDA (alternate function open-drain) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void USART1_Config(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static void USART2_Config(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

static void I2C1_Config(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

static void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* SysTick priority set by SysTick_Config() — do not use NVIC_Init with SysTick_IRQn */

    /* USART1: preemption 1, sub 0 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ===================== LoRa Data Send ============================== */

static void lora_send_lidar_imu(void)
{
    uint8_t payload[28];
    uint32_t ts = millis();

    memcpy(payload,      &ts,               4);
    memcpy(payload + 4,  (void*)&g_imu.acc_x,  4);
    memcpy(payload + 8,  (void*)&g_imu.acc_y,  4);
    memcpy(payload + 12, (void*)&g_imu.acc_z,  4);
    memcpy(payload + 16, (void*)&g_imu.gyro_x, 4);
    memcpy(payload + 20, (void*)&g_imu.gyro_y, 4);
    memcpy(payload + 24, (void*)&g_imu.gyro_z, 4);

    lora_send(CMD_LIDAR_IMU, payload, sizeof(payload));
}

static void lora_send_lidar_dist(void)
{
    uint16_t sectors[8];
    uint8_t  payload[16];
    uint8_t  i;

    lidar_compress_distances((const uint16_t*)g_dist.distances, NULL, sectors);

    for (i = 0; i < 8; i++) {
        payload[2 * i]     = (uint8_t)(sectors[i] & 0xFF);
        payload[2 * i + 1] = (uint8_t)(sectors[i] >> 8);
    }

    lora_send(CMD_LIDAR_DIST, payload, sizeof(payload));
}

static void lora_send_lidar_status(void)
{
    uint8_t payload[4];
    payload[0] = g_lidar_connected ? 1 : 0;
    payload[1] = (uint8_t)g_state.imu_pkt_count;
    payload[2] = (uint8_t)g_state.dist_pkt_count;
    payload[3] = 0;

    lora_send(CMD_LIDAR_STATUS, payload, sizeof(payload));
}

/* ===================== LoRa Response Parse ========================= */

static void parse_lora_response(const uint8_t *buf, uint16_t len)
{
    if (len < 5) return;
    if (buf[0] != LORA_FRAME_HEAD) return;
    if (buf[1] != CMD_STATE_RESP) return;

    uint8_t data_len = buf[2];
    if (len < (uint16_t)(data_len + 5)) return;
    if (data_len < 12) return;

    memcpy(&g_state.fps,   buf + 3,      4);
    memcpy(&g_state.pos_x, buf + 3 + 4,  4);
    memcpy(&g_state.pos_y, buf + 3 + 8,  4);
}

/* ===================== OLED Display ================================ */

static void update_oled(void)
{
    oled_printf(0, "L1-LiDAR v1.0");

    if (g_state.slam_running) {
        oled_printf(1, "SLAM: RUNNING");
    } else {
        oled_printf(1, "SLAM: STOPPED ");
    }

    if (g_lidar_connected) {
        oled_printf(2, "A:%.1f G:%.1f",
                    g_imu.acc_x, g_imu.gyro_z);
    } else if (g_state.fps > 0.0f) {
        oled_printf(2, "FPS:%.1f", g_state.fps);
    } else {
        oled_printf(2, "Waiting data...");
    }

    if (g_state.pos_x != 0.0f || g_state.pos_y != 0.0f) {
        oled_printf(3, "X:%.2f Y:%.2f", g_state.pos_x, g_state.pos_y);
    } else {
        oled_printf(3, "%s I:%d D:%d",
                    g_lidar_connected ? "LDR:OK" : "LDR:--",
                    g_state.imu_pkt_count,
                    g_state.dist_pkt_count);
    }

    /* Distance sector overview (line 4-5) */
    {
        uint16_t sectors[8];
        int pos;
        char line4[22];
        char line5[22];
        uint8_t i;

        lidar_compress_distances((const uint16_t*)g_dist.distances, NULL, sectors);

        pos = snprintf(line4, sizeof(line4), "D:");
        for (i = 0; i < 4 && pos < 21; i++) {
            float m = sectors[i] * 0.001f;
            pos += snprintf(line4 + pos, sizeof(line4) - pos,
                            "%.1f ", m);
        }
        oled_printf(4, "%s", line4);

        pos = snprintf(line5, sizeof(line5), "  ");
        for (i = 4; i < 8 && pos < 21; i++) {
            float m = sectors[i] * 0.001f;
            pos += snprintf(line5 + pos, sizeof(line5) - pos,
                            "%.1f ", m);
        }
        oled_printf(5, "%s", line5);
    }

    oled_printf(6, "Keys:S=Stp R=Rst");
    oled_printf(7, "     Q=Qry");
}
