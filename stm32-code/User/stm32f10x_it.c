#include "stm32f10x_it.h"
#include "lidar.h"

/* Millisecond counter incremented by SysTick */
static volatile uint32_t systick_ms = 0;

uint32_t millis(void)
{
    return systick_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms);
}

/* Cortex-M3 Processor Exceptions Handlers */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    systick_ms++;
}

/* USART1 RX interrupt - LiDAR data input */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = USART_ReceiveData(USART1);
        lidar_ring_push(byte);
        g_last_lidar_byte_ms = systick_ms;
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
    }
}
