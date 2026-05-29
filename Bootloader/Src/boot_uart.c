/**
 * @file    boot_uart.c
 * @brief   USART1 setup + blocking printf retarget for the bootloader.
 *
 * USART1 layout (matches application Core/Src/usart.c):
 *   PB14 = TX (AF4)
 *   PB15 = RX (AF4)
 *   921600 8N1, no flow control, no FIFO.
 *
 * Bootloader uses polling-mode HAL_UART_Transmit (blocking) — no DMA, no
 * interrupts. Application later re-initialises USART1 with DMA + IRQ.
 */
#include "stm32h7xx_hal.h"
#include <stdio.h>

static UART_HandleTypeDef boot_huart1;

void boot_uart_init(void)
{
    /* Peripheral clock select: D2PCLK2 (= APB2 PCLK) */
    RCC_PeriphCLKInitTypeDef p = {0};
    p.PeriphClockSelection    = RCC_PERIPHCLK_USART1;
    p.Usart16ClockSelection   = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&p);

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOB, &g);

    boot_huart1.Instance              = USART1;
    boot_huart1.Init.BaudRate         = 921600;
    boot_huart1.Init.WordLength       = UART_WORDLENGTH_8B;
    boot_huart1.Init.StopBits         = UART_STOPBITS_1;
    boot_huart1.Init.Parity           = UART_PARITY_NONE;
    boot_huart1.Init.Mode             = UART_MODE_TX_RX;
    boot_huart1.Init.HwFlowCtl        = UART_HWCONTROL_NONE;
    boot_huart1.Init.OverSampling     = UART_OVERSAMPLING_16;
    boot_huart1.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    boot_huart1.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    boot_huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&boot_huart1);
    HAL_UARTEx_DisableFifoMode(&boot_huart1);
}

/* MicroLIB / Arm Compiler 6 retarget: blocking, one byte at a time. */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t b = (uint8_t)ch;
    HAL_UART_Transmit(&boot_huart1, &b, 1, 100);
    return ch;
}
