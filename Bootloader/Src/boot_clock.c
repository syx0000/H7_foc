/**
 * @file    boot_clock.c
 * @brief   Minimal clock setup for the bootloader (HSE + PLL → 480 MHz).
 *
 * Mirrors Core/Src/main.c::SystemClock_Config(), but trimmed to what the
 * bootloader needs (CPU + APB1/2 for USART1 + Flash latency).
 */
#include "stm32h7xx_hal.h"

void boot_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

    osc.OscillatorType    = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState          = RCC_HSE_ON;
    osc.PLL.PLLState      = RCC_PLL_ON;
    osc.PLL.PLLSource     = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM          = 5;
    osc.PLL.PLLN          = 192;
    osc.PLL.PLLP          = 2;
    osc.PLL.PLLQ          = 3;
    osc.PLL.PLLR          = 2;
    osc.PLL.PLLRGE        = RCC_PLL1VCIRANGE_2;
    osc.PLL.PLLVCOSEL     = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN      = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1);
    }

    clk.ClockType        = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                         | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                         | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource     = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider    = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider    = RCC_HCLK_DIV2;
    clk.APB3CLKDivider   = RCC_APB3_DIV2;
    clk.APB1CLKDivider   = RCC_APB1_DIV2;
    clk.APB2CLKDivider   = RCC_APB2_DIV2;
    clk.APB4CLKDivider   = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        while (1);
    }
}
