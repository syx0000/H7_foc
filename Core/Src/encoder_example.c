/**
  ******************************************************************************
  * @file    encoder_example.c
  * @brief   DPT encoder usage example
  *
  * Usage:
  * 1. In main.c, after MX_USART2_UART_Init():
  *    DPT_Encoder_Init(&huart2, RS485_DIR_GPIO_Port, RS485_DIR_Pin);
  *
  * 2. Add this file's callbacks to stm32h7xx_it.c or main.c USER CODE sections
  *
  * 3. In main loop:
  *    DPT_Angles angles;
  *    if (DPT_ReadDualAngle_DMA(&angles, 20) == DPT_OK) {
  *        printf("Inner: %.2f deg, Outer: %.2f deg\n",
  *               angles.inner_deg, angles.outer_deg);
  *    }
  ******************************************************************************
  */

#include "encoder.h"
#include "usart.h"

/* ---- HAL Callbacks (add to stm32h7xx_it.c USER CODE sections) ------------ */

/**
  * @brief  Tx Transfer completed callback (add to USER CODE in stm32h7xx_it.c)
  * @note   This callback is called when UART TX DMA completes.
  *         It switches RS485 direction pin and starts RX.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        DPT_UART_TxCpltCallback(huart);
    }
}

/**
  * @brief  Rx Transfer completed callback (add to USER CODE in stm32h7xx_it.c)
  * @note   This callback is called when UART RX DMA completes.
  *         It validates CRC and parses encoder data.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        DPT_UART_RxCpltCallback(huart);
    }
}

/* ---- Example usage in main.c --------------------------------------------- */
#if 0

void encoder_test(void)
{
    DPT_Angles angles;
    DPT_Status status;

    /* Blocking DMA read (waits for completion) */
    status = DPT_ReadDualAngle_DMA(&angles, 20);
    if (status == DPT_OK) {
        printf("Inner: %.2f deg (0x%06lX)\n", angles.inner_deg, angles.inner_raw);
        printf("Outer: %.2f deg (0x%06lX)\n", angles.outer_deg, angles.outer_raw);
    } else {
        printf("Read error: %d\n", status);
    }

    /* Read with status byte */
    status = DPT_ReadDualAngleWithStatus_DMA(&angles, 20);
    if (status == DPT_OK) {
        printf("Status: 0x%02X\n", angles.status);
        if (angles.status & DPT_STATUS_RS485_ERR_BIT) {
            printf("  RS485 error detected\n");
        }
    }

    /* Read temperature */
    int16_t temp;
    status = DPT_ReadTemperature_DMA(&temp, 20);
    if (status == DPT_OK) {
        printf("Temperature: %.1f C\n", temp / 10.0f);
    }
}

#endif
