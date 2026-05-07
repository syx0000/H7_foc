/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define VDC_Pin GPIO_PIN_6
#define VDC_GPIO_Port GPIOA
#define CUR_A_Pin GPIO_PIN_7
#define CUR_A_GPIO_Port GPIOA
#define CUR_B_Pin GPIO_PIN_4
#define CUR_B_GPIO_Port GPIOC
#define CUR_C_Pin GPIO_PIN_5
#define CUR_C_GPIO_Port GPIOC
#define TEMP_MOTOR_Pin GPIO_PIN_0
#define TEMP_MOTOR_GPIO_Port GPIOB
#define TEMP_MOS_Pin GPIO_PIN_1
#define TEMP_MOS_GPIO_Port GPIOB
#define STB_Pin GPIO_PIN_10
#define STB_GPIO_Port GPIOA
#define EN_GATE_Pin GPIO_PIN_4
#define EN_GATE_GPIO_Port GPIOD
#define ERR_RUN_Pin GPIO_PIN_4
#define ERR_RUN_GPIO_Port GPIOB
#define LED_RUN_Pin GPIO_PIN_5
#define LED_RUN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
