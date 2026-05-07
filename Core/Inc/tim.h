/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
/* DWT (Data Watchpoint and Trace) cycle counter for microsecond timing */
void DWT_Init(void);
uint32_t DWT_GetCycles(void);
uint32_t DWT_GetMicros(void);  // 获取从初始化后经过的微秒数
uint32_t DWT_CyclesToUs(uint32_t cycles);  // 周期数转微秒

/* TIM1 PWM启动：使能输出、中断、计数器，启动CH4输出比较 */
void TIM1_PWM_Start(void);

/* TIM1时间戳（线性化：上升=CNT，下降=23999-CNT，一个完整周期0~23999 counts≈100us） */
extern volatile uint32_t g_tim1_cc4_cnt;         /* CC4中断进入时 */
extern volatile uint32_t g_tim1_cc4_exit_cnt;    /* CC4中断退出时 */
extern volatile uint32_t g_tim1_enc_done_cnt;    /* 编码器读取完成时 */
extern volatile uint32_t g_tim1_update_cnt;      /* UP中断进入时 */
extern volatile uint32_t g_tim1_update_exit_cnt; /* UP中断退出时 */

/* 读取TIM1线性计数值：
 * 中央对齐模式下CNT先升后降，这里转换成0~23999的连续计数 */
static inline uint32_t TIM1_GetLinearCnt(void)
{
    uint32_t cnt = TIM1->CNT;
    /* CR1.DIR: 0=向上计数, 1=向下计数 */
    if (TIM1->CR1 & TIM_CR1_DIR) {
        return 23999u - cnt;   /* 下降阶段：线性时间 = 12000 + (11999-CNT) */
    }
    return cnt;                /* 上升阶段 */
}
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

