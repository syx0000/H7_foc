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

/* DWT时间戳（480MHz周期计数器，32位，约9秒回绕，1 cycle = 1/480 us ≈ 2.08ns） */
extern volatile uint32_t g_tim1_cc4_cycles;         /* CC4中断进入时DWT周期 */
extern volatile uint32_t g_tim1_cc4_exit_cycles;    /* CC4中断退出时DWT周期 */
extern volatile uint32_t g_tim1_enc_done_cycles;    /* 编码器读取完成时DWT周期 */

/* ADC注入中断时间戳 */
extern volatile uint32_t g_adc_isr_in_cycles;       /* ADC ISR进入时DWT周期 */
extern volatile uint32_t g_adc_isr_out_cycles;      /* ADC ISR退出时DWT周期 */
extern volatile uint32_t g_adc_isr_cycles;      /* 上一次ADC ISR耗时（DWT周期，480MHz） */
extern volatile uint32_t g_adc_isr_cycles_max;  /* ADC ISR最大耗时 */

/* ADC ISR 分段耗时（DWT 周期）
 * 分段: read=电流raw读取 / enc=编码器电角度+位置 / pos=位置环 / vel=速度环 / cur=电流环+SVPWM
 * pos/vel 只在实际运行的那一拍更新（受 POSITION_CALCULATE_DIV / VELOCETY_CALCULATE_DIV 分频） */
extern volatile uint32_t g_adc_isr_t_read;      /* 电流raw读取段耗时 */
extern volatile uint32_t g_adc_isr_t_enc;       /* 编码器计算段耗时 */
extern volatile uint32_t g_adc_isr_t_pos;       /* 位置环耗时（上次运行） */
extern volatile uint32_t g_adc_isr_t_vel;       /* 速度环耗时（上次运行） */
extern volatile uint32_t g_adc_isr_t_cur;       /* 电流环+SVPWM耗时 */
extern volatile uint32_t g_adc_isr_t_read_max;
extern volatile uint32_t g_adc_isr_t_enc_max;
extern volatile uint32_t g_adc_isr_t_pos_max;
extern volatile uint32_t g_adc_isr_t_vel_max;
extern volatile uint32_t g_adc_isr_t_cur_max;

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

