/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

/* FOC电流采样数据（由TIM1 TRGO触发，10kHz采样率） */
typedef struct {
    int32_t  i_a_raw;      /* CUR_A 原始值（有符号，已减零点偏置） */
    int32_t  i_b_raw;      /* CUR_B 原始值 */
    uint32_t sample_count; /* 采样计数 */
    uint32_t tim1_done_cnt;/* 采样完成时TIM1线性计数值 */
} FOC_CurrentSample_t;

extern volatile FOC_CurrentSample_t g_foc_current;

/* ADC偏置零点（启动时自检） */
extern volatile int32_t g_adc_offset_a;
extern volatile int32_t g_adc_offset_b;

/* 启动ADC注入触发链（调用前确保TIM1 TRGO已经配置好） */
void ADC_FOC_Start(void);

/* 启动前校准零点（电机静止状态下采样N次取平均） */
void ADC_CalibrateOffsets(uint16_t n_samples);

/* 规则通道实时数据（VDC/温度，TIM6触发，1kHz采样） */
extern volatile uint32_t g_vdc_raw;         /* VDC平均值（2次采样） */
extern volatile uint32_t g_temp_motor_raw;  /* 电机温度原始值 */
extern volatile uint32_t g_temp_mos_raw;    /* MOS温度原始值 */
extern volatile uint32_t g_reg_callback_count;  /* 规则通道回调计数 */

/* 启动规则通道DMA采样（TIM6 TRGO触发） */
void ADC_Regular_Start(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

