/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include "foc_api.h"
#include "encoder_calc.h"
#include "foc_current_loop.h"

/* FOC开环测试使能标志 */
volatile uint8_t g_foc_openloop_enable = 0;

extern ControllerStruct controller_eyou;
extern uint8_t open_loop_mode;
extern int16_t v_d_test;
extern int16_t v_q_test;

/* 规则通道DMA缓冲区（双ADC同步模式，VDC采2次求平均） */
static uint32_t adc_reg_buffer[2];  /* [0]=VDC+TEMP_MOTOR, [1]=VDC+TEMP_MOS */

/* 规则通道实时数据 */
volatile uint32_t g_vdc_raw = 0;         /* VDC平均值（2次采样） */
volatile uint32_t g_temp_motor_raw = 0;  /* 电机温度原始值 */
volatile uint32_t g_temp_mos_raw = 0;    /* MOS温度原始值 */
volatile uint32_t g_reg_callback_count = 0;  /* 规则通道回调计数（调试用） */

/* FOC电流采样数据 */
volatile FOC_CurrentSample_t g_foc_current = {0};
volatile int32_t g_adc_offset_a = 0;
volatile int32_t g_adc_offset_b = 0;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetSignedSaturation = DISABLE;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = 8;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = 8;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc2);

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetSignedSaturation = DISABLE;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = 8;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    PA7     ------> ADC1_INP7
    PC5     ------> ADC1_INP8
    */
    GPIO_InitStruct.Pin = VDC_Pin|CUR_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CUR_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CUR_C_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream6;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC4     ------> ADC2_INP4
    PB0     ------> ADC2_INP9
    PB1     ------> ADC2_INP5
    */
    GPIO_InitStruct.Pin = CUR_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CUR_B_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TEMP_MOTOR_Pin|TEMP_MOS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Stream2;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    PA7     ------> ADC1_INP7
    PC5     ------> ADC1_INP8
    */
    HAL_GPIO_DeInit(GPIOA, VDC_Pin|CUR_A_Pin);

    HAL_GPIO_DeInit(CUR_C_GPIO_Port, CUR_C_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC4     ------> ADC2_INP4
    PB0     ------> ADC2_INP9
    PB1     ------> ADC2_INP5
    */
    HAL_GPIO_DeInit(CUR_B_GPIO_Port, CUR_B_Pin);

    HAL_GPIO_DeInit(GPIOB, TEMP_MOTOR_Pin|TEMP_MOS_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  启动ADC注入触发链（TIM1 TRGO → ADC1/ADC2 同步注入）
  * @note   调用前请确保 TIM1 已初始化并启动，TRGO源=Update事件
  *         双ADC regsimult+injecsimult 模式下，只需启动主机ADC1，ADC2会跟随
  */
void ADC_FOC_Start(void)
{
    /* ADC硬件校准（单端模式） */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    /* 注入通道：ADC2必须先启动（从机），再启动ADC1（主机） */
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);  /* 主机开中断，JEOC时触发回调 */
}

/**
  * @brief  启动ADC规则通道DMA采样（VDC/温度，TIM6触发）
  * @note   双ADC同步模式，只需启动主机ADC1，数据包含ADC1+ADC2
  *         缓冲区格式：[0]=VDC+TEMP_MOTOR, [1]=VDC+TEMP_MOS（VDC采2次求平均）
  */
void ADC_Regular_Start(void)
{
    /* 双ADC同步模式，启动主机ADC1的DMA */
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_reg_buffer, 2);
}

/**
  * @brief  规则通道DMA传输完成回调（双ADC同步模式）
  * @note   H7 HAL中 MultiMode Regular DMA 完成只走 HAL_ADC_ConvCpltCallback，
  *         不存在 HAL_ADCEx_MultiModeConvCpltCallback。
  *         CDR寄存器：低16位=ADC1数据，高16位=ADC2数据。
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        g_reg_callback_count++;

        uint16_t vdc1       = adc_reg_buffer[0] & 0xFFFF;   /* ADC1 Rank1: VDC */
        uint16_t temp_motor = adc_reg_buffer[0] >> 16;      /* ADC2 Rank1: TEMP_MOTOR */
        uint16_t vdc2       = adc_reg_buffer[1] & 0xFFFF;   /* ADC1 Rank2: VDC */
        uint16_t temp_mos   = adc_reg_buffer[1] >> 16;      /* ADC2 Rank2: TEMP_MOS */

        g_vdc_raw        = (vdc1 + vdc2) / 2;
        g_temp_motor_raw = temp_motor;
        g_temp_mos_raw   = temp_mos;
    }
}

/**
  * @brief  电流零点校准（电机必须静止，IGBT不输出）
  * @param  n_samples 采样次数，建议1024
  */
void ADC_CalibrateOffsets(uint16_t n_samples)
{
    int64_t sum_a = 0, sum_b = 0;
    uint32_t start = g_foc_current.sample_count;

    while ((g_foc_current.sample_count - start) < n_samples) {
        /* 等待采样完成 */
    }

    /* 读取累计的原始值前，暂时清零偏置 */
    int32_t saved_a = g_adc_offset_a;
    int32_t saved_b = g_adc_offset_b;
    g_adc_offset_a = 0;
    g_adc_offset_b = 0;

    start = g_foc_current.sample_count;
    uint32_t count = 0;
    while (count < n_samples) {
        if (g_foc_current.sample_count != start) {
            sum_a += g_foc_current.i_a_raw;
            sum_b += g_foc_current.i_b_raw;
            start = g_foc_current.sample_count;
            count++;
        }
    }

    g_adc_offset_a = (int32_t)(sum_a / n_samples);
    g_adc_offset_b = (int32_t)(sum_b / n_samples);
    (void)saved_a; (void)saved_b;
}

/**
  * @brief  ADC注入转换完成回调（主机ADC1 JEOC触发）
  * @note   在ADC_IRQHandler中调用。整个ISR耗时<1us @480MHz
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        /* 读取同步采样的A/B相电流 */
        int32_t raw_a = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        int32_t raw_b = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        /* g_foc_current 是ADC驱动独立的调试/校准结构，存减偏置后的值，
           供 ADC_CalibrateOffsets / 驱动层监控使用。
           FOC 的电流通路不经过这里 —— FOC 用原始 raw_a/raw_b 写入 controller_eyou.Ia_raw，
           由 phase_current_sample() 内部减 FlashData.Ia_offset（只会减一次）。 */
        g_foc_current.i_a_raw = raw_a - g_adc_offset_a;
        g_foc_current.i_b_raw = raw_b - g_adc_offset_b;
        g_foc_current.tim1_done_cnt = TIM1_GetLinearCnt();
        g_foc_current.sample_count++;

        /* 编码器计算（10kHz，读取DPT最新异步缓冲数据） */
        Encoder_data_Calculate(&controller_eyou, 10000);
        Encoder_out_data_Calculate(&controller_eyou, 10000);

        /* 相电流采样处理（编码器计算后，FOC运算前）
           参考PHU/H7架构：单独一步，把raw_a/raw_b转成I_a/I_b/I_c */
        controller_eyou.Ia_raw = (uint16_t)raw_a;
        controller_eyou.Ib_raw = (uint16_t)raw_b;
        phase_current_sample(&controller_eyou);

        /* FOC开环测试（校准完成后使能） */
        if (g_foc_openloop_enable) {
            FocOpenTest(&controller_eyou, open_loop_mode, v_d_test, v_q_test,
                        (uint16_t)raw_a, (uint16_t)raw_b);
        }
    }
}

/* USER CODE END 1 */

