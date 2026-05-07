/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();  // 初始化DWT周期计数器
	DPT_Encoder_Init(&huart2);

	HAL_Delay(500);
	printf("LT H7 foc start\r\n");
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_SET);
	HAL_Delay(500);
	TIM1->BDTR |= 0xC000;//Enable main output and Automatic output enable(AOE)
	TIM1->DIER |= TIM_DIER_UIE;           // enable update interrupt
	TIM1->DIER |= TIM_DIER_CC4IE;         // enable CC4 interrupt for encoder pre-trigger
	TIM1->CR1  |= TIM_CR1_UDIS;						//????????
	TIM1->CR1  |= 0x0001;//Enable Counter
	TIM1->CR1 &= ~TIM_CR1_CEN;  // ?????
	
	TIM1->CNT = 0;              // ?????
	TIM1->CR1 &= ~TIM_CR1_UDIS; // ?? UDIS,??????
	TIM1->CR1 |= TIM_CR1_CEN;   // ???????
	// ?? TIM1 CH4 ????????(TIMING ??? CC4IF ?????,????????)
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	/* 启动ADC注入采样链路（TIM1 TRGO→ADC1/ADC2同步） */
	ADC_FOC_Start();

	/* 启动ADC规则通道DMA采样（TIM6 TRGO→VDC/温度） */
	HAL_TIM_Base_Start(&htim6);
	ADC_Regular_Start();

	/* 电流零点校准（电机静止，IGBT未输出时） */
	printf("Calibrating ADC offsets...\r\n");
	HAL_Delay(100);  // 等待ADC稳定
	ADC_CalibrateOffsets(1024);
	printf("ADC calibration done: Off_a=%ld Off_b=%ld\r\n",
	       (int32_t)g_adc_offset_a, (int32_t)g_adc_offset_b);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	htim1.Instance->CCER |= 0x0555;//Enable channel output
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 6000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 6000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 6000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		/* 编码器由TIM1 CC4中断触发读取 */
//		static uint32_t stat_tick = 0;
//		uint32_t now = HAL_GetTick();
//		if (now - stat_tick >= 1000) {  /* 每1秒打印一次统计 */
//			uint32_t elapsed_ms = now - stat_tick;
//			stat_tick = now;

//			uint32_t trig, succ, skip, last_us, min_us, max_us;
//			DPT_GetAndResetStats(&trig, &succ, &skip, &last_us, &min_us, &max_us);

//			DPT_Angles angles;
//			DPT_GetLatestAngles(&angles);

//			/* 触发频率 = trig / 时间(秒) */
//			uint32_t freq_hz = (trig * 1000) / elapsed_ms;

//			printf("Inner: %.2f° Outer: %.2f° Status: 0x%02X | "
//			       "Trig: %lu Hz, Succ: %lu, Skip: %lu | "
//			       "T(us): last=%lu min=%lu max=%lu\r\n",
//				angles.inner_deg, angles.outer_deg, angles.status,
//				freq_hz, succ, skip,
//				last_us, min_us, max_us);
//		}

//		/* 每1秒打印时序测试 */
//		static uint32_t ts_tick = 0;
//		if (HAL_GetTick() - ts_tick >= 1000) {
//			ts_tick = HAL_GetTick();

//			/* 快照5个时间戳（TIM1 CNT单位，1us = 240 counts） */
//			uint32_t t_cc4_in   = g_tim1_cc4_cnt;
//			uint32_t t_cc4_out  = g_tim1_cc4_exit_cnt;
//			uint32_t t_enc_done = g_tim1_enc_done_cnt;
//			uint32_t t_up_in    = g_tim1_update_cnt;
//			uint32_t t_up_out   = g_tim1_update_exit_cnt;

//			uint32_t trig, succ, skip, last_us, min_us, max_us;
//			DPT_GetAndResetStats(&trig, &succ, &skip, &last_us, &min_us, &max_us);

//			DPT_Angles angles;
//			DPT_GetLatestAngles(&angles);

//			printf("Inner:%.2f Outer:%.2f Sta:0x%02X | Trig:%luHz Succ:%lu Skip:%lu | "
//			       "T(us):last=%lu min=%lu max=%lu | "
//			       "Timing(us): CC4_in=%lu.%lu CC4_out=%lu.%lu Enc_done=%lu.%lu UP_in=%lu.%lu UP_out=%lu.%lu\r\n",
//				angles.inner_deg, angles.outer_deg, angles.status,
//				trig, succ, skip,
//				last_us, min_us, max_us,
//				t_cc4_in/240,   (t_cc4_in%240)*10/240,
//				t_cc4_out/240,  (t_cc4_out%240)*10/240,
//				t_enc_done/240, (t_enc_done%240)*10/240,
//				t_up_in/240,    (t_up_in%240)*10/240,
//				t_up_out/240,   (t_up_out%240)*10/240);
//		}

//		/* ADC注入采样检测（TIM1 TRGO=10kHz，中央对齐每个完整周期触发1次） */
//		static uint32_t adc_tick = 0;
//		static uint32_t last_sample_count = 0;
//		if (HAL_GetTick() - adc_tick >= 1000) {
//			uint32_t elapsed_ms = HAL_GetTick() - adc_tick;
//			adc_tick = HAL_GetTick();

//			/* 快照采样数据 */
//			uint32_t cnt_now   = g_foc_current.sample_count;
//			int32_t  ia        = g_foc_current.i_a_raw;
//			int32_t  ib        = g_foc_current.i_b_raw;
//			uint32_t t_done    = g_foc_current.tim1_done_cnt;
//			int32_t  off_a     = g_adc_offset_a;
//			int32_t  off_b     = g_adc_offset_b;

//			uint32_t delta     = cnt_now - last_sample_count;
//			last_sample_count  = cnt_now;
//			uint32_t rate_hz   = (delta * 1000) / elapsed_ms;

//			printf("ADC: Rate=%luHz Cnt=%lu | Ia=%ld Ib=%ld | Off_a=%ld Off_b=%ld | t_done=%lu.%luus\r\n",
//				rate_hz, cnt_now, ia, ib, off_a, off_b,
//				t_done/240, (t_done%240)*10/240);
//		}

		/* ADC监控（注入10kHz + 规则1kHz） */
		static uint32_t adc_mon_tick = 0;
		if (HAL_GetTick() - adc_mon_tick >= 1000) {
			adc_mon_tick = HAL_GetTick();

			/* 注入通道（电流） */
			int32_t ia = g_foc_current.i_a_raw;
			int32_t ib = g_foc_current.i_b_raw;
			uint32_t inj_cnt = g_foc_current.sample_count;

			/* 规则通道（VDC/温度） */
			uint32_t vdc = g_vdc_raw;
			uint32_t t_motor = g_temp_motor_raw;
			uint32_t t_mos = g_temp_mos_raw;

			/* 调试：检查ADC1规则通道状态 */
			uint32_t adc1_state = hadc1.State;
			uint32_t dma_state = hadc1.DMA_Handle->State;
			uint32_t cb_cnt = g_reg_callback_count;

			printf("ADC_Inj(10kHz): Ia=%ld Ib=%ld Cnt=%lu | ADC_Reg(1kHz): VDC=%lu T_motor=%lu T_mos=%lu | State:ADC=0x%lX DMA=0x%lX CB=%lu\r\n",
				ia, ib, inj_cnt, vdc, t_motor, t_mos, adc1_state, dma_state, cb_cnt);
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
