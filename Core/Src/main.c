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
#include "foc_api.h"
#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_data.h"
#include "encoder_calc.h"
#include "ifly_test.h"
#include "can_wly.h"
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
extern ControllerStruct controller_eyou;
uint8_t open_loop_mode = 0;  // 0=自动旋转, 1=编码器跟随
int16_t v_d_test = 0;        // d轴电压（Q10格式）
int16_t v_q_test = 512;      // q轴电压（Q10格式，约0.5V）
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

	/* 启动TIM1 PWM输出、中断和CH4编码器预触发 */
	TIM1_PWM_Start();

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

	/* FOC控制器初始化（参考PHU Init_func顺序） */
	printf("Initializing FOC controller...\r\n");

	/* 1. 设置电机参数（必须在Init_foc之前，PID全局变量会被controller_init引用） */
	set_ver_par(90);  // id=90: motor_h7_0426配套，NPP=8

	/* 2. 使能PWM输出（Init_foc 中的 ElecAngleEstimate 需要 PWM 能驱动电机） */
	htim1.Instance->CCER |= 0x0555;

	/* 3. FOC核心初始化（滤波器/斜坡/标志位/FlashData含ElecAngleEstimate/ResetControlData） */
	Init_foc(&controller_eyou);

	/* 4. 将ADC校准的零点偏置同步到FOC控制器 */
	controller_eyou.FlashData.Ia_offset = (uint16_t)g_adc_offset_a;
	controller_eyou.FlashData.Ib_offset = (uint16_t)g_adc_offset_b;

	/* 5. 输出端编码器零位初始化 */
	Encoder_out_data_Reset(controller_eyou.FlashData.MaxPositionLimit,
	                       controller_eyou.FlashData.MinPositionLimit);

	/* 6. 开机辨识: Rs → Ld/Lq → 电流环PI自整定 */
	printf("Starting motor identification...\r\n");

	/* 打印实际 VDC（确认 UDC 宏是否匹配硬件） */
	extern volatile uint32_t g_vdc_raw;
	HAL_Delay(10);
	/* VDC: ADC 16bit, 3.3V 满量程, 分压比需根据硬件确认 */
	float vdc_volt = g_vdc_raw * 3.3f / 65535.0f;
	printf("VDC ADC raw = %lu, voltage (before divider scaling) = %.3f V\r\n",
	       g_vdc_raw, vdc_volt);

//	/* Rs 辨识前先对齐 d 轴，消除上电编码器位置不确定 */
//	alignDAxis(&controller_eyou);

	identifyMotorParamsCached(&controller_eyou);
	//autoTuneCurrentLoopPI(controller_eyou.ident_test.Rs, controller_eyou.ident_test.Ld, controller_eyou.ident_test.Lq);

	/* 7. 复位控制数据（清辨识期间积分器残留） */
	ResetControlData(&controller_eyou);

	/* 8. 辨识完成，设置运行状态 */
	controller_eyou.foc_run = 2;
	printf("FOC initialization done, NPP=%d, foc_run=%d\r\n", NPP, controller_eyou.foc_run);

	/* 启动USART1调试命令接收 */
	USART1_DebugRx_Start();

	/* 万里扬FDCAN协议从站初始化 (fdcan_rx_user 已在 can_wly.c 中覆盖弱符号) */
	can_wly_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	g_foc_openloop_enable = USEFOC_OPEN_TEST;//开环测试
	if(g_foc_openloop_enable == 1) {
		controller_eyou.foc_run = 0;
	}
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* 调试串口命令解析 */
		dbg_cmd_set();

		/* 非阻塞周期调用日志打印 */
		static uint32_t log_tick = 0;
		if (HAL_GetTick() - log_tick >= logPriodMs) {
			log_tick = HAL_GetTick();
			dbg_log_print();
		}

		/* 带宽测试 done 标志轮询 + 结果打印 */
		Test_log_print();

		/* CAN 1ms tick (主动上报模式, 默认关闭) */
		static uint32_t can_tick = 0;
		if (HAL_GetTick() - can_tick >= 1) {
			can_tick = HAL_GetTick();
			can_wly_tick_1ms();
		}

		/* 历史调试打印块已迁到 foc_bsp.c dbg_log_print()，详见 logid 120/130/140/150/151 */
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
