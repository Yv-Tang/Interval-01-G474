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
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "controller.h"
#include "inverter.h"
//#include "oled_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 实际值计算公式 signal = (ADC * ADC_resolution - ADC_CHx_bias) * ADC_CHx_ratio
#define ADC_resolution 3.3/4096	// 每位对应电压
#define ADC_CH1_ratio 37.7931f	// ADC电压每伏对应实际电压
#define ADC_CH1_bias 1.6766f		// ADC电压偏置
#define ADC_CH2_ratio 4.01193f
#define ADC_CH2_bias 1.4863f
#define ADC_CH3_ratio 1.0f
#define ADC_CH3_bias 1.68f
#define ADC_CH4_ratio 1.0f
#define ADC_CH4_bias 1.68f
// 交流电压使用1通道
#define AC_U ADC_voltage[0]
// 电感电流使用2通道
#define L_I ADC_voltage[1]
// 直流电压使用3通道
#define DC_U ADC_voltage[2]
/* 图腾柱拓扑
 *					----------------------Vdc+
 *					|		|		|
 *					Q1		Q3		|
 * Vac+ ----UUU-----|		|		|
 * 					|		|		=
 * Vac- ----UUU-------------|		|
 * 					Q2		Q4		|
 * 					|		|		|
 * 					----------------------Vdc-
 * Q1Q2慢管，Q3Q4快管
 * Q1Q2连接TIM1_CH1_CH1N
 * 		Vac>0.1V(0.1V作为软开关),duty1=1,
 * Q3Q4连接TIM1_CH2_CH2N
 *
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// DC->AC 电压电流环PI结构体
extern PID_Struct ac_U_PI;
extern PID_Struct ac_I_PI;
extern PID_Struct ac_V_PI;

// 采样ADC值
uint16_t ADC_value[4] = {0}; // {ac_U_value, ac_I_value, dc_U_value, dc_I_value}

// 转换电压值
float ADC_voltage[4] = {0}; // {ac_U_value, ac_I_value, dc_U_value, dc_I_value}

// 中断标志位
uint16_t adc_conv_complete_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Inverter_Init();

  // 开启定时器TIM1中断模式
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  // ADC校准
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  // 开启DMA自动采样
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_value, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(adc_conv_complete_flag == 1)
      {
          adc_conv_complete_flag = 0;

    	  // 双环 电压环 PI + 电流环 PR
          Inverter_Loop_With_ALL(AC_U, L_I, DC_U);

    	  // 单环 电压环 PI
    	  // Inverter_Loop_With_Voltage(AC_U);

    	  // 开环
    	  // Inverter_Loop_With_None();

      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
