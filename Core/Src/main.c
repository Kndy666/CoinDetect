/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9833.h"
#include "Retarget.h"
#include "FFTv2.h"

#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC1ConvertedValues[ADC_BUF_SIZE];
float ADC1ConvertedVoltage[ADC_BUF_SIZE];
FFTresult FFT_Res;

bool isADC1Converted = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GetPeeks(float *source, float *res, uint16_t len);
uint32_t GetADCCLK(ADC_HandleTypeDef *hadc);
void SetSimplingTime(ADC_HandleTypeDef *hadc, uint32_t time);
void SetSimpleRate(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim, uint32_t rate);
void VoltageConvert(uint16_t *source, float *res, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_ADC_Stop_DMA(hadc);
    isADC1Converted = true;
  }
}
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  FFT_Init();
  AD9833_Init(wave_sine, 3e4, 0);
  RetargetInit(&huart1);
  SetSimpleRate(&hadc1, &htim2, 75 * 1e3);//75kHz
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1ConvertedValues, ADC_BUF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (isADC1Converted)
    {
      float peeks[2];
      VoltageConvert(ADC1ConvertedValues, ADC1ConvertedVoltage, ADC_BUF_SIZE);
      GetPeeks(ADC1ConvertedVoltage, peeks, ADC_BUF_SIZE);
      FFT_f32(ADC1ConvertedVoltage, ADC_BUF_SIZE, &FFT_Res);
      isADC1Converted = false;
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1ConvertedValues, ADC_BUF_SIZE);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//Get min and max value of array
void GetPeeks(float *source, float *res, uint16_t len)
{
  float min = 4096;
  float max = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    if (source[i] > max)
    {
      max = source[i];
    }
    if (source[i] < min)
    {
      min = source[i];
    }
  }
  res[0] = min;
  res[1] = max;
}
//Get ADCCLK
uint32_t GetADCCLK(ADC_HandleTypeDef *hadc)
{
  uint32_t PCLK2 = HAL_RCC_GetPCLK2Freq();
  switch (hadc->Init.ClockPrescaler)
  {
  case ADC_CLOCK_SYNC_PCLK_DIV2:
  {
    return PCLK2 / 2;
  }
  case ADC_CLOCK_SYNC_PCLK_DIV4:
  {
    return PCLK2 / 4;
  }
  case ADC_CLOCK_SYNC_PCLK_DIV6:
  {
    return PCLK2 / 6;
  }
  case ADC_CLOCK_SYNC_PCLK_DIV8:
  {
    return PCLK2 / 8;
  }
  }
}
//Set ADC sampling time
void SetSimplingTime(ADC_HandleTypeDef *hadc, uint32_t time)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = time;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
/*
Set ADC sampling rate. Automatically set ADC and TIM.
Strategy:
  Sampling an analog signal more slowly reduces surge "parasitic" drain on the analog line as a result of the sampling process itself, 
  which both draws a tiny current from the analog line and takes time. 
  If your analog line has an extremely small current source capability 
  (think: the analog source "recharging" the capacitance of the analog line can only produce a tiny tiny current to recharge that analog line), 
  then it cannot handle a high sample rate sampling on it. This is, again, because the sampling process itself will draw current, 
  making the analog signal sag (distorting it from its true value), and introducing noise to the analog signal. 
  In such cases you are wise to choose a slow sample rate by setting your sample time to a very long value.
*/
void SetSimpleRate(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim, uint32_t rate)
{
  uint32_t ADCCLK = GetADCCLK(hadc);
  uint32_t sTime = ADCCLK / rate - 12;
  if (sTime >= 480)
    SetSimplingTime(hadc, ADC_SAMPLETIME_480CYCLES);
  else if (sTime < 480 && sTime >= 144)
    SetSimplingTime(hadc, ADC_SAMPLETIME_144CYCLES);
  else if (sTime < 144 && sTime >= 112)
    SetSimplingTime(hadc, ADC_SAMPLETIME_112CYCLES);
  else if (sTime < 122 && sTime >= 84)
    SetSimplingTime(hadc, ADC_SAMPLETIME_84CYCLES);
  else if (sTime < 84 && sTime >= 56)
    SetSimplingTime(hadc, ADC_SAMPLETIME_56CYCLES);
  else if (sTime < 56 && sTime >= 28)
    SetSimplingTime(hadc, ADC_SAMPLETIME_28CYCLES);
  else if (sTime < 28 && sTime >= 15)
    SetSimplingTime(hadc, ADC_SAMPLETIME_15CYCLES);
  else
    SetSimplingTime(hadc, ADC_SAMPLETIME_3CYCLES);

  htim->Init.Period = HAL_RCC_GetPCLK2Freq() / rate - 1;
  if (HAL_TIM_Base_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }
}
void VoltageConvert(uint16_t *source, float *res, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    res[i] = source[i] * 3.3 / 4096;
  }
}
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

#ifdef  USE_FULL_ASSERT
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
