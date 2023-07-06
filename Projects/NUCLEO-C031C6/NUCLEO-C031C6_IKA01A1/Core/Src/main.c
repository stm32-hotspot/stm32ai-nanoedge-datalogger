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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "NanoEdgeAI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/************************************************************ NEAI algorithm defines begin ************************************************************/
/************************************************************ Global settings part ************************************************************/
#ifndef SAMPLES
  #define SAMPLES           256     /* Should be between 16 & 256 */
#endif
#ifndef CURRENT_ODR
  #define CURRENT_ODR       1000    /* Should be between 1 & 100 kHz */
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
  #define NEAI_MODE         0       /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
#if (NEAI_MODE == 1)
  #ifndef NEAI_LEARN_NB
    #define NEAI_LEARN_NB   50      /* Number of buffers to be learn by the NEAI library */
  #endif
#endif
/************************************************************ NEAI algorithm defines end ************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t neai_similarity = 0, neai_state = 0;
uint16_t neai_cnt = 0;
float odr_timer = 0.0;
float neai_buffer[SAMPLES] = {0.0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void NEAI_Delay(uint32_t us);
static void get_adc_data(void);
static void convert_adc_data_to_mA(void);
static void print_float(float float_input);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  /* Get the time to wait between each samples in µs */
  odr_timer = 1000000.0 / CURRENT_ODR;
  if (NEAI_MODE) {
    neai_state = neai_anomalydetection_init();
    printf("Initialize NEAI library. NEAI init return: %d.\n",  neai_state);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    get_adc_data();
    convert_adc_data_to_mA();
#if NEAI_MODE
    if (neai_cnt < NEAI_LEARN_NB) {
      neai_cnt++;
      neai_state = neai_anomalydetection_learn(neai_buffer);
      printf("Learn: %d / %d. NEAI learn return: %d.\n", neai_cnt, NEAI_LEARN_NB, neai_state);
    }
    else {
      neai_state = neai_anomalydetection_detect(neai_buffer, &neai_similarity);
      printf("Similarity: %d / 100. NEAI detect return: %d.\n", neai_similarity, neai_state);
    }
#else
    for (uint16_t i = 0; i < SAMPLES; i++) {
      print_float(neai_buffer[i]);
    }
    printf("\n");
#endif
    /* Clean neai buffer */
    memset(neai_buffer, 0x00, SAMPLES * sizeof(float));
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Redirecting stdout to USART1 which is connected on the STLINK port
  * @retval
  * @param
  */
int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart2, &*c, 1, 10);
 return ch;
}

/*
 * @brief  Used to have an output data rate between 1 Hz & 100 kHz
 *
 * @param  us: delay in µs
 *
 * @return No
 *
 */
static void NEAI_Delay(uint32_t us)
{
  /* With a 1 MHz Timer on 16 bits, we can only count 0.065 s <-> 15.25 Hz */
  if (CURRENT_ODR > 15) {
    /* Set the htim1 counter to 0 in order to restart counter */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us - 8);
  }
  /* So for a lower odr we use the standard HAL_Delay which counts in ms */
  else {
    HAL_Delay((uint32_t) (us / 1000));
  }
}

/*
 * @brief  Get the ADC raw values in the neai buffer
 *
 * @param  No
 *
 * @return No
 *
 */
static void get_adc_data ()
{
  for (uint16_t i = 0; i < SAMPLES; i++) {
    HAL_ADC_Start(&hadc1);
    /* Here we call PollForConversion function without delay */
    HAL_ADC_PollForConversion(&hadc1, 0);
    /* Get ADC data in neai_buffer */
    neai_buffer[i] = HAL_ADC_GetValue(&hadc1);
    NEAI_Delay(odr_timer);
  }
}

/*
 * @brief  Convert the ADC raw values to mA
 *
 * @param  No
 *
 * @return No
 *
 */
static void convert_adc_data_to_mA ()
{
  for (uint16_t i = 0; i < SAMPLES; i++) {
    /* Transform raw ADC values in mA thanks to the conversion formula */
    neai_buffer[i] = 805.0 * neai_buffer[i] / 1586.0;
  }
}

/*
 * @brief  Print float data without using newlib-nano in order to save memory
 *
 * @param  float_input: the data to be print
 *
 * @return No
 *
 */
void print_float(float float_input)
{
  uint8_t char_nb = 0;
  char float_string[20];
  int32_t integer_res = 0;

  /* If  input is negative */
  if (float_input < 0.0) {
      float_input = -1.0 * float_input;
      putchar('-');
  }

  integer_res = (int32_t) (1000 * float_input);
  char_nb = snprintf(float_string, 20, "%ld", integer_res);

  if (float_input == 0.0) {
    putchar('0');
    putchar('.');
    putchar('0');
  }
  else if (char_nb <= 3) {
    putchar('0');
    putchar('.');
    for (uint8_t i = 0; i < 3 - char_nb; i++) {
      putchar('0');
    }
    for (uint8_t i = 0; i < char_nb; i++) {
      putchar(float_string[i]);
    }
  }
  else {
    for (uint8_t i = 0; i < char_nb - 3; i++) {
      putchar(float_string[i]);
    }
    putchar('.');
    for (uint8_t i = char_nb - 3; i < char_nb; i++) {
      putchar(float_string[i]);
    }
  }
  putchar(' ');
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
