/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ism330dhcx_reg.h"
#include "cycle_dwt.h"
#include "NanoEdgeAI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/************************************************************ NEAI algorithm defines begin ************************************************************/
/************************************************************ Global settings part ************************************************************/
#ifndef AXIS
  #define AXIS                          3                         /* Axis should be defined between 1 and 3 */
#endif
#ifndef SAMPLES
  #define SAMPLES                       256                       /* Should be between 16 & 4096 */
#endif
#define MAX_FIFO_SIZE                   256                       /* The maximum number of data we can get in the FIFO is 512 but here we define max to 256 for our need */
#define FIFO_FULL                       512                       /* FIFO full size */
/************************************************************ Sensor type part ************************************************************/
#define GYROSCOPE                       0
#define ACCELEROMETER                   1
#ifndef SENSOR_TYPE
  #define SENSOR_TYPE                   ACCELEROMETER             /* Here we define the data type we're going to collect */
#endif
/************************************************************ Sensors configuration part ************************************************************/
#if (SENSOR_TYPE == ACCELEROMETER)
  #ifndef ACCELEROMETER_ODR
    #define ACCELEROMETER_ODR           ISM330DHCX_XL_ODR_1666Hz  /* Shoud be between ISM330DHCX_XL_ODR_12Hz5 and ISM330DHCX_XL_ODR_6667Hz */
  #endif
  #ifndef ACCELEROMETER_FS
    #define ACCELEROMETER_FS            ISM330DHCX_2g             /* Should be between ISM330DHCX_2g and ISM330DHCX_8g */
  #endif
#elif (SENSOR_TYPE == GYROSCOPE)
  #ifndef GYROSCOPE_ODR
    #define GYROSCOPE_ODR               ISM330DHCX_GY_ODR_1666Hz  /* Shoud be between ISM330DHCX_GY_ODR_12Hz5 and ISM330DHCX_GY_ODR_6667Hz */
  #endif
  #ifndef GYROSCOPE_FS
    #define GYROSCOPE_FS                ISM330DHCX_2000dps        /* Should be between ISM330DHCX_125dps and ISM330DHCX_4000dps */
  #endif
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
  #define NEAI_MODE                     0                         /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
#if (NEAI_MODE == 1)
  #ifndef NEAI_LEARN_NB
    #define NEAI_LEARN_NB               20                        /* Number of buffers to be learn by the NEAI library */
  #endif
#endif
/************************************************************ NEAI algorithm defines end ************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static uint8_t whoamI, rst;
uint8_t neai_similarity = 0, neai_state = 0;
volatile uint8_t drdy = 0;
uint16_t data_left = (uint16_t) SAMPLES, number_read = 0, neai_buffer_ptr = 0, neai_cnt = 0;
float neai_time = 0.0;
static float neai_buffer[AXIS * SAMPLES] = {0.0};
stmdev_ctx_t dev_ctx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void ism330dhcx_initialize(void);
static void ism330dhcx_initialize_basics(void);
static void ism330dhcx_initialize_fifo(void);
static void ism330dhcx_get_buffer_from_fifo(uint16_t nb);
static float ism330dhcx_convert_gyro_data_to_mdps(int16_t gyro_raw_data);
static float ism330dhcx_convert_accel_data_to_mg(int16_t accel_raw_data);

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
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hspi1;
  /* USER CODE END 1 */

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
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  KIN1_InitCycleCounter();
  KIN1_EnableCycleCounter();
  ism330dhcx_initialize();
  if (NEAI_MODE) {
    neai_state = neai_anomalydetection_init();
    printf("Initialize NEAI library. NEAI init return: %d.\n",  neai_state);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t wtm_flag = 0, status2 = 0;
    uint16_t num = 0;
    if (drdy) {
      /* Reset data ready contition */
      drdy = 0;
      ism330dhcx_read_reg(&dev_ctx, ISM330DHCX_FIFO_STATUS2, &status2, 1);
      wtm_flag = status2 >> 7;
      if (wtm_flag) {
        ism330dhcx_fifo_data_level_get(&dev_ctx, &num);
        if (data_left < num) {
          num = data_left;
        }
        ism330dhcx_get_buffer_from_fifo(num);
        data_left -= num;
        number_read += num;
        if (data_left == 0) {
          HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
          ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);
#if NEAI_MODE
          uint32_t cycles_cnt = 0;
          if (neai_cnt < NEAI_LEARN_NB) {
            neai_cnt++;
            KIN1_ResetCycleCounter();
            neai_state = neai_anomalydetection_learn(neai_buffer);
            cycles_cnt = KIN1_GetCycleCounter();
            neai_time = (cycles_cnt * 1000000.0) / HAL_RCC_GetSysClockFreq();
            printf("Learn: %d / %d. NEAI learn return: %d. Cycles counter: %ld = %.1f µs at %ld Hz.\n",
                  neai_cnt, NEAI_LEARN_NB, neai_state, cycles_cnt, neai_time, HAL_RCC_GetSysClockFreq());
          }
          else {
            KIN1_ResetCycleCounter();
            neai_state = neai_anomalydetection_detect(neai_buffer, &neai_similarity);
            cycles_cnt = KIN1_GetCycleCounter();
            neai_time = (cycles_cnt * 1000000.0) / HAL_RCC_GetSysClockFreq();
            if (neai_similarity < 90) {
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
            }
            else {
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            }
            printf("Similarity: %d / 100. NEAI detect return: %d. Cycles counter: %ld = %.1f µs at %ld Hz.\n",
                  neai_similarity, neai_state, cycles_cnt, neai_time, HAL_RCC_GetSysClockFreq());
          }
#else
          for (uint16_t i = 0; i < AXIS * SAMPLES; i++) {
            printf("%.3f ", neai_buffer[i]);
          }
          printf("\n");
#endif
          data_left = (uint16_t) SAMPLES;
          number_read = 0;
          memset(neai_buffer, 0x00, AXIS * SAMPLES * sizeof(float));
          if (SAMPLES <= MAX_FIFO_SIZE) {
            ism330dhcx_fifo_watermark_set(&dev_ctx, (uint16_t) SAMPLES);
          }
          else {
            ism330dhcx_fifo_watermark_set(&dev_ctx, (uint16_t) MAX_FIFO_SIZE);
          }
          ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_STREAM_MODE);
        }
        else if (data_left < MAX_FIFO_SIZE) {
          ism330dhcx_fifo_watermark_set(&dev_ctx, data_left);
        }
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ISM330DHCX_INT1_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ISM330DHCX_CS_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ISM330DHCX_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
 HAL_UART_Transmit(&hlpuart1, &*c, 1, 10);
 return ch;
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
  case ISM330DHCX_INT1_Pin:
    drdy = 1;
    break;
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Initialize ISM330DHCX sensor interface
 *
 * @param  No
 *
 * @return No
 *
 */
static void ism330dhcx_initialize()
{
  ism330dhcx_initialize_basics();
#if (SENSOR_TYPE == ACCELEROMETER)
  /* Accelelerometer configuration */
  ism330dhcx_xl_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
  ism330dhcx_xl_full_scale_set(&dev_ctx, ACCELEROMETER_FS);
#elif (SENSOR_TYPE == GYROSCOPE)
  /* Gyroscope configuration */
  ism330dhcx_gy_data_rate_set(&dev_ctx, GYROSCOPE_ODR);
  ism330dhcx_gy_full_scale_set(&dev_ctx, GYROSCOPE_FS);
#endif
  ism330dhcx_initialize_fifo();
}

/*
 * @brief  Initialize ISM330DHCX basics
 *
 * @param  No
 *
 * @return No
 *
 */
static void ism330dhcx_initialize_basics()
{
  /* Check device ID */
  whoamI = 0;

  do {
    /* Wait sensor boot time */
    HAL_Delay(10);
    ism330dhcx_device_id_get(&dev_ctx, &whoamI);
  } while (whoamI != ISM330DHCX_ID);

  /* Restore default configuration */
  ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    ism330dhcx_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start device configuration. */
  ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
}

/*
 * @brief  Initialize ISM330DHCX internal FIFO
 *
 * @param  No
 *
 * @return No
 *
 */
static void ism330dhcx_initialize_fifo()
{
#if (SENSOR_TYPE == ACCELEROMETER)
  /* Batch odr config */
  ism330dhcx_fifo_xl_batch_set(&dev_ctx, ACCELEROMETER_ODR);
  ism330dhcx_fifo_gy_batch_set(&dev_ctx, 0);
#elif (SENSOR_TYPE == GYROSCOPE)
  /* Batch odr config */
  ism330dhcx_fifo_xl_batch_set(&dev_ctx, 0);
  ism330dhcx_fifo_gy_batch_set(&dev_ctx, GYROSCOPE_ODR);
#endif
  /* FIFO MODE */
  ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);
  HAL_Delay(10);
  ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_STREAM_MODE);
  /* Watermark config */
  if (SAMPLES <= MAX_FIFO_SIZE) {
    ism330dhcx_fifo_watermark_set(&dev_ctx, (uint16_t) SAMPLES);
  }
  else {
    ism330dhcx_fifo_watermark_set(&dev_ctx, (uint16_t) MAX_FIFO_SIZE);
  }
  uint8_t ctrl = 0x08;
  ism330dhcx_write_reg(&dev_ctx, ISM330DHCX_INT1_CTRL, (uint8_t *) &ctrl, 1);
}

/*
 * @brief  Get accelerometer or gyroscope data from
 *         ISM330DHCX using the internal FIFO buffer
 *
 * @param  No
 *
 * @return No
 *
 */
static void ism330dhcx_get_buffer_from_fifo(uint16_t nb)
{
  static int16_t dummy[3];
  ism330dhcx_fifo_tag_t reg_tag;
  for (uint16_t i = 0; i < nb; i++) {
    /* Read FIFO tag */
    ism330dhcx_fifo_sensor_tag_get(&dev_ctx, &reg_tag);
    if(reg_tag == ISM330DHCX_XL_NC_TAG) {
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_fifo_out_raw_get(&dev_ctx, (uint8_t *) data_raw_acceleration);
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = ism330dhcx_convert_accel_data_to_mg(data_raw_acceleration[j]);
      }
    }
    else if(reg_tag == ISM330DHCX_GYRO_NC_TAG) {
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_fifo_out_raw_get(&dev_ctx, (uint8_t *) data_raw_angular_rate);
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = ism330dhcx_convert_gyro_data_to_mdps(data_raw_angular_rate[j]);
      }
    }
    else {
      /* Flush unused samples */
      printf("Bad sensor tag: %d.\n", reg_tag);
      memset(dummy, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_fifo_out_raw_get(&dev_ctx, (uint8_t *) dummy);
    }
  }
  neai_buffer_ptr += nb;
  if (neai_buffer_ptr == SAMPLES) {
    neai_buffer_ptr = 0;
  }
}

/*
 * @brief  Convert gyroscope raw data to milli degrees per second (mdps)
 *
 * @param  gyro_raw_data: which is gyroscope raw data
 *                        depending on the full scale selected
 *
 * @return The converted value in milli degrees per second (mdps)
 *
 */
static float ism330dhcx_convert_gyro_data_to_mdps(int16_t gyro_raw_data)
{
  float gyro_data_mdps = 0.0;
#if (SENSOR_TYPE == GYROSCOPE)
  switch (GYROSCOPE_FS)
  {
  case ISM330DHCX_125dps:
    gyro_data_mdps = ism330dhcx_from_fs125dps_to_mdps(gyro_raw_data);
    break;
  case ISM330DHCX_250dps:
    gyro_data_mdps = ism330dhcx_from_fs250dps_to_mdps(gyro_raw_data);
    break;
  case ISM330DHCX_500dps:
    gyro_data_mdps = ism330dhcx_from_fs500dps_to_mdps(gyro_raw_data);
    break;
  case ISM330DHCX_1000dps:
    gyro_data_mdps = ism330dhcx_from_fs1000dps_to_mdps(gyro_raw_data);
    break;
  case ISM330DHCX_2000dps:
    gyro_data_mdps = ism330dhcx_from_fs2000dps_to_mdps(gyro_raw_data);
    break;
  case ISM330DHCX_4000dps:
    gyro_data_mdps = ism330dhcx_from_fs4000dps_to_mdps(gyro_raw_data);
    break;
  default:
    gyro_data_mdps = 0.0;
    break;
  }
#endif
  return gyro_data_mdps;
}

/*
 * @brief  Convert accelerometer raw data to milli-G' (mg)
 *
 * @param  accel_raw_data: which is accelerometer raw data
 *                        depending on the full scale selected
 *
 * @return The converted value in milli-G' (mg)
 *
 */
static float ism330dhcx_convert_accel_data_to_mg(int16_t accel_raw_data)
{
  float accel_data_mg = 0.0;
#if (SENSOR_TYPE == ACCELEROMETER)
  switch (ACCELEROMETER_FS)
  {
  case ISM330DHCX_2g:
    accel_data_mg = ism330dhcx_from_fs2g_to_mg(accel_raw_data);
    break;
  case ISM330DHCX_4g:
    accel_data_mg = ism330dhcx_from_fs4g_to_mg(accel_raw_data);
    break;
  case ISM330DHCX_8g:
    accel_data_mg = ism330dhcx_from_fs8g_to_mg(accel_raw_data);
    break;
  case ISM330DHCX_16g:
    accel_data_mg = ism330dhcx_from_fs16g_to_mg(accel_raw_data);
    break;
  default:
    accel_data_mg = 0.0;
    break;
  }
#endif
  return accel_data_mg;
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
