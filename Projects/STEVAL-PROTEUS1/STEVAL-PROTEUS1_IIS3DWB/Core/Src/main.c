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
#include "iis3dwb_reg.h"
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
  #define AXIS                  3                     /* Axis should be defined between 1 and 3 */
#endif
#ifndef SAMPLES
  #define SAMPLES               256                   /* Should be between 8 & 4096 */
#endif
#define FIFO_WATERMARK          256                   /* Definition of the FIFO watermark to the half of FIFO full size */
#define FIFO_FULL_SIZE          512                   /* FIFO full size */
/************************************************************ Sensor type part ************************************************************/
/************************************************************ Sensors configuration part ************************************************************/
#ifndef ACCELEROMETER_ODR
  #define ACCELEROMETER_ODR   IIS3DWB_XL_ODR_26k7Hz   /* Only one ODR is available with this sensor IIS3DWB_XL_ODR_26k7Hz */
#endif
#ifndef ACCELEROMETER_FS
  #define ACCELEROMETER_FS    IIS3DWB_8g              /* Should be between IIS3DWB_2g and IIS3DWB_16g */
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
  #define NEAI_MODE             0                     /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
#if (NEAI_MODE == 1)
  #ifndef NEAI_LEARN_NB
    #define NEAI_LEARN_NB       20                    /* Number of buffers to be learn by the NEAI library */
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
uint8_t neai_similarity = 0, neai_state = 0;
volatile uint8_t drdy = 0;
uint16_t data_left = (uint16_t) SAMPLES, neai_buffer_ptr = 0, num = 0, neai_cnt = 0;
float neai_time = 0.0;
static float neai_buffer[AXIS * SAMPLES] = {0.0};
static iis3dwb_fifo_out_raw_t fifo_data[FIFO_FULL_SIZE];
stmdev_ctx_t dev_ctx;
iis3dwb_fifo_status_t fifo_status;
iis3dwb_pin_int1_route_t int1_route;

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
static void iis3dwb_initialize(void);
static void iis3dwb_get_buffer_from_fifo(uint16_t nb);
static float iis3dwb_convert_accel_data_to_mg(int16_t accel_raw_data);

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
  /* Initialize mems driver interface */
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
  iis3dwb_initialize();
  if (NEAI_MODE) {
    neai_state = neai_anomalydetection_init();
    printf("Initialize NEAI library. NEAI init return: %d.\n",  neai_state);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (drdy) {
      /* Reset data ready variable */
      drdy = 0;
      /* Read watermark flag */
      iis3dwb_fifo_status_get(&dev_ctx, &fifo_status);
      if (fifo_status.fifo_th) {
        num = fifo_status.fifo_level;
        if (data_left > num) {
          data_left -= num;
        }
        else {
          num = data_left;
          data_left = 0;
        }
        iis3dwb_get_buffer_from_fifo(num);
        if (data_left == 0) {
          /* Stop FIFO mode, set it in BYPASS mode */
          iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_BYPASS_MODE);
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
          /* Clear internal FIFO */
          iis3dwb_fifo_out_multi_raw_get(&dev_ctx, fifo_data, FIFO_FULL_SIZE);
          /* Clear neai_buffer */
          memset(neai_buffer, 0x00, AXIS * SAMPLES * sizeof(float));
          /* Enable FIFO continuous mode */
          iis3dwb_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
          iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_STREAM_MODE);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM330DHCX_CS_GPIO_Port, ISM330DHCX_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IIS3DWB_CS_Pin ISM330DHCX_CS_Pin */
  GPIO_InitStruct.Pin = IIS3DWB_CS_Pin|ISM330DHCX_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin = IIS3DWB_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IIS3DWB_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
  case IIS3DWB_INT1_Pin:
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
  HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET);
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
  HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/**
  * @brief  Initialize IIS3DWB.
  * @param  None
  * @retval None
  */
static void iis3dwb_initialize()
{
  uint8_t rst;
  uint8_t whoamI;

  /* Check device ID */
  do {
    /* Wait sensor boot time */
    HAL_Delay(10);
    iis3dwb_device_id_get(&dev_ctx, &whoamI);
  } while (whoamI != IIS3DWB_ID);

  /* Restore default configuration */
  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis3dwb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Set full scale */
  iis3dwb_xl_full_scale_set(&dev_ctx, ACCELEROMETER_FS);
  /* Set Output Data Rate */
  iis3dwb_xl_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);

  /*
  * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
  * stored in FIFO) to FIFO_WATERMARK samples
  */
  iis3dwb_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch XL ODR to 26.7 kHz */
  iis3dwb_fifo_xl_batch_set(&dev_ctx, IIS3DWB_XL_BATCHED_AT_26k7Hz);

  /* Set interrupt on FIFO threshold on INT1 pin */
  int1_route.fifo_th = 1;
  iis3dwb_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_STREAM_MODE);
}

/*
 * @brief  Get accelerometer data from IIS3DWB using the internal FIFO buffer
 *
 * @param  No
 *
 * @return No
 *
 */
static void iis3dwb_get_buffer_from_fifo(uint16_t nb)
{
  /* read out all FIFO entries in a single read */
  iis3dwb_fifo_out_multi_raw_get(&dev_ctx, fifo_data, nb);

  for (uint16_t i = 0; i < nb; i++) {
    if (fifo_data[i].tag >> 3 == IIS3DWB_XL_TAG) {
      /* Fill neai_buffer with accelerometer data */
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = iis3dwb_convert_accel_data_to_mg((int16_t) fifo_data[i].data[(2 * j) + 1] << 8 | fifo_data[i].data[(2 * j)]);
      }
    }
    else {
      /* Bad tag sensor */
      printf("Bad sensor tag: %d.\n", fifo_data[i].tag >> 3);
      Error_Handler();
    }
  }
  neai_buffer_ptr += nb;
  if (neai_buffer_ptr == SAMPLES) {
    neai_buffer_ptr = 0;
  }
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
static float iis3dwb_convert_accel_data_to_mg(int16_t accel_raw_data)
{
  float accel_data_mg = 0.0;
  switch (ACCELEROMETER_FS)
  {
  case IIS3DWB_2g:
    accel_data_mg = iis3dwb_from_fs2g_to_mg(accel_raw_data);
    break;
  case IIS3DWB_4g:
    accel_data_mg = iis3dwb_from_fs4g_to_mg(accel_raw_data);
    break;
  case IIS3DWB_8g:
    accel_data_mg = iis3dwb_from_fs8g_to_mg(accel_raw_data);
    break;
  case IIS3DWB_16g:
    accel_data_mg = iis3dwb_from_fs16g_to_mg(accel_raw_data);
    break;
  default:
    accel_data_mg = 0.0;
    break;
  }
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
      printf("Error.\n");
      HAL_Delay(1000);
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
