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
#include "lsm6dso_reg.h"
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
  #define AXIS                          3                       /* Axis should be defined between 1 and 3 */
#endif
#ifndef SAMPLES
  #define SAMPLES                       128                     /* Should be between 16 & 4096 */
#endif
#define MAX_FIFO_SIZE                   256                     /* The maximum number of data we can get in the FIFO is 512 but here we define max to 256 for our need */
#define FIFO_FULL                       512                     /* FIFO full size */
#define FIFO_WORD                       7                         /* FIFO word size composed of 1 byte which is identification tag & 6 bytes of fixed data */
/************************************************************ Sensor type part ************************************************************/
#define GYROSCOPE                       0
#define ACCELEROMETER                   1
#ifndef SENSOR_TYPE
  #define SENSOR_TYPE                   ACCELEROMETER           /* Here we define the data type we're going to collect */
#endif
/************************************************************ Sensors configuration part ************************************************************/
#if (SENSOR_TYPE == ACCELEROMETER)
  #ifndef ACCELEROMETER_ODR
    #define ACCELEROMETER_ODR           LSM6DSO_XL_ODR_1667Hz   /* Shoud be between LSM6DSO_XL_ODR_12Hz5 and LSM6DSO_XL_ODR_6667Hz */
  #endif
  #ifndef ACCELEROMETER_FS
    #define ACCELEROMETER_FS            LSM6DSO_2g              /* Should be between LSM6DSO_2g and LSM6DSO_16g */
  #endif
#elif (SENSOR_TYPE == GYROSCOPE)
  #ifndef GYROSCOPE_ODR
    #define GYROSCOPE_ODR               LSM6DSO_GY_ODR_1667Hz   /* Shoud be between LSM6DSO_GY_ODR_12Hz5 and LSM6DSO_GY_ODR_6667Hz */
  #endif
  #ifndef GYROSCOPE_FS
    #define GYROSCOPE_FS                LSM6DSO_2000dps         /* Should be between LSM6DSO_125dps and LSM6DSO_2000dps */
  #endif
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
  #define NEAI_MODE                     0                       /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
#if (NEAI_MODE == 1)
  #ifndef NEAI_LEARN_NB
    #define NEAI_LEARN_NB               20                      /* Number of buffers to be learn by the NEAI library */
  #endif
#endif
/************************************************************ NEAI algorithm defines end ************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
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
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void lsm6dso_initialize(void);
static void lsm6dso_initialize_basics(void);
static void lsm6dso_initialize_fifo(void);
static void lsm6dso_get_buffer_from_fifo(uint16_t nb);
static float lsm6dso_convert_gyro_data_to_mdps(int16_t gyro_raw_data);
static float lsm6dso_convert_accel_data_to_mg(int16_t accel_raw_data);
static void iks01a3_i2c_stuck_quirk(void);

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
  dev_ctx.handle = &hi2c1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  iks01a3_i2c_stuck_quirk();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  KIN1_InitCycleCounter();
  KIN1_EnableCycleCounter();
  lsm6dso_initialize();
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
      /* Reset data ready condition */
      drdy = 0;
      lsm6dso_read_reg(&dev_ctx, LSM6DSO_FIFO_STATUS2, &status2, 1);
      wtm_flag = status2 >> 7;
      if (wtm_flag) {
        lsm6dso_fifo_data_level_get(&dev_ctx, &num);
        if (data_left < num) {
          num = data_left;
        }
        lsm6dso_get_buffer_from_fifo(num);
        data_left -= num;
        number_read += num;
        if (data_left == 0) {
          lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_BYPASS_MODE);
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
            lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) SAMPLES);
          }
          else {
            lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) MAX_FIFO_SIZE);
          }
          lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_FIFO_MODE);
        }
        else if (data_left < MAX_FIFO_SIZE) {
          lsm6dso_fifo_watermark_set(&dev_ctx, data_left);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_ACC_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_ACC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_ACC_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Redirecting stdout to USART2 which is connected on the STLINK port
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

/**
  * @brief  EXTI line rising detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
  case GYRO_ACC_INT_Pin:
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
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
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Initialize LSM6DSO sensor interface
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize()
{
  lsm6dso_initialize_basics();
#if (SENSOR_TYPE == ACCELEROMETER)
  /* Accelelerometer configuration */
  lsm6dso_xl_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
  lsm6dso_xl_full_scale_set(&dev_ctx, ACCELEROMETER_FS);
#elif (SENSOR_TYPE == GYROSCOPE)
  /* Gyroscope configuration */
  lsm6dso_gy_data_rate_set(&dev_ctx, GYROSCOPE_ODR);
  lsm6dso_gy_full_scale_set(&dev_ctx, GYROSCOPE_FS);
#endif
  lsm6dso_initialize_fifo();
}

/*
 * @brief  Initialize LSM6DSO basics
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize_basics()
{
  /* Check device ID */
  whoamI = 0;

  do {
    HAL_Delay(20);
    lsm6dso_device_id_get(&dev_ctx, &whoamI);
  } while(whoamI != LSM6DSO_ID);

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
}

/*
 * @brief  Initialize LSM6DSO internal FIFO
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize_fifo()
{
#if (SENSOR_TYPE == ACCELEROMETER)
  /* Batch odr config */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, ACCELEROMETER_ODR);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, 0);
#elif (SENSOR_TYPE == GYROSCOPE)
  /* Batch odr config */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, 0);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, GYROSCOPE_ODR);
#endif
  /* FIFO MODE */
  lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_FIFO_MODE);
  /* Watermark config */
  if (SAMPLES <= MAX_FIFO_SIZE) {
    lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) SAMPLES);
  }
  else {
    lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) MAX_FIFO_SIZE);
  }
  /* Need to enable interrupt pin when wtm is reached */
  uint8_t ctrl = 0x08;
  lsm6dso_write_reg(&dev_ctx, LSM6DSO_INT1_CTRL, (uint8_t *) &ctrl, 1);
}

/*
 * @brief  Get accelerometer or gyroscope data from
 *         LSM6DSO using the internal FIFO buffer
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_get_buffer_from_fifo(uint16_t nb)
{
  uint8_t reg_tag = 0;
  uint8_t buff_tmp[nb * FIFO_WORD];
  /*
   * The data stored in FIFO are accessible from dedicated registers and each FIFO word is composed of 7
   * bytes: one tag byte (FIFO_DATA_OUT_TAG (78h)), in order to identify the sensor, and 6 bytes of fixed data
   * (FIFO_DATA_OUT registers from (79h) to (7Eh))
   * So, here we read the fifo in only one transaction in order to save time
   */
  lsm6dso_read_reg(&dev_ctx, LSM6DSO_FIFO_DATA_OUT_TAG, buff_tmp, nb * FIFO_WORD);
  for (uint16_t i = 0; i < nb; i++) {
    /* According to the datasheet, the TAG_SENSOR is the 5 MSB of the FIFO_DATA_OUT_TAG register, so we shift 3 bits to the right */
    reg_tag = buff_tmp[FIFO_WORD * i] >> 3;
    if(reg_tag == LSM6DSO_XL_NC_TAG) {
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = lsm6dso_convert_accel_data_to_mg((uint16_t) buff_tmp[(FIFO_WORD * i) + (2 * j) + 2] << 8 | buff_tmp[(FIFO_WORD * i) + (2 * j) + 1]);
      }
    }
    else if(reg_tag == LSM6DSO_GYRO_NC_TAG) {
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = lsm6dso_convert_gyro_data_to_mdps((uint16_t) buff_tmp[(FIFO_WORD * i) + (2 * j) + 2] << 8 | buff_tmp[(FIFO_WORD * i) + (2 * j) + 1]);
      }
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
static float lsm6dso_convert_gyro_data_to_mdps(int16_t gyro_raw_data)
{
  float gyro_data_mdps = 0.0;
#if (SENSOR_TYPE == GYROSCOPE)
  switch (GYROSCOPE_FS)
  {
  case LSM6DSO_125dps:
    gyro_data_mdps = lsm6dso_from_fs125_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_250dps:
    gyro_data_mdps = lsm6dso_from_fs250_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_500dps:
    gyro_data_mdps = lsm6dso_from_fs500_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_1000dps:
    gyro_data_mdps = lsm6dso_from_fs1000_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_2000dps:
    gyro_data_mdps = lsm6dso_from_fs2000_to_mdps(gyro_raw_data);
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
static float lsm6dso_convert_accel_data_to_mg(int16_t accel_raw_data)
{
  float accel_data_mg = 0.0;
#if (SENSOR_TYPE == ACCELEROMETER)
  switch (ACCELEROMETER_FS)
  {
  case LSM6DSO_2g:
    accel_data_mg = lsm6dso_from_fs2_to_mg(accel_raw_data);
    break;
  case LSM6DSO_4g:
    accel_data_mg = lsm6dso_from_fs4_to_mg(accel_raw_data);
    break;
  case LSM6DSO_8g:
    accel_data_mg = lsm6dso_from_fs8_to_mg(accel_raw_data);
    break;
  case LSM6DSO_16g:
    accel_data_mg = lsm6dso_from_fs16_to_mg(accel_raw_data);
    break;
  default:
    accel_data_mg = 0.0;
    break;
  }
#endif
  return accel_data_mg;
}

/*
 * Pressing the reset button while the sensor is answering to a read request
 * might lead to disaster.
 * In this case the device is stuck, waiting for pulses on SCL to finish the
 * previous transfer.
 * While stuck the sensor keep the SDA low.
 *
 * As a workaround we simply configure the SCL pin as a GPIO and send a burst
 * of pulses to bring the sensor back to an idle state.
 */
static void iks01a3_i2c_stuck_quirk(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure SCL as a GPIO */
  GPIO_InitStruct.Pin = SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

  /* Send a burst of pulses on SCL */
  int pulses = 20;
  do {
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
  } while (pulses--);

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_DISABLE();
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
