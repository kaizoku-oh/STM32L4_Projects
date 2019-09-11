/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_I2C_ADDRESS_0  0x76
#define BMP280_I2C_ADDRESS_1  0x77

#define RXBUFFERSIZE          8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId sensorThreadHandle;
osThreadId loggingThreadHandle;
osThreadId LedThreadHandle;
/* USER CODE BEGIN PV */
osSemaphoreId osSemaphore;

uint8_t rx_buff[RXBUFFERSIZE];

BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

bool bme280p;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

void StartSensorThread(void const * argument);
void StartLoggingThread(void const * argument);
void StartLedThread(void const * argument);

/* USER CODE BEGIN PFP */
static void MX_BMP_Init(void);
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
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  MX_I2C1_Init();
//  /* USER CODE BEGIN 2 */
//  MX_BMP_Init();
//  // BSP_GYRO_Init();
//  HAL_UART_Receive_IT(&huart2, rx_buff, 8);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreDef(SEM);
  osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of sensorThread */
  osThreadDef(sensorThread, StartSensorThread, osPriorityNormal, 0, 1024);
  sensorThreadHandle = osThreadCreate(osThread(sensorThread), osSemaphore);

  /* definition and creation of loggingThread */
  osThreadDef(loggingThread, StartLoggingThread, osPriorityLow, 0, 1024);
  loggingThreadHandle = osThreadCreate(osThread(loggingThread), osSemaphore);

  /* definition and creation of LedThread */
  osThreadDef(LedThread, StartLedThread, osPriorityIdle, 0, 128);
  LedThreadHandle = osThreadCreate(osThread(LedThread), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_BMP_Init(void)
{
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_1;
  bmp280.i2c = &hi2c1;

  while (!bmp280_init(&bmp280, &bmp280.params))
  {
    printf("BMP280 initialization failed\n");
    HAL_Delay(2000);
  }
  bme280p = bmp280.id == BMP280_CHIP_ID;
  if (bme280p == true)
  {
    printf("BMP280: found BMP280\n");
  }
  else
  {
    printf("BMP280: found BME280\n");
  }
}

// void Gyro_demo(void)
// {
//   /* Gyroscope variables */
//   float buffer[3] = {0};
//   int32_t xval = 0;
//   int32_t yval = 0;
//   int32_t zval = 0;
//   uint32_t xvalabs = 0;
//   uint32_t yvalabs = 0;
//   uint32_t zvalabs = 0;

//   /* Init Gyroscope Mems */
//   if(BSP_GYRO_Init() != HAL_OK)
//   {
//     /* Initialization Error */
//     Error_Handler();
//   }

//   /* Read Gyroscope Angular data */
//   BSP_GYRO_GetXYZ(buffer);

//   /* Get absolute value */
//   xval = (int32_t) buffer[0];
//   yval = (int32_t) buffer[1];
//   zval = (int32_t) buffer[2];
//   xvalabs = ABS(xval);
//   yvalabs = ABS(yval);
//   zvalabs = ABS(zval);
  
//   /* insert Delay */
//   HAL_Delay(200);
// }

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorThread */
/**
  * @brief  Function implementing the sensorThread thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartSensorThread */
void StartSensorThread(void const * argument)
{
  /* USER CODE BEGIN 5 */
  osSemaphoreId semaphore = (osSemaphoreId) argument;

  while (1)
  {
    if (semaphore != NULL)
    {
      /* Try to obtain the semaphore */
      if (osSemaphoreWait(semaphore , 100) == osOK)
      {
        // HAL_UART_Transmit(&huart2, "Hello from Sensor Thread\n", 25, 10);
        while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity))
        {
          printf("Temperature/pressure reading failed\n");
          osDelay(2000);
        }
        osSemaphoreRelease(semaphore);
        osDelay(500);
      }
    }
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartLoggingThread */
/**
* @brief Function implementing the loggingThread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggingThread */
void StartLoggingThread(void const * argument)
{
  /* USER CODE BEGIN StartLoggingThread */
  osSemaphoreId semaphore = (osSemaphoreId) argument;

  while (1)
  {
    if (semaphore != NULL)
    {
      /* Try to obtain the semaphore */
      if (osSemaphoreWait(semaphore , 100) == osOK)
      {
        // HAL_UART_Transmit(&huart2, "Hello from Logging Thread\n", 26, 10);
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (!bme280p)
        {
          printf(", Humidity: %.2f\n", humidity);
        }
        else
        {
          printf("\n");
        }
        osSemaphoreRelease(semaphore);
        osDelay(500);
      }
    }
  }
  /* USER CODE END StartLoggingThread */
}

/* USER CODE BEGIN Header_StartLedThread */
/**
* @brief Function implementing the LedThread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedThread */
void StartLedThread(void const * argument)
{
  /* USER CODE BEGIN StartLedThread */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    osDelay(1000);
  }
  /* USER CODE END StartLedThread */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
