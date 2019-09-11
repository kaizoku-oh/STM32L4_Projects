#include "main.h"

#define RX_BUFFER_SIZE    32
#define QUEUE_SIZE        RX_BUFFER_SIZE
#define RX_ONE_BYTE       1
#define START_CMD_SIZE    6
#define STOP_CMD_SIZE     5

#define START_CMD         "start\n"
#define STOP_CMD          "stop\n"
#define LF                '\n'
#define CR                '\r'
#define CR_LF             "\r\n"

TIM_HandleTypeDef htim7;
UART_HandleTypeDef huart2;

uint8_t uRxByte;
uint8_t aRxBuffer[RX_BUFFER_SIZE];
QueueHandle_t xQueue;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void vTask1(void *pvParameters);

static void echo_back(uint8_t *string);
static void process_cmd(uint8_t *aRxBuffer, uint8_t current_index);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();

  HAL_UART_Receive_IT(&huart2, &uRxByte, RX_ONE_BYTE);

  xQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
  if (xQueue == NULL)
  {
    Error_Handler();
  }

  xTaskCreate(vTask1, "Task 1", 128, NULL, osPriorityHigh, NULL);

  /* Starts the TIM Base generation in interrupt mode */
  // HAL_TIM_Base_Start_IT(&htim7);

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
    printf("Running in main\n");
    HAL_Delay(1000);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* The following clock configuration sets the Clock configuration sets after System reset                */
  /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
  /* and to be eventually adapted to new clock configuration                                               */

  /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}

/* 
  In terms of frequency:
  HandleTimer.Init.Prescaler = Timer'sClockFrequency/DesiredFrequency -1;

  In terms of period:
  HandleTimer.Init.Prescaler = Timer'sClockFrequency*DesiredPeriod -1;

  Frequency is in Hz.
  Period in Seconds.

  Example to get a counter period 1 sec (1HZ)
  HandleTimer.Init.Prescaler = Timer'sClockFrequency -1;

  Example to get a counter frequency 100 Hz (10 msec period)
  HandleTimer.Init.Prescaler = Timer'sClockFrequency/100 -1; 
*/
static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 599;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

void vTask1(void *pvParameters)
{
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  uint8_t byte;
  uint8_t index = 0;

  for(;;)
  {
    if(xQueueReceive(xQueue, &byte, 0))
    {
      aRxBuffer[index] = byte;
      if (byte == LF)
      {
        printf("Should Echo Here\n");
        HAL_UART_Transmit(&huart2, aRxBuffer, index, 10);
        memset(aRxBuffer, 0, RX_BUFFER_SIZE);
        index = 0;
        // echo_back(aRxBuffer + index);
        // process_cmd(aRxBuffer, index);
      }
      index++;
    }
    vTaskDelay(xDelay100ms);
  }
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
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  } else if (htim->Instance == TIM7)
  {
    // printf("Timer Period Elapsed\r\n");
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  printf("UART Rx Callback\r\n");
  if (huart->Instance == USART2)
  {
    if (xQueueSendFromISR(xQueue, &uRxByte, NULL))
    {
      printf("Sending 1 byte from ISR to the Queue\r\n\r\n");
      // echo_back(aRxBuffer);
    }
    else
    {
      printf("Could not send 1 byte from ISR to queue\r\n\r\n");
    }
    HAL_UART_Receive_IT(&huart2, &uRxByte, RX_ONE_BYTE);
  }
}

static void echo_back(uint8_t *string)
{
  if (HAL_UART_Transmit_IT(&huart2, string, RX_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

static void process_cmd(uint8_t *aRxBuffer, uint8_t current_index)
{
  uint8_t start_cmd_index;
  uint8_t stop_cmd_index;

  start_cmd_index = current_index - START_CMD_SIZE;
  stop_cmd_index = current_index - STOP_CMD_SIZE;

  if (memcmp(aRxBuffer + start_cmd_index, START_CMD, START_CMD_SIZE) == 0)
  {
    // Start timer
    printf("Starting timer\r\n");
  }
  else if (memcmp(aRxBuffer + stop_cmd_index, STOP_CMD, STOP_CMD_SIZE) == 0)
  {
    // Stop timer
    printf("Stopping timer\r\n");
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  printf("Error_Handler\n");
  while (1);
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