#include "main.h"

#define QUEUE_SIZE        10
#define COUNTER_MAX       254

TIM_HandleTypeDef htim7;
QueueHandle_t xQueue;
uint8_t counter = 0;

void SystemClock_Config(void);
static void MX_TIM7_Init(void);
static void vReadingTask(void *pvParameters);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */ 
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* TIM7 Initialization Function */
  MX_TIM7_Init();

  xQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
  if (xQueue == NULL)
  {
    Error_Handler();
  }
  xTaskCreate(vReadingTask, "Reading", 128, NULL, osPriorityNormal, NULL);
  /* Starts the TIM Base generation in interrupt mode */
  HAL_TIM_Base_Start_IT(&htim7);
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

void vReadingTask(void *pvParameters)
{
  uint8_t count;
  uint8_t count_numbers[QUEUE_SIZE];
  uint8_t index = 0;
  // const TickType_t xDelay10s = pdMS_TO_TICKS(10000);

  for(;;)
  {
    // vTaskDelay(xDelay10s);
    // vPrintString("Reading Task is running\r\n");
    if(xQueueReceive(xQueue, &count, 0))
    {
      // Fill the buffer with the values coming from the queue
      count_numbers[index-1] = count;
      if (index >= QUEUE_SIZE)
      {
        index = 0;
      }
      if (count % 10 == 0 && count != 0)
      {
        for (int i = 0; i < QUEUE_SIZE; i++)
        {
          vPrintStringAndNumber("Got ", count_numbers[i]);
        }
      }
      index++;
    }
    else
    {
      // vPrintString("No msg\n");
    }
    // xQueueReset(xQueue);
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
  }
  else if (htim->Instance == TIM7)
  {
    printf("\r\n=== TIMER PERIOD ELAPSED ===\r\n\r\n");
    if (xQueueSendFromISR(xQueue, &counter, NULL))
    {
      if (counter >= COUNTER_MAX)
      {
        counter = 0;
      }
      counter++;
    }
    else
    {
      printf("Could not send to queue from ISR\r\n");
    }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("Error_Handler\n");
  while (1);
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
#endif
