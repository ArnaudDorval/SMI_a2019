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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId taskLED1Handle;
osThreadId taskLED2Handle;
osThreadId taskLED3Handle;
osThreadId taskLED4Handle;
osThreadId taskSalutHandle;
osThreadId myTaskBonjourHandle;
osThreadId taskFillQueueHandle;
osThreadId taskEmptyQueueHandle;
osMessageQId receptionQueueHandle;
osMutexId uartTxMutexHandle;
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskLED1(void const * argument);
void StartTaskLED2(void const * argument);
void StartTaskLED3(void const * argument);
void StartTaskLED4(void const * argument);
void StartTaskSalut(void const * argument);
void StartTaskBonjour(void const * argument);
void StartTaskFillQueue(void const * argument);
void StartTaskEmptyQueue(void const * argument);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of uartTxMutex */
  osMutexDef(uartTxMutex);
  uartTxMutexHandle = osMutexCreate(osMutex(uartTxMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of receptionQueue */
  osMessageQDef(receptionQueue, 1, uint32_t);
  receptionQueueHandle = osMessageCreate(osMessageQ(receptionQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of taskLED1 */
  //osThreadDef(taskLED1, StartTaskLED1, osPriorityNormal, 0, 128);
  //taskLED1Handle = osThreadCreate(osThread(taskLED1), NULL);

  /* definition and creation of taskLED2 */
  //osThreadDef(taskLED2, StartTaskLED2, osPriorityNormal, 0, 128);
  //taskLED2Handle = osThreadCreate(osThread(taskLED2), NULL);

  /* definition and creation of taskLED3 */
  //osThreadDef(taskLED3, StartTaskLED3, osPriorityNormal, 0, 128);
  //taskLED3Handle = osThreadCreate(osThread(taskLED3), NULL);

  /* definition and creation of taskLED4 */
  //osThreadDef(taskLED4, StartTaskLED4, osPriorityNormal, 0, 128);
  //taskLED4Handle = osThreadCreate(osThread(taskLED4), NULL);

  /* definition and creation of taskSalut */
  //osThreadDef(taskSalut, StartTaskSalut, osPriorityNormal, 0, 128);
  //taskSalutHandle = osThreadCreate(osThread(taskSalut), NULL);

  /* definition and creation of myTaskBonjour */
  //osThreadDef(myTaskBonjour, StartTaskBonjour, osPriorityNormal, 0, 128);
  //myTaskBonjourHandle = osThreadCreate(osThread(myTaskBonjour), NULL);

  /* definition and creation of taskFillQueue */
  osThreadDef(taskFillQueue, StartTaskFillQueue, osPriorityNormal, 0, 128);
  taskFillQueueHandle = osThreadCreate(osThread(taskFillQueue), NULL);

  /* definition and creation of taskEmptyQueue */
  osThreadDef(taskEmptyQueue, StartTaskEmptyQueue, osPriorityNormal, 0, 128);
  taskEmptyQueueHandle = osThreadCreate(osThread(taskEmptyQueue), NULL);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskLED1 */
/**
* @brief Function implementing the taskLED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED1 */
void StartTaskLED1(void const * argument)
{
  /* USER CODE BEGIN StartTaskLED1 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    osDelay(1000);
  }
  /* USER CODE END StartTaskLED1 */
}

/* USER CODE BEGIN Header_StartTaskLED2 */
/**
* @brief Function implementing the taskLED2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED2 */
void StartTaskLED2(void const * argument)
{
  /* USER CODE BEGIN StartTaskLED2 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	osDelay(500);
  }
  /* USER CODE END StartTaskLED2 */
}

/* USER CODE BEGIN Header_StartTaskLED3 */
/**
* @brief Function implementing the taskLED3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED3 */
void StartTaskLED3(void const * argument)
{
  /* USER CODE BEGIN StartTaskLED3 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	osDelay(250);
  }
  /* USER CODE END StartTaskLED3 */
}

/* USER CODE BEGIN Header_StartTaskLED4 */
/**
* @brief Function implementing the taskLED4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED4 */
void StartTaskLED4(void const * argument)
{
  /* USER CODE BEGIN StartTaskLED4 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	osDelay(125);
  }
  /* USER CODE END StartTaskLED4 */
}

/* USER CODE BEGIN Header_StartTaskSalut */
/**
* @brief Function implementing the taskSalut thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSalut */
void StartTaskSalut(void const * argument)
{
  /* USER CODE BEGIN StartTaskSalut */
	uint8_t salutBuffer[] = "Salut";
	uint8_t head = 0;

	/* Infinite loop */
	for(;;)
	{
		head = 0;

		// On attend que le Mutex soit disponible
		osMutexWait(uartTxMutexHandle, osWaitForever);

		// On transmet "Salut" lettre par lettre
		while(head < 5) {
			if(HAL_UART_Transmit(&huart2, &(salutBuffer[head]), 1, 100) == HAL_OK) {
				head++;
			} else  {
				// Ce délai servait à générer volontairement des erreurs d'overlap
				osDelay(5);
			}
		}

		// On relâche le Mutex
		osMutexRelease(uartTxMutexHandle);

		// Ce délai sert à donner du temps au thread Bonjour de prendre le Mutex
		osDelay(10);
	}
  /* USER CODE END StartTaskSalut */
}

/* USER CODE BEGIN Header_StartTaskBonjour */
/**
* @brief Function implementing the myTaskBonjour thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBonjour */
void StartTaskBonjour(void const * argument)
{
  /* USER CODE BEGIN StartTaskBonjour */
	uint8_t bonjourBuffer[] = "Bonjour";
	uint8_t head = 0;

	/* Infinite loop */
	for(;;)
	{
		head = 0;

		// On attend que le Mutex soit disponible
		osMutexWait(uartTxMutexHandle, osWaitForever);

		// On transmet "Bonjour" lettre par lettre
		while(head < 7) {
			if(HAL_UART_Transmit(&huart2, &(bonjourBuffer[head]), 1, 100) == HAL_OK) {
				head++;
			} else {
				// Ce délai servait à générer volontairement des erreurs d'overlap
				osDelay(7);
			}
		}

		// On relâche le Mutex
		osMutexRelease(uartTxMutexHandle);

		// Ce délai sert à donner du temps au thread Salut de prendre le Mutex
		osDelay(10);
	}
  /* USER CODE END StartTaskBonjour */
}

/* USER CODE BEGIN Header_StartTaskFillQueue */
/**
* @brief Function implementing the taskFillQueue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskFillQueue */
void StartTaskFillQueue(void const * argument)
{
  /* USER CODE BEGIN StartTaskFillQueue */
  int8_t buffer[4];

  /* Infinite loop */
  for(;;)
  {
	// On attend de recevoir 4 octets par UART
    HAL_UART_Receive(&huart2, buffer, 4, osWaitForever);

    // On ajoute ces 4 octets à la file
    osMessagePut(receptionQueueHandle, (uint32_t) buffer, osWaitForever);
  }
  /* USER CODE END StartTaskFillQueue */
}

/* USER CODE BEGIN Header_StartTaskEmptyQueue */
/**
* @brief Function implementing the taskEmptyQueue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskEmptyQueue */
void StartTaskEmptyQueue(void const * argument)
{
  /* USER CODE BEGIN StartTaskEmptyQueue */
  uint8_t * buffer;

  /* Infinite loop */
  for(;;)
  {
	// On attend que la file se remplisse pour une seconde
	osEvent evt = osMessageGet(receptionQueueHandle, 1000);
    if(evt.status == osEventMessage) {
    	// Si on a reçu un message, on l'incrémente et on le renvoie
    	buffer = (uint8_t *) evt.value.p;
    	for(int i = 0; i < 4; i++) {
    		buffer[i]++;
    	}
    	HAL_UART_Transmit(&huart2, buffer, 4, 200);
    } else if (evt.status == osEventTimeout) {
    	// Sinon, on envoie "Zut"
    	buffer = "Zut \n";
    	HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sizeof(buffer), 200);
    }
  }
  /* USER CODE END StartTaskEmptyQueue */
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
  if (htim->Instance == TIM6) {
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
