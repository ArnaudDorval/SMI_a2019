
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "lcd.h"
#include "uart.h"
#include "timer.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

LCD_Handler lcd;
Timer_Handler tim3;
Timer_Handler tim2;
UART_Handler uart2;

uint8_t bytesReceived = 0;
uint8_t cursorPosition = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
void executeCommand(uint8_t command, uint8_t param);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */

  // Initialize UART
  uart2 = UART_Create(USART2, 19200, UART_WORDLENGTH_9B, UART_PARITY_EVEN);
  UART_StartRX(&uart2);

  // Initialize LED pins for monitoring
  GPIO_Init();

  // Initialize LCD
  lcd = LCD_Create(GPIOE, GPIO_PIN_8, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_7, GPIO_PIN_9, GPIO_PIN_11, GPIO_PIN_13);
  LCD_Write_String(&lcd, "JMF_ADL");


  // Initialize timer to change its value every 1 us
  tim3 = Timer_Create(TIM3, 84, 65535);
  Timer_EnableInterrupts(&tim3);
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM3->CR1 |= TIM_CR1_CEN;

  // Initialize timer to interrupt every 1 ms (timer counter)
    tim2 = Timer_Create(TIM2, 840, 99);
    Timer_EnableInterrupts(&tim2);

  /* Infinite loop */
  while (1)
  {

	// Wait for bytes to be received
	if(uart2.receivedData != 0) {

		uint8_t command = uart2.buffer[uart2.readPointer];
		UART_IncrementReadPtr(&uart2);
		uint8_t param = uart2.buffer[uart2.readPointer];
		UART_IncrementReadPtr(&uart2);
		uint8_t checksum = uart2.buffer[uart2.readPointer];
		UART_IncrementReadPtr(&uart2);

		// Execute command only if checksum is ok
		if(UART_ValidateChecksum(command, param, checksum)) {

			executeCommand(command, param);
		}

		// Decrement receivedData counter because we finished reading
		uart2.receivedData--;
	}

	// Update time on LCD
	uint32_t seconds = (uint32_t) tim2.counter/1000;
	char buffer [5];
	sprintf(buffer, "%05d", seconds);
	LCD_Move_Cursor(&lcd, 0, 0);
	LCD_Write_String(&lcd, "JMF_ADL    ");
	LCD_Write_String(&lcd, buffer);
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 PD13 PD14 PD15*/
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


void executeCommand(uint8_t command, uint8_t param) {

	switch(command) {

	case 0x41:
		if(param == 0x30) {
			// Éteint la LED
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		} else if(param == 0x31) {
			// Allume la LED
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}
		break;

	case 0x42:
		// Efface le LCD
		LCD_Clear_Display(&lcd);
		break;
	case 0x43:
		// Affiche caractère reçu sur LCD
		LCD_Move_Cursor(&lcd, cursorPosition, 1);
		LCD_Write_Data(&lcd, param);
		cursorPosition++;
		if(cursorPosition > 15) {
			cursorPosition = 0;
		}
		break;
	default:
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	// Announces that a new byte has been received
	bytesReceived++;

	// If a complete message is received, announce a new data
	if(bytesReceived == 3) {
		uart2.receivedData++;
		bytesReceived = 0;
	}

	// Restart UART reception for next byte
	UART_StartRX(&uart2);
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
