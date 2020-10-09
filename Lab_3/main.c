
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LCD1602A.h"
#include "gpio.h"
#include "keypad.h"
#include "stdio.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void lcdConstructor(void);
void LCDOverFlow(void)

pinOut RS;
pinOut RW;
pinOut EN;

uint16_t DBpin[8];
LCDPinHandler lcd;

Keypad_Handler keypadHandle;
char txt[11];
#define tree 0x08
uint8_t counter;


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
  /* Infinite loop */

  lcdConstructor();
  HAL_GPIO_WritePin(lcd.pinEN.reg, lcd.pinEN.pin, 1);
  Initialize(&lcd);

  ClearDisplay(&lcd);
  WriteString(&lcd, "SMI_JMF_ADL");
  MoveCursor(&lcd, 1, 0);


  GPIO_Pin PB7 = pinCreate(GPIOB, GPIO_PIN_7);
  GPIO_Pin PB5 = pinCreate(GPIOB, GPIO_PIN_5);
  GPIO_Pin PB3 = pinCreate(GPIOB, GPIO_PIN_3);
  GPIO_Pin PD6 = pinCreate(GPIOD, GPIO_PIN_6);

  GPIO_Pin PD4 = pinCreate(GPIOD, GPIO_PIN_4);
  GPIO_Pin PD2 = pinCreate(GPIOD, GPIO_PIN_2);
  GPIO_Pin PD0 = pinCreate(GPIOD, GPIO_PIN_0);

  GPIO_Pin pinsRows[] = {PB7, PB5, PB3, PD6};
  GPIO_Pin pinsCols[] = {PD4, PD2, PD0};
  keypadHandle = Keypad_Create(pinsRows, pinsCols);
  Keypad_Init(&keypadHandle);

  counter = 0;
  while (1)
  {
	   HAL_Delay(300);
		Scan_Keypad(&keypadHandle);
		if(keypadHandle.pressedKeys[2] == '\000') {
			for (int i=0; i<2; i++) {
				if(keypadHandle.pressedKeys[i] == '#'){
					ClearDisplay(&lcd);
					WriteString(&lcd, "SMI_JMF_ADL");
					MoveCursor(&lcd, 1, 0);
					counter = 0;
				}
				else if(keypadHandle.pressedKeys[i] == '*'){
					LCDOverFlow();
					WriteSpecialCaracter(&lcd, tree);
				}
				else if(keypadHandle.pressedKeys[i] != '\000'){

					LCDOverFlow();
					//sprintf(txt, "%d", keypadHandle.pressedKeys[i]);
					WriteCharacter(&lcd,keypadHandle.pressedKeys[i]);
					counter++;
				}
			}
		}

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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0| GPIO_PIN_2 |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void lcdConstructor(void){
	  RS.reg = GPIOC;
	  RS.pin = GPIO_PIN_0;

	  RW.reg = GPIOC;
	  RW.pin = GPIO_PIN_2;

	  EN.reg = GPIOA;
	  EN.pin = GPIO_PIN_1;

	  portType DBport[] = {GPIOA, GPIOA, GPIOA, GPIOC, GPIOB, GPIOB, GPIOE, GPIOE};
	  uint16_t DBpin[] = {GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_4, GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_8, GPIO_PIN_10};

	  lcd = LCDCreate(RS, EN, RW, DBport, DBpin);

}

void LCDOverFlow(void){
	if (counter > 15){
		MoveCursor(&lcd, 1, 0);
		WriteString(&lcd, "                ");
		MoveCursor(&lcd, 1, 0);
		counter = 0;
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

