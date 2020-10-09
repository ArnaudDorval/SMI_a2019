/**
  ******************************************************************************
  * @file           : gpio.c
  * @brief          : Utility to work with GPIOs easier
  ******************************************************************************
  */

#include "gpio.h"


GPIO_Pin pinCreate(GPIO_TypeDef* port, uint16_t pin){
	GPIO_Pin pinBuff;
	pinBuff.port = port;
	pinBuff.pin = pin;

	return pinBuff;
}

void GPIO_Setup_Input(GPIO_Pin gpio, uint32_t pull)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = gpio.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = pull;
	HAL_GPIO_Init(gpio.port, &GPIO_InitStruct);

	Enable_GPIO_Clock(gpio.port);
}


void GPIO_Setup_Output_PP(GPIO_Pin gpio, uint32_t pull, uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = gpio.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = pull;
	GPIO_InitStruct.Speed = speed;
	HAL_GPIO_Init(gpio.port, &GPIO_InitStruct);

	Enable_GPIO_Clock(gpio.port);
}


void GPIO_Setup_Output_OD(GPIO_Pin gpio, uint32_t pull, uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = gpio.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = pull;
	GPIO_InitStruct.Speed = speed;
	HAL_GPIO_Init(gpio.port, &GPIO_InitStruct);

	Enable_GPIO_Clock(gpio.port);
}


void Enable_GPIO_Clock(GPIO_TypeDef * _port)
{
	if(_port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if(_port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if(_port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if(_port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if(_port == GPIOD)
		__HAL_RCC_GPIOD_CLK_ENABLE();
	else if(_port == GPIOE)
		__HAL_RCC_GPIOE_CLK_ENABLE();
	else if(_port == GPIOH)
		__HAL_RCC_GPIOH_CLK_ENABLE();
}

GPIO_PinState GPIO_ReadPin(GPIO_Pin gpio)
{
	return HAL_GPIO_ReadPin(gpio.port, gpio.pin);
}

void GPIO_WritePin(GPIO_Pin gpio, GPIO_PinState pinState)
{
	HAL_GPIO_WritePin(gpio.port, gpio.pin, pinState);
}
