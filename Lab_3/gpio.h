/*
 * gpio.h
 *
 *  Created on: 7 oct. 2019
 *      Author: Jean-Michel
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4xx_hal.h"

typedef struct{
	GPIO_TypeDef* port;
	uint16_t pin;
} GPIO_Pin;

GPIO_Pin pinCreate(GPIO_TypeDef* port, uint16_t pin);

void GPIO_Setup_Input(GPIO_Pin gpio, uint32_t pull);
void GPIO_Setup_Output_PP(GPIO_Pin gpio, uint32_t pull, uint32_t speed);
void GPIO_Setup_Output_OD(GPIO_Pin gpio, uint32_t pull, uint32_t speed);
void Enable_GPIO_Clock(GPIO_TypeDef * _port);

GPIO_PinState GPIO_ReadPin(GPIO_Pin gpio);
void GPIO_WritePin(GPIO_Pin gpio, GPIO_PinState PinState);


#endif /* GPIO_H_ */

/*
 * const GPIO_Pin PA0 = {GPIOA, GPIO_PIN_0};
const GPIO_Pin PA1 = {GPIOA, GPIO_PIN_1};
const GPIO_Pin PA2 = {GPIOA, GPIO_PIN_2};
const GPIO_Pin PA3 = {GPIOA, GPIO_PIN_3};
const GPIO_Pin PA4 = {GPIOA, GPIO_PIN_4};
const GPIO_Pin PA5 = {GPIOA, GPIO_PIN_5};
const GPIO_Pin PA6 = {GPIOA, GPIO_PIN_6};
const GPIO_Pin PA7 = {GPIOA, GPIO_PIN_7};
const GPIO_Pin PA8 = {GPIOA, GPIO_PIN_8};
const GPIO_Pin PA9 = {GPIOA, GPIO_PIN_9};
const GPIO_Pin PA10 = {GPIOA, GPIO_PIN_10};
const GPIO_Pin PA11 = {GPIOA, GPIO_PIN_11};
const GPIO_Pin PA12 = {GPIOA, GPIO_PIN_12};
const GPIO_Pin PA13 = {GPIOA, GPIO_PIN_13};
const GPIO_Pin PA14 = {GPIOA, GPIO_PIN_14};
const GPIO_Pin PA15 = {GPIOA, GPIO_PIN_15};

const GPIO_Pin PB0 = {GPIOB, GPIO_PIN_0};
const GPIO_Pin PB1 = {GPIOB, GPIO_PIN_1};
const GPIO_Pin PB2 = {GPIOB, GPIO_PIN_2};
const GPIO_Pin PB3 = {GPIOB, GPIO_PIN_3};
const GPIO_Pin PB4 = {GPIOB, GPIO_PIN_4};
const GPIO_Pin PB5 = {GPIOB, GPIO_PIN_5};
const GPIO_Pin PB6 = {GPIOB, GPIO_PIN_6};
const GPIO_Pin PB7 = {GPIOB, GPIO_PIN_7};
const GPIO_Pin PB8 = {GPIOB, GPIO_PIN_8};
const GPIO_Pin PB9 = {GPIOB, GPIO_PIN_9};
const GPIO_Pin PB10 = {GPIOB, GPIO_PIN_10};
const GPIO_Pin PB11 = {GPIOB, GPIO_PIN_11};
const GPIO_Pin PB12 = {GPIOB, GPIO_PIN_12};
const GPIO_Pin PB13 = {GPIOB, GPIO_PIN_13};
const GPIO_Pin PB14 = {GPIOB, GPIO_PIN_14};
const GPIO_Pin PB15 = {GPIOB, GPIO_PIN_15};

const GPIO_Pin PC0 = {GPIOC, GPIO_PIN_0};
const GPIO_Pin PC1 = {GPIOC, GPIO_PIN_1};
const GPIO_Pin PC2 = {GPIOC, GPIO_PIN_2};
const GPIO_Pin PC3 = {GPIOC, GPIO_PIN_3};
const GPIO_Pin PC4 = {GPIOC, GPIO_PIN_4};
const GPIO_Pin PC5 = {GPIOC, GPIO_PIN_5};
const GPIO_Pin PC6 = {GPIOC, GPIO_PIN_6};
const GPIO_Pin PC7 = {GPIOC, GPIO_PIN_7};
const GPIO_Pin PC8 = {GPIOC, GPIO_PIN_8};
const GPIO_Pin PC9 = {GPIOC, GPIO_PIN_9};
const GPIO_Pin PC10 = {GPIOC, GPIO_PIN_10};
const GPIO_Pin PC11 = {GPIOC, GPIO_PIN_11};
const GPIO_Pin PC12 = {GPIOC, GPIO_PIN_12};
const GPIO_Pin PC13 = {GPIOC, GPIO_PIN_13};
const GPIO_Pin PC14 = {GPIOC, GPIO_PIN_14};
const GPIO_Pin PC15 = {GPIOC, GPIO_PIN_15};

const GPIO_Pin PD0 = {GPIOD, GPIO_PIN_0};
const GPIO_Pin PD1 = {GPIOD, GPIO_PIN_1};
const GPIO_Pin PD2 = {GPIOD, GPIO_PIN_2};
const GPIO_Pin PD3 = {GPIOD, GPIO_PIN_3};
const GPIO_Pin PD4 = {GPIOD, GPIO_PIN_4};
const GPIO_Pin PD5 = {GPIOD, GPIO_PIN_5};
const GPIO_Pin PD6 = {GPIOD, GPIO_PIN_6};
const GPIO_Pin PD7 = {GPIOD, GPIO_PIN_7};
const GPIO_Pin PD8 = {GPIOD, GPIO_PIN_8};
const GPIO_Pin PD9 = {GPIOD, GPIO_PIN_9};
const GPIO_Pin PD10 = {GPIOD, GPIO_PIN_10};
const GPIO_Pin PD11 = {GPIOD, GPIO_PIN_11};
const GPIO_Pin PD12 = {GPIOD, GPIO_PIN_12};
const GPIO_Pin PD13 = {GPIOD, GPIO_PIN_13};
const GPIO_Pin PD14 = {GPIOD, GPIO_PIN_14};
const GPIO_Pin PD15 = {GPIOD, GPIO_PIN_15};

const GPIO_Pin PE0 = {GPIOE, GPIO_PIN_0};
const GPIO_Pin PE1 = {GPIOE, GPIO_PIN_1};
const GPIO_Pin PE2 = {GPIOE, GPIO_PIN_2};
const GPIO_Pin PE3 = {GPIOE, GPIO_PIN_3};
const GPIO_Pin PE4 = {GPIOE, GPIO_PIN_4};
const GPIO_Pin PE5 = {GPIOE, GPIO_PIN_5};
const GPIO_Pin PE6 = {GPIOE, GPIO_PIN_6};
const GPIO_Pin PE7 = {GPIOE, GPIO_PIN_7};
const GPIO_Pin PE8 = {GPIOE, GPIO_PIN_8};
const GPIO_Pin PE9 = {GPIOE, GPIO_PIN_9};
const GPIO_Pin PE10 = {GPIOE, GPIO_PIN_10};
const GPIO_Pin PE11 = {GPIOE, GPIO_PIN_11};
const GPIO_Pin PE12 = {GPIOE, GPIO_PIN_12};
const GPIO_Pin PE13 = {GPIOE, GPIO_PIN_13};
const GPIO_Pin PE14 = {GPIOE, GPIO_PIN_14};
const GPIO_Pin PE15 = {GPIOE, GPIO_PIN_15};

const GPIO_Pin PH0 = {GPIOH, GPIO_PIN_0};
const GPIO_Pin PH1 = {GPIOH, GPIO_PIN_1};
 */
