/*
 * timer.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Jean-Michel
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32f4xx_hal.h"


typedef struct{
	TIM_HandleTypeDef htim;
	uint32_t counter;
} Timer_Handler;


Timer_Handler Timer_Create(TIM_TypeDef *instance, uint32_t prescaler, uint32_t period);

void Timer_EnableInterrupts(Timer_Handler* timer);
void Timer_EnableNoInterrupts(Timer_Handler* timer);

#endif
