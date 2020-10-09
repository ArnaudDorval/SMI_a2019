/*
 * timer.c
 *
 *  Created on: Nov 22, 2019
 *      Author: Jean-Michel
 */

#include "timer.h"

Timer_Handler Timer_Create(TIM_TypeDef *instance, uint32_t prescaler, uint32_t period) {

	// Initialize UART_Handler
	Timer_Handler timer;

	timer.counter = 0;

	// Initialize TIMER peripheral
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	TIM_HandleTypeDef htim;

	htim.Instance = instance;
	htim.Init.Prescaler = prescaler;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = period;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&htim);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	timer.htim = htim;

	return timer;

}


void Timer_EnableInterrupts(Timer_Handler* timer) {

	HAL_TIM_Base_Start_IT(&(timer->htim));
}

void Timer_EnableNoInterrupts(Timer_Handler* timer) {

	HAL_TIM_Base_Start(&(timer->htim));
}
