/*
 * uart.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Jean-Michel
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define BUFFER_SIZE 20

typedef struct{
	UART_HandleTypeDef huart;
	uint8_t receivedData;
	uint8_t buffer[BUFFER_SIZE];
	uint8_t writePointer;
	uint8_t readPointer;
} UART_Handler;


UART_Handler UART_Create(USART_TypeDef *instance, uint32_t baud, uint32_t wordLength, uint32_t parity);

void UART_StartRX(UART_Handler* uart);
void UART_IncrementReadPtr(UART_Handler* uart);
void UART_IncrementWritePtr(UART_Handler* uart);
bool UART_ValidateChecksum(uint8_t command, uint8_t param, uint8_t checksum);


#endif /* INC_UART_H_ */
