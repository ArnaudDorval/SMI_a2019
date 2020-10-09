/*
 * uart.c
 *
 *  Created on: Nov 22, 2019
 *      Author: Jean-Michel
 */

#include "uart.h"

UART_Handler UART_Create(USART_TypeDef *instance, uint32_t baud, uint32_t wordLength, uint32_t parity) {

	// Initialize UART_Handler
	UART_Handler uart;

	uart.readPointer = 0;
	uart.writePointer = 0;
	uart.receivedData = 0;

	// Initialize UART peripheral
	UART_HandleTypeDef huart;

	huart.Instance = instance;
	huart.Init.BaudRate = baud;
	huart.Init.WordLength = wordLength;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = parity;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart);
	uart.huart = huart;

	return uart;

}

void UART_StartRX(UART_Handler* uart) {

	HAL_UART_Receive_IT(uart, &(uart->buffer[uart->writePointer]), 1);
}

void UART_IncrementReadPtr(UART_Handler* uart) {

	uart->readPointer++;
	if(uart->readPointer > BUFFER_SIZE-1) {
		uart->readPointer = 0;
	}
}

void UART_IncrementWritePtr(UART_Handler* uart) {

	uart->writePointer++;
	if(uart->writePointer > BUFFER_SIZE-1) {
		uart->writePointer = 0;
	}
}

bool UART_ValidateChecksum(uint8_t command, uint8_t param, uint8_t checksum) {

	uint8_t sum = command + param + checksum;
	return sum==0;
}

