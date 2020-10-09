/*
 * eeprom.h
 *
 *  Created on: Nov 2, 2019
 *      Author: Jean-Michel
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32f4xx_hal.h"


typedef struct {
	char address;
	I2C_HandleTypeDef hi2c;
} EEPROM_Handler;

EEPROM_Handler EEPROM_Init(I2C_TypeDef* instance, char A0, char A1, char A2);

char EEPROM_Read(EEPROM_Handler* eeprom, uint16_t address,
		uint16_t nBytes, uint8_t *dest);

char EEPROM_Write(EEPROM_Handler* eeprom, uint16_t address,
		uint16_t nBytes, uint8_t *source);


#endif /* INC_EEPROM_H_ */
