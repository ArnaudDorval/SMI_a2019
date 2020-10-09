/*
 * eeprom.c
 *
 *  Created on: Nov 2, 2019
 *      Author: Jean-Michel
 */

#include "eeprom.h"

#define PAGE_SIZE 64

EEPROM_Handler EEPROM_Init(I2C_TypeDef* instance, char A0, char A1, char A2) {

	// Initialize EEPROM_Handler
	EEPROM_Handler eeprom;

	// Generate I2C device address
	char address = 0b10100000;
	address = address | (A2 << 3);
	address = address | (A1 << 2);
	address = address | (A0 << 1);
	eeprom.address = address;

	I2C_HandleTypeDef hi2c1;

	// Initialize I2C peripheral
	hi2c1.Instance = instance;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(&hi2c1);
	eeprom.hi2c = hi2c1;

	return eeprom;
}

char EEPROM_Read(EEPROM_Handler* eeprom, uint16_t address, uint16_t nBytes, uint8_t *dest) {

	if(HAL_I2C_Mem_Read(&(eeprom->hi2c), eeprom->address, address,
			I2C_MEMADD_SIZE_16BIT, dest, nBytes, 1000) == HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}

char EEPROM_Write(EEPROM_Handler* eeprom, uint16_t address, uint16_t nBytes, uint8_t *source) {

	while(nBytes > 0) {
		int writableBytes = PAGE_SIZE-(address%PAGE_SIZE);
		if(writableBytes < nBytes) {

			if(HAL_I2C_Mem_Write(&(eeprom->hi2c), eeprom->address, address,
						I2C_MEMADD_SIZE_16BIT, source, writableBytes, 1000) != HAL_OK) {
				return 1;
			}
			HAL_Delay(10);
			nBytes -= writableBytes;
			address += writableBytes;
			source += writableBytes;

		} else {

			if(HAL_I2C_Mem_Write(&(eeprom->hi2c), eeprom->address, address,
						I2C_MEMADD_SIZE_16BIT, source, nBytes, 1000) != HAL_OK) {
				return 1;
			}
			nBytes = 0;

		}
	}

	return 0;
}


