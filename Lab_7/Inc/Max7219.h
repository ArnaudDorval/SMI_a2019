/*
 * Max7219.h
 *
 *  Created on: Dec 7, 2019
 *      Author: Arnaud Dorval
 */

#ifndef MAX7219_H_
#define MAX7219_H_


#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_spi.h"
//#include "stm32f4xx_hal_gpio.h"

#define MAX7219_ADDR_DECODE_MODE 0x09
#define MAX7219_ADDR_INTENSITY   0x0A
#define MAX7219_ADDR_SCAN_LIM    0x0B
#define MAX7219_ADDR_SHUTDOWN    0x0C
#define MAX7219_ADDR_DISP_TEST   0x0F

#define MAX7219_ADDR_DIG_0       0x01
#define MAX7219_ADDR_DIG_1	     0x02
#define MAX7219_ADDR_DIG_2       0x03
#define MAX7219_ADDR_DIG_3       0x04
#define MAX7219_ADDR_DIG_4       0x05
#define MAX7219_ADDR_DIG_5       0x06
#define MAX7219_ADDR_DIG_6       0x07
#define MAX7219_ADDR_DIG_7       0x08

#define DIGITS_TO_DISPLAY         8

void init_MAX7219_powerup(SPI_HandleTypeDef *hspi3);
void write_to_MAX7219(SPI_HandleTypeDef *hspi3, uint8_t addr, uint8_t data);
void clearDisplay_MAX7219(SPI_HandleTypeDef *hspi3);

//pas implemente
void setRow_MAX7219(uint8_t pRow, uint8_t pState);
//pas implemente
void setColumn_MAX7219(uint8_t pColumn, uint8_t pState);

//prend colonne de 0x01 a 0x08 prend ligne 1 a 6
uint8_t setLed_MAX7219(SPI_HandleTypeDef *hspi3,uint8_t pOld, uint8_t pRow,uint8_t pColumn, uint8_t pState);
void setCol_MAX7219(SPI_HandleTypeDef *hspi3, uint8_t pColumn, uint8_t pValue);


#endif /* MAX7219_H_ */
