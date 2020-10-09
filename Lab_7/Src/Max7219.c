/*
 * Max7219.c
 *
 *  Created on: Dec 7, 2019
 *      Author: Arnaud Dorval
 */

#include "Max7219.h"
#include "stm32f4xx_hal.h"

void init_MAX7219_powerup(SPI_HandleTypeDef *hspi3)
{
	//dummy send
	//write_to_MAX7219(hspi3, MAX7219_ADDR_DECODE_MODE, 0xFF);

	//vrai init
	write_to_MAX7219(hspi3, MAX7219_ADDR_DECODE_MODE, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_INTENSITY, 0x0A);
	write_to_MAX7219(hspi3, MAX7219_ADDR_SCAN_LIM, 0x07);
	write_to_MAX7219(hspi3, MAX7219_ADDR_SHUTDOWN, 0x01);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DISP_TEST, 0x00);

	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_0, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_1, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_2, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_3, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_4, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_5, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_6, 0x00);
	write_to_MAX7219(hspi3, MAX7219_ADDR_DIG_7, 0x00);
	clearDisplay_MAX7219(hspi3);
}

void write_to_MAX7219(SPI_HandleTypeDef *hspi3, uint8_t addr, uint8_t data)
{
	uint16_t buff[1];
	buff[1] = 0x4141;
	uint16_t Addr, Data, Data_to_send;
	Addr = (uint16_t) addr;
	Data = (uint16_t) data;

	Data_to_send = ((Addr << 8) & 0xFF00) | Data;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(hspi3, (uint8_t*)buff, 2, 50);
	HAL_SPI_Transmit(hspi3, (uint8_t*)&Data_to_send, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
}


void clearDisplay_MAX7219(SPI_HandleTypeDef *hspi3)
{
	uint8_t i = DIGITS_TO_DISPLAY;
	while (i)
	{
		write_to_MAX7219(hspi3,  i, 0x00);
		i--;
	}

}

uint8_t setLed_MAX7219(SPI_HandleTypeDef *hspi3, uint8_t pOld, uint8_t pRow,uint8_t pColumn, uint8_t pState){
	uint8_t val = 0x00;
	uint8_t dest;
	uint8_t new;

    uint8_t bytet = (pRow * 0x0202020202ULL & 0x010884422010ULL) % 1023;
    new = pOld;
    if(pState == 1){
    	dest = 1UL << (8-pRow);
    	new |= 1UL << (8-pRow);
    }else{
    	dest = ~(1UL << (8-pRow));
    	new &= ~(1UL << (8-pRow));
    }

    write_to_MAX7219(hspi3, pColumn, new);
    HAL_Delay(1);
    return new;
}

void setCol_MAX7219(SPI_HandleTypeDef *hspi3, uint8_t pColumn, uint8_t pValue){

	uint8_t new;

    write_to_MAX7219(hspi3, pColumn, pValue);
    HAL_Delay(1);
}
