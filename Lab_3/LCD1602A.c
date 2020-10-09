/*
 * LCD1602A.c
 *
 *  Created on: Oct 3, 2019
 *      Author: Arnaud Dorval
 */


#include "LCD1602A.h"

// variable permet de gerer le offset des lignes
const uint8_t ROW_16[] = {0x00, 0x40};

/**
 * Constucteur du handler des pin du LCD
 * DB represente les data pin
 * RW read/write
 * EN enable(clk)
 * RS commande/data mode
 */
LCDPinHandler LCDCreate(pinOut pinRS, pinOut pinEN, pinOut pinRW, GPIO_TypeDef* DBport[], uint16_t DBpin[]){
	LCDPinHandler lcd;

	//strncpy(lcd.DBport, DBport);
	//strncpy(lcd.DBpin,DBpin);
	lcd.DBport[0] = DBport[0];
	lcd.DBport[1] = DBport[1];
	lcd.DBport[2] = DBport[2];
	lcd.DBport[3] = DBport[3];
	lcd.DBport[4] = DBport[4];
	lcd.DBport[5] = DBport[5];
	lcd.DBport[6] = DBport[6];
	lcd.DBport[7] = DBport[7];

	lcd.DBpin[0] = DBpin[0];
	lcd.DBpin[1] = DBpin[1];
	lcd.DBpin[2] = DBpin[2];
	lcd.DBpin[3] = DBpin[3];
	lcd.DBpin[4] = DBpin[4];
	lcd.DBpin[5] = DBpin[5];
	lcd.DBpin[6] = DBpin[6];
	lcd.DBpin[7] = DBpin[7];
	//lcd.pinDB.reg = pinDB->reg;
	//lcd.pinDB.pin = pinDB->pin;
	lcd.pinEN = pinEN;
	lcd.pinRS = pinRS;
	lcd.pinRW = pinRW;

	return lcd;
}

/*
 * initialise les parametre de base du lcd
 */
void Initialize(LCDPinHandler* lcdpin){

	uint8_t t = FUNCTION_SET | OPT_DataLength | OPT_Nblines;
	//HAL_GPIO_WritePin(lcdpin->pinRW.reg, lcdpin->pinRW.pin, 0);
	writeCommandeLCD(lcdpin, t);
	writeCommandeLCD(lcdpin, CLEAR_DISPLAY);
	writeCommandeLCD(lcdpin, DISPLAY_CONTROL | OPT_ON | OPT_CursorDisplay);
	writeCommandeLCD(lcdpin, ENTRY_MODE_SET | OPT_CursorINCR);
	writeCommandeLCD(lcdpin, RETURN_HOME);
}

/*
 * reset les caractere sur le lcd remet le cursor a 0,0 aussi
 */
void ClearDisplay(LCDPinHandler* lcdpin){
	writeCommandeLCD(lcdpin, CLEAR_DISPLAY);
}

/*
 * Permet de modifier les parametre du lcd
 * les commande sont defini dans LCD1602.h
 */
void writeCommandeLCD(LCDPinHandler* lcdpin, uint8_t command){
	//HAL_GPIO_WritePin(GPIOC, 0, GPIO_PIN_RESET);
	//GPIO_TypeDef *p = &pinRS.reg;
	HAL_GPIO_WritePin(lcdpin->pinRS.reg, lcdpin->pinRS.pin, 0);
	//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
	//lcdpin->pinRS.reg, lcdpin->pinRS.pin, 0x01
	writeToLCD(lcdpin, command);

}

/*
 * fonction qui ecrit 1 packet de data au lcd
 */
void writeToLCD(LCDPinHandler*lcdpin, uint8_t data){

	//pinOut p = lcdpin->pinDB;

	for (uint8_t i = 0; i < 8; i++){
		HAL_GPIO_WritePin(lcdpin->DBport[i], lcdpin->DBpin[i], (data >> i) & 0x01);
	}
	//lcdpin->pinDB[i].reg, lcdpin->pinDB[i].pin, (data >> i) & 0x01
	HAL_GPIO_WritePin(lcdpin->pinEN.reg, lcdpin->pinEN.pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdpin->pinEN.reg,lcdpin-> pinEN.pin, 0);
}

/*
 * fonction permet de deplacer curseur
 */
void MoveCursor(LCDPinHandler* lcdpin, uint8_t row, uint8_t column){
	writeCommandeLCD(lcdpin, 0x80 + ROW_16[row] + column);
}

/*
 * fonction permet d'ecrire 1 caractere a la position du curseur
 */
void WriteCharacter(LCDPinHandler* lcdpin, char data){
	HAL_GPIO_WritePin(lcdpin->pinRS.reg, lcdpin->pinRS.pin, 1);

	writeToLCD(lcdpin, data);
	//for(uint8_t i = 0; i < strlen(data); i++)
		//{
		//writeToLCD(data[i]);
		//}
}

/*
 * fonction permet d'ecrire un string ou un array de char
 */
void WriteString(LCDPinHandler *lcdpin, char *data)
{
	for(uint8_t i = 0; i < strlen(data); i++)
	{
		WriteCharacter(lcdpin, data[i]);
	}
}

/*
 * fonction permet d'ecrire 1 caractere special preprogrammer en entrant une valeur hex
 */
void WriteSpecialCaracter(LCDPinHandler *lcdpin, char data){
	HAL_GPIO_WritePin(lcdpin->pinRS.reg, lcdpin->pinRS.pin, 1);
	writeToLCD(lcdpin, data);
}
