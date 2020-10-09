
#ifndef LCD1602A_H_
#define LCD1602A_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "main.h"

#define portType GPIO_TypeDef*

	typedef struct{
		GPIO_TypeDef* reg;
		uint16_t pin;
	} pinOut;

	typedef struct
	{
		pinOut pinRS;
		pinOut pinEN;
		pinOut pinRW;
		GPIO_TypeDef* DBport[8];
		uint16_t DBpin[8];


	} LCDPinHandler;

	void Initialize(LCDPinHandler* lcdpin);
	void writeCommandeLCD(LCDPinHandler* lcdpin, uint8_t command);
	void writeToLCD(LCDPinHandler* lcdpin, uint8_t data);
	void ClearDisplay(LCDPinHandler* lcdpin);
	//void ActivateCursor(LCDPinHandler* lcdpin, _Bool pActivate);
	void MoveCursor(LCDPinHandler* lcdpin, uint8_t row, uint8_t column);
	void WriteCharacter(LCDPinHandler* lcdpin, char data);
	void WriteString(LCDPinHandler *lcdpin, char *data);
	void WriteSpecialCaracter(LCDPinHandler *lcdpin, char data);
	//_Bool checkBusy(LCDPinHandler* lcdpin);
	LCDPinHandler LCDCreate(pinOut pinRS, pinOut pinEN, pinOut pinRW, GPIO_TypeDef* DBport[], uint16_t DBpin[]);


	/*
	 * liste de commande
	 */
	#define CLEAR_DISPLAY 0x01

	#define RETURN_HOME 0x02

	#define ENTRY_MODE_SET 0x04
	#define OPT_ShiftDisplay	0x01
	#define OPT_CursorINCR 0x02

	#define DISPLAY_CONTROL 0x08
	#define OPT_ON	0x04
	#define OPT_CursorDisplay	0x02
	#define OPT_Blink 	0x01

	#define CURSOR_DISPLAY_SHIFT 0x10

	#define FUNCTION_SET 0x20
	#define OPT_DataLength 0x10
	#define OPT_Nblines 0x08
	#define OPT_font 0x04

	#define SET_DDRAM_ADDR 0x80


#endif

