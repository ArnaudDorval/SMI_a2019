
#ifndef LCD_h
#define LCD_h

#include "stm32f4xx_hal.h"
#include "string.h"

typedef struct{
	GPIO_TypeDef *port;
	uint16_t pinRS;
	uint16_t pinRW;
	uint16_t pinEN;
	uint16_t pinsData[4];
} LCD_Handler;


// initializers
LCD_Handler LCD_Create(GPIO_TypeDef *gpioport, uint16_t rs, uint16_t rw, uint16_t enable,
  uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3);

void LCD_Init(LCD_Handler* lcd);

// high-level functions
void LCD_Clear_Display(LCD_Handler* lcd);
void LCD_Return_Home(LCD_Handler* lcd);
void LCD_Create_Char(LCD_Handler* lcd, uint8_t location, uint8_t charmap[]);
void LCD_Move_Cursor(LCD_Handler* lcd, uint8_t col, uint8_t row);
void LCD_Write_String(LCD_Handler* lcd, char *str);

// mid-level functions
void LCD_Write_Command(LCD_Handler* lcd, uint8_t value);
void LCD_Write_Data(LCD_Handler* lcd, uint8_t value);

// low-level functions
void LCD_Write(LCD_Handler* lcd, uint8_t);

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#endif
