

#include "lcd.h"

uint8_t _row_offsets[2] = {0x00, 0x40};

LCD_Handler LCD_Create(GPIO_TypeDef *gpioport, uint16_t rs, uint16_t rw, uint16_t enable,
			     uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3)
{
	LCD_Handler lcd;

	lcd.port = gpioport;
	lcd.pinRS = rs;
	lcd.pinRW = rw;
	lcd.pinEN = enable;
	lcd.pinsData[0] = d0;
	lcd.pinsData[1] = d1;
	lcd.pinsData[2] = d2;
	lcd.pinsData[3] = d3;
  
	LCD_Init(&lcd);

	return lcd;
}

void LCD_Init(LCD_Handler* lcd) {

	//Initializing GPIO Pins

	GPIO_InitTypeDef gpio_init;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;

	gpio_init.Pin = lcd->pinRS | lcd->pinRW | lcd->pinEN | lcd->pinsData[0] | lcd->pinsData[1] | lcd->pinsData[2] | lcd->pinsData[3];

	HAL_GPIO_Init(lcd->port, &gpio_init);

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// so we'll wait 50 just to make sure
	HAL_Delay(50);

	// Now we pull both RS and R/W low to begin commands
	HAL_GPIO_WritePin(lcd->port, lcd->pinRW, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->port, lcd->pinRS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->port, lcd->pinEN, GPIO_PIN_RESET);



	//put the LCD into 4 bit or 8 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	// we start in 8bit mode, try to set 4 bit mode
	LCD_Write(lcd, 0x03);
	HAL_Delay(5); // wait min 4.1ms

	// second try
	LCD_Write(lcd, 0x03);
	HAL_Delay(5); // wait min 4.1ms

	// third go!
	LCD_Write(lcd, 0x03);
	HAL_Delay(1);

	// finally, set to 4-bit interface
	LCD_Write(lcd, 0x02);

	LCD_Write_Command(lcd, LCD_FUNCTIONSET | (LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS));
	LCD_Write_Command(lcd, LCD_DISPLAYCONTROL | (LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF));

	LCD_Clear_Display(lcd);

	LCD_Write_Command(lcd, LCD_ENTRYMODESET | (LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT));

}

/********** high level commands, for the user! */
void LCD_Clear_Display(LCD_Handler* lcd)
{
	LCD_Write_Command(lcd, LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

void LCD_Return_Home(LCD_Handler* lcd)
{
	LCD_Write_Command(lcd, LCD_RETURNHOME);  // set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCD_Create_Char(LCD_Handler* lcd, uint8_t location, uint8_t charmap[])
{
  location &= 0x7; // we only have 8 locations 0-7
  LCD_Write_Command(lcd, LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
	  LCD_Write_Data(lcd, charmap[i]);
  }
}

void LCD_Move_Cursor(LCD_Handler* lcd, uint8_t col, uint8_t row)
{
	LCD_Write_Command(lcd, LCD_SETDDRAMADDR + _row_offsets[row] + col);
}


void LCD_Write_String(LCD_Handler* lcd, char *str)
{
	for(uint8_t i = 0; i < strlen(str); i++)
	{
		LCD_Write_Data(lcd, (uint8_t) str[i]);
	}
}


/*********** mid level commands, for sending data/cmds */

void LCD_Write_Command(LCD_Handler* lcd, uint8_t value)
{
	HAL_GPIO_WritePin(lcd->port, lcd->pinRS, GPIO_PIN_RESET);
	LCD_Write(lcd, value>>4);
	LCD_Write(lcd, value);
}

void LCD_Write_Data(LCD_Handler* lcd, uint8_t value)
{
	HAL_GPIO_WritePin(lcd->port, lcd->pinRS, GPIO_PIN_SET);
	LCD_Write(lcd, value>>4);
	LCD_Write(lcd, value);
}

// LOW-LEVEL FUNCTIONS

void LCD_Write(LCD_Handler* lcd, uint8_t value)
{
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(lcd->port, lcd->pinsData[i], ((value >> i) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(lcd->port, lcd->pinEN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcd->port, lcd->pinEN, GPIO_PIN_SET);
	HAL_Delay(1);    // enable pulse must be >450ns
	HAL_GPIO_WritePin(lcd->port, lcd->pinEN, GPIO_PIN_RESET);
	HAL_Delay(1);   // commands need > 37us to settle
}

