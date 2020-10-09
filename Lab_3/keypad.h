/*
 * keypad.h
 *
 *  Created on: 25 sept. 2019
 *      Author: Jean-Michel
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "gpio.h"

#define ROW_COUNT 4
#define COL_COUNT 3

typedef struct
{
	GPIO_Pin pinsRows[ROW_COUNT];
	GPIO_Pin pinsCols[COL_COUNT];
	char pressedKeys[ROW_COUNT*COL_COUNT];
} Keypad_Handler;

Keypad_Handler Keypad_Create(GPIO_Pin pinsRows[], GPIO_Pin pinsCols[]);
void Keypad_Init(Keypad_Handler* keypad);
void Scan_Keypad(Keypad_Handler* keypad);
void Reset_Pressed_Keys(Keypad_Handler* keypad);
void Set_Active_Column(Keypad_Handler* keypad, int column_to_activate);

#endif /* KEYPAD_H_ */
