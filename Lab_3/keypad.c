/**
  ******************************************************************************
  * @file           : keypad.c
  * @brief          : Utility to control matrix keypads
  ******************************************************************************
  */

#include "keypad.h"


const char KEYPAD_KEYS[ROW_COUNT][COL_COUNT] = {{'1', '2', '3'},
															  {'4', '5', '6'},
															  {'7', '8', '9'},
															  {'*', '0', '#'}};

Keypad_Handler Keypad_Create(GPIO_Pin pinsRows[], GPIO_Pin pinsCols[])
{
	Keypad_Handler keypad;

	keypad.pinsRows[0] = pinsRows[0];
	keypad.pinsRows[1] = pinsRows[1];
	keypad.pinsRows[2] = pinsRows[2];
	keypad.pinsRows[3] = pinsRows[3];
	keypad.pinsCols[0] = pinsCols[0];
	keypad.pinsCols[1] = pinsCols[1];
	keypad.pinsCols[2] = pinsCols[2];

	return keypad;
}

void Keypad_Init(Keypad_Handler* keypad)
{
	/*Configure GPIO pins : COLUMNS*/
	for(int i=0; i<COL_COUNT; i++) {
		GPIO_Setup_Input(keypad->pinsCols[i], GPIO_PULLUP);
	}

	/*Configure GPIO pins : ROWS*/
	for(int i=0; i<ROW_COUNT; i++) {
		GPIO_Setup_Input(keypad->pinsRows[i], GPIO_PULLUP);
	}

	Reset_Pressed_Keys(keypad);
}

void Scan_Keypad(Keypad_Handler* keypad)
{
	Reset_Pressed_Keys(keypad);
	int cnt = 0;

	for(int col=0; col<COL_COUNT; col++) {
		// Set active column to 0, others to input
		Set_Active_Column(keypad, col);

		for(int row=0; row<ROW_COUNT; row++) {
			int row_state = GPIO_ReadPin(keypad->pinsRows[row]);
			if(row_state == GPIO_PIN_RESET) {
				keypad->pressedKeys[cnt] = KEYPAD_KEYS[row][col];
				cnt++;
			}
		}
	}
}

void Reset_Pressed_Keys(Keypad_Handler* keypad)
{
	for(int i=0; i < sizeof(keypad->pressedKeys); i++) {
		keypad->pressedKeys[i] = '\000';
	}
}

void Set_Active_Column(Keypad_Handler* keypad, int column_to_activate)
{
	for(int i=0; i<COL_COUNT; i++) {
		if(i == column_to_activate){
			GPIO_Setup_Output_PP(keypad->pinsCols[i], GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
			GPIO_WritePin(keypad->pinsCols[i], GPIO_PIN_RESET);
		} else {
			GPIO_Setup_Input(keypad->pinsCols[i], GPIO_PULLUP);
		}
	}
}

