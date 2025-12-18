/*
 * keypad4x3.c
 *
 *  Created on: Dec 8, 2025
 *      Author: Mehmet Demirbaş
 */

#include "keypad4x3.h"

static uint16_t rowArray[ROWSIZE];
static uint16_t colArray[COLSIZE];

uint8_t keypadArray[ROWSIZE][COLSIZE] =
{{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'},};

void keypadInit(GPIO_TypeDef *GPIOx, uint16_t rowPin1, uint16_t rowPin2,
		uint16_t rowPin3, uint16_t rowPin4, uint16_t colPin1, uint16_t colPin2,
		uint16_t colPin3) {

	rowArray[0] = rowPin1;
	rowArray[1] = rowPin2;
	rowArray[2] = rowPin3;
	rowArray[3] = rowPin4;
	colArray[0] = colPin1;
	colArray[1] = colPin2;
	colArray[2] = colPin3;

}

void readKeypad(GPIO_TypeDef *GPIOx, uint8_t *keyPressed)
{
	uint8_t row, col;

	for(row = 0; row < ROWSIZE; row++)
	{
		HAL_GPIO_WritePin(GPIOx, rowArray[row],	GPIO_PIN_RESET);

		for(col =0; col <COLSIZE; col++)
		{
			if(HAL_GPIO_ReadPin(GPIOx, colArray[col]) == GPIO_PIN_RESET)
			{
				*keyPressed = getChar(row, col);
			}

		}
		HAL_GPIO_WritePin(GPIOx, rowArray[row], GPIO_PIN_SET);
	}
}


uint8_t getChar(uint8_t row, uint8_t col)
{
	return keypadArray[row][col];
}


