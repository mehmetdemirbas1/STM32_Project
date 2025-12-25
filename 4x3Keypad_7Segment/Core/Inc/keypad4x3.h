/*
 * keypad4x3.h
 *
 *  Created on: Dec 8, 2025
 *      Author: Mehmet Demirbaş
 */

#ifndef INC_KEYPAD4X3_H_
#define INC_KEYPAD4X3_H_

#include "main.h"

#define  ROWSIZE		4
#define  COLSIZE		3

void keypadInit(GPIO_TypeDef *GPIOx, uint16_t rowPin1, uint16_t rowPin2,
		uint16_t rowPin3, uint16_t rowPin4, uint16_t colPin1, uint16_t colPin2,
		uint16_t colPin3);

void readKeypad(GPIO_TypeDef *GPIOx, uint8_t *keyPressed);
uint8_t getChar(uint8_t row, uint8_t col);



#endif /* INC_KEYPAD4X3_H_ */
