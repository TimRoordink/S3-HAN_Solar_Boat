/*
 * HSB_CurrentModule.h
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#ifndef INC_HSB_CURRENTMODULE_H_
#define INC_HSB_CURRENTMODULE_H_

#include "adc.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

uint16_t currentE;
uint16_t currentF;
uint16_t rawValues[2];
uint16_t rawE;
uint16_t rawF;
uint16_t voltageE;
uint16_t voltageF;
const float offset = 322;
const float Vref = 3.3;
const float twelfBitADC = 4096;
const float unitValue = Vref / twelfBitADC * 1000;
const float sensitivity = 1000.0 / 264.0;// 1000mA per 265 mV
uint8_t convCompleted = 0;

#endif /* INC_HSB_CURRENTMODULE_H_ */
