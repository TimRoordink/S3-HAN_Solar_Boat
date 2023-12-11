/*
 * HSB_VoltageModule.h
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#ifndef INC_HSB_VOLTAGEMODULE_H_
#define INC_HSB_VOLTAGEMODULE_H_

#include "i2c.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>


void HSB_VoltageModule_Init(uint8_t adress);

uint16_t HSB_ReadMCP3427(int channel);

float HSb_ConvertValue(int channel);

void HSB_VoltageModule(float* E, float* F);

#endif /* INC_HSB_VOLTAGEMODULE_H_ */
