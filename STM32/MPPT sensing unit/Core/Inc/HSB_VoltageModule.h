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

//I2C variables
uint8_t i2cAddress = (0x68 << 1) | 0b0; //0xD0
uint8_t readCH1 = 0x88; // write to acd to start reading channel 1
uint8_t readCH2 = 0xA8; // write to acd to start reading channel 2

void HSB_VoltageModule_Init(uint8_t adress);

uint16_t HSB_ReadMCP3427(int channel);

#endif /* INC_HSB_VOLTAGEMODULE_H_ */
