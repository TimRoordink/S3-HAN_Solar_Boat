/*
 * HSB_CurrentModule.h
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#ifndef INC_HSB_CURRENTMODULE_H_
#define INC_HSB_CURRENTMODULE_H_

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "HSB_DebugPrint.h"

void HSB_ADC_Select_ChannelE (void);

void HSB_ADC_Select_ChannelF (void);

uint16_t HSB_ReadCurrentModule(int channel);

void HSB_CurrentModule(float* E, float* F);

#endif /* INC_HSB_CURRENTMODULE_H_ */
