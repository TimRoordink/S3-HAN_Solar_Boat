/*
 * HSB_CurrentModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_CurrentModule.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
}

void HSB_CurrentModule(){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
	while(!convCompleted);

	  for(uint8_t i=0; i<hadc1.Init.NbrOfConversion; i++){
		  rawE = (uint16_t) rawValues[0];
		  rawF = (uint16_t) rawValues[1];
	}

	voltageE = unitValue * rawE;
	voltageF = unitValue * rawF;

	currentE = (voltageE - offset) * sensitivity;
	currentF = (voltageF - offset) * sensitivity;

	HSB_DebugPrint("rawE: %hu ", rawE);

	HSB_DebugPrint("\trawF: %hu ", rawF);

	HSB_DebugPrint("\tvoltageE: %hu ", voltageE);

	HSB_DebugPrint("\tvoltageF: %hu ", voltageF);

	HSB_DebugPrint("\tcurrentE: %hu ", currentE);

	HSB_DebugPrint("\tcurrentF: %hu \r\n", currentF);

	HAL_Delay(250);
}
