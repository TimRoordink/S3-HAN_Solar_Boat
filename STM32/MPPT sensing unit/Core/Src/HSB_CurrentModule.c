/*
 * HSB_CurrentModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_CurrentModule.h"

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

void HSB_CurrenModule_Init()
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
}


void HSB_CurrentModule(){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
	//while(!convCompleted);

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
