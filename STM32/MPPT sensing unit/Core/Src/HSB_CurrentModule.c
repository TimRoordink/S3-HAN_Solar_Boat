/*
 * HSB_CurrentModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_CurrentModule.h"

float current;
uint16_t rawValues[2];
uint16_t raw;
float voltage;
const float offset = 322;
const float Vref = 3.3;
const float twelfBitADC = 4096;
const float unitValue = Vref / twelfBitADC * 1000;
const float sensitivity = 1000.0 / 264.0;// 1000mA per 265 mV
uint8_t convCompleted = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
}

uint16_t HSB_ReadCurrentModule(int channel){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
	while(!convCompleted);
	for(uint8_t i=0; i<hadc1.Init.NbrOfConversion; i++){
		  raw = (uint16_t) rawValues[channel];
	}

	raw = (uint16_t) rawValues[channel];
	voltage = unitValue * raw;
	current = (voltage - offset) * sensitivity;
	HSB_DebugPrint("\n\rCurrent Module\n\rValue %.2f \n\r", current);
	return current;



}

void HSB_CurrentModule(float* E, float* F){
	HSB_DebugPrint("start current");
	*E = HSB_ReadCurrentModule(0);
	*F = HSB_ReadCurrentModule(1);
}
