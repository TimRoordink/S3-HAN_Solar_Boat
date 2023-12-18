/*
 * HSB_CurrentModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_CurrentModule.h"

float currentE;
uint16_t rawE;
float voltageE;

float currentF;
uint16_t rawF;
float voltageF;

const float offset = 322;
const float Vref = 3.3;
const float twelfBitADC = 4096;
const float unitValue = Vref / twelfBitADC * 1000;
const float sensitivity = 1000.0 / 264.0;// 1000mA per 265 mV

uint16_t adcResults[2];
int adcChannelCount = 2;
volatile int adcConversionComplete = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcConversionComplete = 1;
}

void HSB_ReadCurrentModule(float* E, float* F){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResults, adcChannelCount);
	while (adcConversionComplete == 0) {
		  //wait
	}
	adcConversionComplete = 0;


	rawE = adcResults[0];
	voltageE = unitValue * rawE;
	currentE = (voltageE - offset) * sensitivity;
	HSB_DebugPrint("Current Module\n\rChannelE:\tValue %.2f \n\r", currentE);

	rawF = adcResults[1];
	voltageF = unitValue * rawF;
	currentF = (voltageF - offset) * sensitivity;
	HSB_DebugPrint("Current Module\n\rChannelF:\tValue %.2f \n\r", currentF);

	*E = currentE;
	*F = currentF;

}

//void HSB_CurrentModule(float* E, float* F){
//	HSB_DebugPrint("\n\rstart current\n\r");
//	*F = HSB_ReadCurrentModule(1);
//	//*E = HSB_ReadCurrentModule(0);
//
//}
