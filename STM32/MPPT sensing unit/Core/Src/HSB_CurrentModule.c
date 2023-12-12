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

void HSB_ADC_Select_ChannelE (void) {
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void HSB_ADC_Select_ChannelF (void) {
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

uint16_t HSB_ReadCurrentModule(int channel){
	if 		(channel == 0) {
		HSB_ADC_Select_ChannelE();
	}
	else if (channel == 1){
		HSB_ADC_Select_ChannelF();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	rawValues[channel] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	raw = (uint16_t) rawValues[channel];
	voltage = unitValue * raw;
	current = (voltage - offset) * sensitivity;
	HSB_DebugPrint("Current Module\n\rChannel: %i\tValue %.2f \n\r", channel, current);
	return current;
}

void HSB_CurrentModule(float* E, float* F){
	HSB_DebugPrint("\n\rstart current\n\r");
	*E = HSB_ReadCurrentModule(0);
	*F = HSB_ReadCurrentModule(1);
}
