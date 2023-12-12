/*
 * HSB_VoltageModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_VoltageModule.h"
#include "HSB_DebugPrint.h"

//I2C variables
uint8_t i2cAddress = (0x68 << 1) | 0b0; //0xD0
uint8_t readCH1 = 0x88; // write to acd to start reading channel 1
uint8_t readCH2 = 0xA8; // write to acd to start reading channel 2

//adc variables
const float ReferenceVoltage = 3.3;
const float ADCBits = 4096;
const float ADCFactor = ReferenceVoltage / ADCBits;
//conversion factor to acount for the voltage divider on the PCB
//58.7 is the total resistance of the devider and 2.7 the small resistor
const float converionFactor = 1 / 2.7 * 58.7;

void HSB_VoltageModule_Init(uint8_t adress){
	i2cAddress = (adress << 1) | 0b0;
}

uint16_t HSB_ReadMCP3427(int channel){
	uint8_t RX_Buffer [] = "A"; // Receive buffer i2c
	uint8_t read = 0;
	if(channel == 0){
		read = readCH1;
	}else{
		read = readCH2;
	}
	HAL_I2C_Master_Transmit(&hi2c1,i2cAddress,&read,1,1000); //Sending in Blocking mode
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, i2cAddress, RX_Buffer, 3,1000);
	HAL_Delay(10);
	HSB_DebugPrint("\n\r\n\rRAW ADC VALUE = %u\n\r\n\r", (RX_Buffer[0] | RX_Buffer[1]));
	return RX_Buffer[0] | RX_Buffer[1];
}

float HSb_ConvertValue(int channel){
	uint16_t adcRaw = HSB_ReadMCP3427(channel);
	float adcVoltage = adcRaw * ADCFactor;
	float result = adcVoltage * converionFactor;
	HSB_DebugPrint("Voltage Module\n\rValue %.2f\n\r", result);
	return result;
}

void HSB_VoltageModule(float* E, float* F){
	HSB_DebugPrint("\n\rstart voltage\n\r");
	*E = HSb_ConvertValue(1);
	*F = HSb_ConvertValue(2);
}
