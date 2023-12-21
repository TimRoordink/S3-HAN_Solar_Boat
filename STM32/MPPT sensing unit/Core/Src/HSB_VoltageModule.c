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
const float ReferenceVoltage = 4.096;
const float ADCBits = 65536;
const float ADCFactor = ReferenceVoltage / ADCBits;
//conversion factor to acount for the voltage divider on the PCB
//58.7 is the total resistance of the devider and 2.7 the small resistor
const float converionFactor = 1 / 2.7 * 58.7;

uint16_t adcRawE;
uint16_t adcRawF;
uint16_t ADCData;
uint16_t MSB = 0;
uint16_t LSB = 0;
uint8_t RX_Buffer [3] = "A"; // Receive buffer i2c

void HSB_VoltageModule_Init(uint8_t adress){
	i2cAddress = (adress << 1) | 0b0;
}

uint16_t HSB_ReadMCP3427(int channel){
	uint8_t read = 0;
	if(channel == 0){
		read = readCH1;
	}else{
		read = readCH2;
	}

	HAL_I2C_Master_Transmit(&hi2c1,i2cAddress,&read,1,1000); //Sending in Blocking mode
	HAL_Delay(10);
	do{
		HAL_I2C_Master_Receive(&hi2c1, i2cAddress, RX_Buffer, 3, 1000);
		HAL_Delay(10);
	}while(RX_Buffer[2] & 1<<(7));

	MSB = (uint16_t)RX_Buffer[0] * 0x100;
	LSB = (uint16_t)RX_Buffer[1];
	ADCData = MSB + LSB;
	return ADCData;
}

void HSB_VoltageModule(float* E, float* F){
	adcRawE = HSB_ReadMCP3427(0);
	float adcVoltageE = adcRawE * ADCFactor;
	float resultE = adcVoltageE * converionFactor;
	HSB_DebugPrint("Voltage Module\n\rValue E %.2f\n\r", resultE);

	adcRawF = HSB_ReadMCP3427(1);
	float adcVoltageF = adcRawF * ADCFactor;
	float resultF = adcVoltageF * converionFactor;
	HSB_DebugPrint("Voltage Module\n\rValue E %.2f\n\r", resultF);

	*E = resultE;
	*F = resultF;
}
