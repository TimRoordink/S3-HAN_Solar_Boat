/*
 * HSB_VoltageModule.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_VoltageModule.h"

void HSB_VoltageModule_Init(uint8_t adress){
	i2cAddress = (adress << 1) | 0b0;
}

uint16_t HSB_ReadMCP3427(int channel){
	uint8_t RX_Buffer [] = "A"; // Receive buffer i2c
	HAL_I2C_Master_Transmit(&hi2c1,i2cAddress,&readCH1,1,1000); //Sending in Blocking mode
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, i2cAddress, RX_Buffer, 3,1000);
	HAL_Delay(10);
	return RX_Buffer[1] | RX_Buffer[2];
}
