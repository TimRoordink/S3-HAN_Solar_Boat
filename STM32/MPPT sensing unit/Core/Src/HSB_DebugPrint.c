/*
 * HSB_DebugPrint.c
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */

#include "HSB_DebugPrint.h"


void HSB_DebugPrint_Init(bool input){
	HSB_DebugPrintOn = input;
	}

void HSB_DebugPrint(const char *x, ...){
	if(HSB_DebugPrintOn){
		//UART variables
		sprintf(msg,x);
		HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
	}
}
