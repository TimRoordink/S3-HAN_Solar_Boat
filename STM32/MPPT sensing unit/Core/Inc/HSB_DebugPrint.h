/*
 * HSB_DebugPrint.h
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */


#ifndef INC_HSB_DEBUGPRINT_H_
#define INC_HSB_DEBUGPRINT_H_

#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

char msg[20];

//Boolean for turn off debug print
//Make false to turn off
bool HSB_DebugPrintOn = true;

void HSB_DebugPrint_Init(bool input);

void HSB_DebugPrint(const char *x, ...);

#endif /* INC_HSB_DEBUGPRINT_H_ */
