/*
 * HSB_DebugPrint.h
 *
 *  Created on: Dec 7, 2023
 *      Author: tim13
 */


#ifndef INC_HSB_DEBUGPRINT_H_
#define INC_HSB_DEBUGPRINT_H_

#include "main.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void HSB_DebugPrint_Init(bool input);

void HSB_DebugPrint(const char *x, ...);

#endif /* INC_HSB_DEBUGPRINT_H_ */
