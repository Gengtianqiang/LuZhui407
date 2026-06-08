#ifndef 	__BALANCE_H
#define 	__BALANCE_H
#include "main.h"
#include "stm32f4xx_hal.h"

void StartBalanceTask(void *argument);
extern float A_Encoder,B_Encoder,C_Encoder,D_Encoder;


#endif
