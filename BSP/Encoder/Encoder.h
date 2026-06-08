#ifndef __ENCODER_H_
#define __ENCODER_H_
#include "main.h"
#include "stm32f4xx_hal.h"

void Encoder_Init(void);
int Read_Encoder(uint8_t TIMX);
void Get_Velocity_Form_Encoder_C(void);
int16_t Encoder_Get_MotorA(void);
int16_t Encoder_Get_MotorB(void);
int16_t Encoder_Get_MotorC(void);
int16_t Encoder_Get_MotorD(void);
void Get_Velocity_Form_Encoder(void);

#endif



