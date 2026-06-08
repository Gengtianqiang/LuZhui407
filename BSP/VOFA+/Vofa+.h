#ifndef __VOFA_H
#define __VOFA_H

#include "string.h"
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"

#define FireWater_Mode  1
#define JustFloat_Mode  2

/*功能接口函数*/
void Vofa_Init_c(uint8_t Data_Mode , uint32_t Banud);
void Vofa_Printf_c(char *format, ...);
void Vofa_JustFloat_SendData(float send_value);
void Vofa_JustFloat_END(void);
void Vofa_JustFloat_RecvData(float *Target ,float *Kp ,float *Ki ,float *Kd );
uint8_t Serial_GetRxFlag(void);
/*转换功能函数*/
void Vofa_float_to_Bytes(float value , uint8_t *Byte);
float Vofa_bytes_to_float(uint8_t *data);
void Vofa_parse_float_array(const uint8_t *data, float *output, int count);
/*串口功能函数*/
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void usart1_send(uint8_t data);

#endif


