#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define Buzzer_Pin          GPIO_PIN_11
#define Buzzer_GPIO_Port    GPIOD

extern TIM_HandleTypeDef    htim6;




void Buzzer_LOOP(void);

// 间隔时间响(持续无限长时间)
void Buzzer_Start_Circle(uint32_t beep_time_ms, uint32_t Off_time_ms); 
// 间隔时间响(持续一段时间)
void Buzzer_Start_Duration(uint32_t beep_time_ms, uint32_t Off_time_ms, uint64_t total_ms); 
// 单次响
void Buzzer_Start_Once(uint32_t beep_time_ms);  
// 停止响
void Buzzer_Stop(void); 
uint64_t Get_Buzzer_Statistics(void);

#endif
