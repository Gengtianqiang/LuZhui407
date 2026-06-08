#ifndef _BATTERY_H
#define _BATTERY_H

#include "stm32f4xx_hal.h"

extern float Volt;

float Get_battery_volt(void);
float Get_battery_volt_Average(uint8_t times);
void Set_battery_led(void);


#endif
