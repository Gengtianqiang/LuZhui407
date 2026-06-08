#ifndef __SLEEP_H
#define __SLEEP_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

bool Get_isWeakup(void);
void Set_Standby(void);

#endif
