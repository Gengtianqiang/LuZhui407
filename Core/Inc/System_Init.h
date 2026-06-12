#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H

#include "main.h"
#include <stdbool.h>
#include "forward.h"


// #define AHAND_CAR
// #define MIDDLE_CAR
 #define MIDDLE_CAR_FIRST //中间车第一辆
// #define BEHIND_CAR



void systemInit(void); 
typedef struct  {
	uint16_t pdoa_time;
	uint16_t twr_time;
	bool pdoa_state;
		bool twr_state;
	
}uwb_set;

typedef struct
{
    uint8_t data[UART_RX_BUFFER_SIZE];  // 数据缓冲区
    uint16_t len;                       // 实际接收的字节数
} UART4_Queue_Data_t;
extern uwb_set uwb_set_t;
extern ADC_HandleTypeDef hadc1;

bool uwb_set_state(void);
void systemInit(void) ;



#endif


