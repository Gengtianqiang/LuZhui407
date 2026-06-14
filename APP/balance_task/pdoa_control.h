#ifndef PDOA_CONTROL_H
#define PDOA_CONTROL_H


#include "Parser.h"
#include "twr_control.h"
#include "jdy_driver.h"


/*加入调速系数*/
#define GaoSu_Speed     100
#define ZhongSu_Speed   100
#define DiSu_Speed      100

//出发返回时车距
#define START_DISTANCE   1.0
#define RETURN_DISTANCE  0.5

typedef enum{
	STATE_Straight,                                   
	STATE_Back,
	STATE_Stop,
	STATE_Turn_Left,
	STATE_Turn,
	STATE_Detection_IDLE
}Control_State;

void pdoa_follow(ProtocolData * pdoa_data);
extern bool behind_is_ready;

#endif // PDOA_CONTROL_H
