#include "balance.h"
#include "twr_control.h"
#include "jdy_driver.h"
#include "mesh_mode.h"
#include "System_Init.h"
#include "pdoa_control.h"

uint8_t PID_Control_Time = 0;
float A_Encoder, B_Encoder, C_Encoder, D_Encoder;
uint8_t text_buffer[] = {0x01, 0x02, 0x03, 0x04};
extern mesh_datasend_pkt_t my_mesh_send_pkt;



void StartBalanceTask(void *argument)
{
#ifdef AHAND_CAR
	JDY_DEBUG_OUT("the ahand car is begin!");
#endif
#ifdef BEHIND_CAR
	JDY_DEBUG_OUT("the behand car is begin!");
#endif
#ifdef MIDDLE_CAR
	JDY_DEBUG_OUT("the middle car is begin!");
#endif
#ifdef MIDDLE_CAR_FIRST
	JDY_DEBUG_OUT("the middle car first is begin!");
#endif

	for (;;)
	{
		osDelay(10);


	}
}
