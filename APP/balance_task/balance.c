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

	for (;;)
	{
		osDelay(10);
#ifdef TIME_IO
		HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_SET);
#endif

#ifdef AHAND_CAR
		StateMachine_Loop(&g_state_machine, &bracket_data);
		// pdoa_follow(&proto_data);
#endif

#ifdef BEHIND_CAR
		 Behind_Car_Loop();
		// pdoa_follow(&retuen_proto_data);
		// pdoa_follow(&proto_data);
#endif

		Motor_Stop_Loop();

#ifdef TIME_IO
		HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_RESET);
#endif
	}
}
