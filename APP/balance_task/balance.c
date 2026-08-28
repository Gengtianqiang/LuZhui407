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
// 0: BLE remote control mode, 1: TWR control mode

void StartBalanceTask(void *argument)
{
#ifdef AHAND_CAR
#endif
#ifdef BEHIND_CAR
#endif
#ifdef MIDDLE_CAR
#endif
#ifdef MIDDLE_CAR_FIRST
#endif

	for (;;)
	{
		osDelay(10);
#ifdef TIME_IO
		HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_SET);
#endif


#ifdef AHAND_CAR
		if(my_4g_dtu.mode_flag)
		{	
			if(0==my_4g_dtu.return_flag) {
				BLE_control();
				Motor_Task_Loop();
			    g_state_machine.ahand_flag = 1;

			}else {
				//收到返回指令
				g_state_machine.ahand_flag = 3;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
				pdoa_follow(&proto_data);
			}

		}
		else
		{

			StateMachine_Loop(&g_state_machine, &bracket_data);
		
		}
		// pdoa_follow(&proto_data);
#endif

#ifdef BEHIND_CAR
		 Behind_Car_Loop();
		// pdoa_follow(&retuen_proto_data);
		// pdoa_follow(&proto_data);
#endif

#ifdef  MIDDLE_CAR_FIRST
		//  Middle_Car_Loop();
		// pdoa_follow(&retuen_proto_data);
		// pdoa_follow(&proto_data);
		static uint16_t cnt = 0;
		uint16_t pwm;

		// 10s = 1000个10ms节拍
		if(cnt < 1000)
		{
			pwm = (uint16_t)((uint32_t)16799 * cnt / 1000U);
			Set_Pwm(pwm, pwm, pwm, pwm);
			cnt++;
		}
		else
		{
			Set_Pwm(0,0,0,0);
			cnt = 0; // 打开这句就循环，注释只跑一次
		}
#endif

#ifdef  MIDDLE_CAR
		 Middle_Car_Loop();
		// pdoa_follow(&retuen_proto_data);
		// pdoa_follow(&proto_data);



#endif
		

		Motor_Stop_Loop();

#ifdef TIME_IO
		HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_RESET);
#endif
	}
}
