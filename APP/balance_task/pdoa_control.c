#include "pdoa_control.h"

Control_State Current_State = STATE_Detection_IDLE;

#ifdef AHAND_CAR
PID_Controller heading_pid = {
	.kp = 1.8f,	 // 比例增益 - 需要调试
	.ki = 0.005f, // 积分增益 - 需要调试
	.kd = 0.2f,	 // 微分增益 - 需要调试
	.integral = 0.0f,
	.prev_error = 0.0f,
	.integral_limit = 50.0f, // 积分限幅
	.output_limit = 80.0f	 // 输出限幅
};
#else
PID_Controller heading_pid = {
	.kp = 1.2f,	 // 比例增益 - 需要调试
	.ki = 0.01f, // 积分增益 - 需要调试
	.kd = 0.2f,	 // 微分增益 - 需要调试
	.integral = 0.0f,
	.prev_error = 0.0f,
	.integral_limit = 50.0f, // 积分限幅
	.output_limit = 80.0f	 // 输出限幅
};
#endif


#ifdef AHAND_CAR
float speed_weight = 15;
#else

float speed_weight = 10;
#endif

float R_D_ratio = 1.00f;
extern Control_State Current_State;
void pdoa_follow(ProtocolData *pdoa_data)
{
	if (true == pdoa_data->PdoaisAvailable)
	{
		if (pdoa_data->aoa_deg > 60)
		{
			pdoa_data->aoa_deg = 60;
		}
		if (pdoa_data->aoa_deg < -60)
		{
			pdoa_data->aoa_deg = -60;
		}
		int pn = 0;
#ifdef AHAND_CAR
		pn = -1;

		if ( 0== my_4g_dtu.return_flag ) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
			
		}else{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

		}


#endif

#ifdef BEHIND_CAR 
		if(pdoa_data==&retuen_proto_data)
			pn = -1;
		else 
			pn = 1;

			
		
	
#endif
		
#ifdef MIDDLE_CAR_FIRST
		if(pdoa_data==&retuen_proto_data)
			pn = -1;
		else 
			pn = 1;

#endif
		
#ifdef MIDDLE_CAR
		if(pdoa_data==&retuen_proto_data)
			pn = -1;
		else 
			pn = 1;
#endif




		float angle_error =  pdoa_data->aoa_deg;

		float Horizontal_Distance = pdoa_data->distance_cm * 1.0f / 100;
 
		//目标车距
		float distance =  0;

#ifdef AHAND_CAR
		distance =  RETURN_DISTANCE;
#else
		if(pdoa_data==&retuen_proto_data)
			distance = RETURN_DISTANCE;
		else
			distance =  START_DISTANCE;
#endif

		switch (Current_State)
		{
		case STATE_Detection_IDLE:
			if (fabs(angle_error) < 15.0f)
			{
				Current_State = STATE_Turn;
			}
			else
			{
				Current_State = STATE_Turn;
			}
			break;

		case STATE_Stop:
			// Motor_Pwm_Stop();
			Current_State = STATE_Detection_IDLE;
			break;

		case STATE_Turn:
			/*直接将基站的AOA角度接收作为误差数据偏移*/
			if (fabs(angle_error) > 15.0f && Horizontal_Distance > distance) /*误差偏移，转向更平滑*/
			{
				float pid_output = pid_update(&heading_pid, angle_error, 1);
				if (fabs(pid_output) < 5.0f)
//					if (fabs(pid_output) < 5.0f)
				{
					pid_output = 0;
				}
				
				differential_drive_control(pid_output, 100);
			}
			else
			{
				Current_State = STATE_Detection_IDLE;
				heading_pid.integral = 0; // 积分项归0--防止下次转弯值过大
			}
			break;

		case STATE_Straight:
			if (Horizontal_Distance > 7.0f)
			{
				/*高速档（仅水平时生效）*/
				float speed = 2 * GaoSu_Speed * speed_weight * pn;
//				Set_Pwm(speed, speed, speed, speed);
				Set_Pwm(0, 0, 0, 0);
				Current_State = STATE_Detection_IDLE;
				// printf("水平→高速档，水平距离=%.2fm，速度=%.1f\r\n", Horizontal_Distance, speed);
			}
			else if (Horizontal_Distance > 3.5f && Horizontal_Distance < 7.0f)
			{
				/*中速档*/
				float speed = 2 * ZhongSu_Speed * speed_weight * pn;
				Set_Pwm(speed, speed, speed, speed);
				Current_State = STATE_Detection_IDLE;
				// printf("水平→中速档，水平距离=%.2fm，速度=%.1f\r\n", Horizontal_Distance, speed);
			}
			else if (Horizontal_Distance > distance && Horizontal_Distance < 3.5f)
			{
				/*低速档*/
				float speed = 2 * DiSu_Speed * speed_weight * pn;
				Set_Pwm(speed, speed, speed, speed);
				Current_State = STATE_Detection_IDLE;
				// printf("水平→低速档，水平距离=%.2fm，速度=%.1f\r\n", Horizontal_Distance, speed);
			}
			else if (Horizontal_Distance > 0.2f && Horizontal_Distance < distance)
			{
				/*停车档*/
				Motor_Pwm_Stop();
				Current_State = STATE_Detection_IDLE;
				// printf("水平→倒车档，水平距离=%.2fm，速度=%.1f\r\n", Horizontal_Distance, speed);
			}
			else if (Horizontal_Distance < 0.2f)
			{
				/*倒车档*/
				Motor_Pwm_Stop();
				Current_State = STATE_Detection_IDLE;

				// printf("水平→倒车档，水平距离=%.2fm，速度=%.1f\r\n", Horizontal_Distance, speed);
			}

			break;
		case STATE_Back:

			Current_State = STATE_Straight;
			break;
		default:
			break;
		}
	}
	else
	{
		Motor_Pwm_Stop();
	}
}

void Behind_Car_Loop()
{
	if(1==g_state_machine.behind_flag){

	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		pdoa_follow(&retuen_proto_data);
	}
	else {
		
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);	
		pdoa_follow(&proto_data);
	}
}



void Middle_Car_Loop()
{


#ifdef MIDDLE_CAR
	if(g_state_machine.middle_flag!=2)
		pdoa_follow(&proto_data);
	else
		pdoa_follow(&retuen_proto_data);
#endif

#ifdef MIDDLE_CAR_FIRST
	if(g_state_machine.middle_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		pdoa_follow(&proto_data);
	}
		
	else if(g_state_machine.middle_flag == 2) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		pdoa_follow(&retuen_proto_data);
	}
#endif

}

