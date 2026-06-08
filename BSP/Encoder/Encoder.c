#include "Encoder.h"
/**************************************************************************
函数功能：初始化编码器
入口参数：无
返回值：无
**************************************************************************/

void Encoder_Init(void)
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
函数功能：读取编码器计数
入口参数：定时器
返回  值：编码器数值(代表速度)
**************************************************************************/
int Read_Encoder(uint8_t TIMX)
{
 int Encoder_TIM;    
 switch(TIMX)
 {
	case 2:  Encoder_TIM= (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;
	case 3:  Encoder_TIM= (short)TIM3 -> CNT;   TIM3 -> CNT=0;  break;
	case 4:  Encoder_TIM= (short)TIM4 -> CNT;   TIM4 -> CNT=0;  break;	
	case 5:  Encoder_TIM= (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;	
	default: Encoder_TIM=0;
 }
	return Encoder_TIM;
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder_C(void)
{
	float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
	/*用于获取编码器的原始数据 */
	Encoder_A_pr= Encoder_Get_MotorA(); 
	Encoder_B_pr= Encoder_Get_MotorB(); 
	Encoder_C_pr= Encoder_Get_MotorC();  
	Encoder_D_pr= Encoder_Get_MotorD(); 

	/*给对应电机编码器赋值*/
	MOTOR_A.Encoder= Encoder_A_pr;
	MOTOR_B.Encoder= Encoder_B_pr;
	MOTOR_C.Encoder= Encoder_C_pr;
	MOTOR_D.Encoder= Encoder_D_pr;
	
	/*运动位置获取*/
	MOTOR_A.Location += Encoder_A_pr;
	MOTOR_B.Location += Encoder_B_pr;
	MOTOR_C.Location += Encoder_C_pr;
	MOTOR_D.Location += Encoder_D_pr;
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：MotorA 对应 TIM2	 MotorB 对应 TIM4  MotorC 对应 TIM1 MotorD 对应 TIM3
返回  值：无
**************************************************************************/

int16_t Encoder_Get_MotorA(void)
{
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SetCounter(&htim4, 0);
	return Temp;	
}

int16_t Encoder_Get_MotorB(void)
{
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SetCounter(&htim2, 0);
	return Temp;
}

int16_t Encoder_Get_MotorC(void)
{
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SetCounter(&htim3, 0);
	return Temp;
}

int16_t Encoder_Get_MotorD(void)
{
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SetCounter(&htim1, 0);
	return Temp;
}

void Get_Velocity_Form_Encoder(void)
{
	float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //用于获取编码器的原始数据 

	Encoder_A_pr= Encoder_Get_MotorA(); 
	Encoder_B_pr= Encoder_Get_MotorB(); 
	Encoder_C_pr= Encoder_Get_MotorC();  
	Encoder_D_pr= Encoder_Get_MotorD(); 

	//The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s
	MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
}

