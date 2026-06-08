#include "motor/motor.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim8;

Motor_t Motor_Instance = {0};

bool Get_Motor_Enable(void)
{
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(Enable_PIN_GPIO_Port, Enable_PIN_Pin))
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint32_t Get_Motor_PwmMaxCount(void)
{
    return htim8.Init.Period;
}

/**
 * @brief 获取电机PWM比较值
 * @param TIM_CHANNEL 该参数可以是以下值之一：
 *         TIM_CHANNEL_1 TIM_CHANNEL_2 TIM_CHANNEL_3 TIM_CHANNEL_4
 *  或者  Motor_A  Motor_B  Motor_C  Motor_D
 * @return PWM比较值
 */
uint32_t Get_Motor_PwmCompare(uint32_t TIM_CHANNEL)
{
    return __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL);
}

/**
 * @brief 设置电机转动方向,和PWM比较值
 * @param Motor 电机编号：Motor_A  Motor_B  Motor_C  Motor_D
 * @param Direct_Compare PID的输出，有正有负
 */
void Set_Motor_PWM(uint32_t Motor, int32_t Direct_Compare)
{
    int32_t Direct = 0;
    uint32_t Compare = (uint32_t)abs(Direct_Compare); // 取绝对值

    // 确保不超过PWM最大占空比
    if (Compare > Motor_Instance.pwm_max_count + 1)
    {
        return;
    }

    if (Direct_Compare > 0)
    {
        Direct = 1;
    }
    else if (Direct_Compare < 0)
    {
        Direct = -1;
    }
    else
    {
        Direct = 0;
    }

    switch (Motor)
    {
    case Motor_A:
        Motor_Instance.A.direct = Direct;
        Motor_Instance.A.pwm_compare = Compare;
        break;

    case Motor_B:
        Motor_Instance.B.direct = Direct;
        Motor_Instance.B.pwm_compare = Compare;
        break;

    case Motor_C:
        Motor_Instance.C.direct = Direct;
        Motor_Instance.C.pwm_compare = Compare;
        break;

    case Motor_D:
        Motor_Instance.D.direct = Direct;
        Motor_Instance.D.pwm_compare = Compare;
        break;

    default:
        break;
    }
}

void __Set_Motor_Direct(uint32_t Motor, int32_t Direct)
{
    switch (Motor)
    {
    case Motor_A:
        if (Direct > 0)
        { // 正转
            HAL_GPIO_WritePin(Motor_B13_GPIO_Port, Motor_B13_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_B12_GPIO_Port, Motor_B12_Pin, GPIO_PIN_SET);
        }
        else if (Direct < 0)
        { // 反转
            HAL_GPIO_WritePin(Motor_B13_GPIO_Port, Motor_B13_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_B12_GPIO_Port, Motor_B12_Pin, GPIO_PIN_RESET);
        }
        else
        { // 刹车
            HAL_GPIO_WritePin(Motor_B13_GPIO_Port, Motor_B13_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_B12_GPIO_Port, Motor_B12_Pin, GPIO_PIN_RESET);
        }
        break;

    case Motor_B:
        if (Direct > 0)
        { // 正转
            HAL_GPIO_WritePin(Motor_C0_GPIO_Port, Motor_C0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_B14_GPIO_Port, Motor_B14_Pin, GPIO_PIN_SET);
        }
        else if (Direct < 0)
        { // 反转
            HAL_GPIO_WritePin(Motor_C0_GPIO_Port, Motor_C0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_B14_GPIO_Port, Motor_B14_Pin, GPIO_PIN_RESET);
        }
        else
        { // 刹车
            HAL_GPIO_WritePin(Motor_C0_GPIO_Port, Motor_C0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_B14_GPIO_Port, Motor_B14_Pin, GPIO_PIN_RESET);
        }
        break;

    case Motor_C:
        if (Direct > 0)
        { // 正转
            HAL_GPIO_WritePin(Motor_D12_GPIO_Port, Motor_D12_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_D10_GPIO_Port, Motor_D10_Pin, GPIO_PIN_SET);
        }
        else if (Direct < 0)
        { // 反转
            HAL_GPIO_WritePin(Motor_D12_GPIO_Port, Motor_D12_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_D10_GPIO_Port, Motor_D10_Pin, GPIO_PIN_RESET);
        }
        else
        { // 刹车
            HAL_GPIO_WritePin(Motor_D12_GPIO_Port, Motor_D12_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_D10_GPIO_Port, Motor_D10_Pin, GPIO_PIN_RESET);
        }
        break;

    case Motor_D:
        if (Direct > 0)
        { // 正转
            HAL_GPIO_WritePin(Motor_A8_GPIO_Port, Motor_A8_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_D3_GPIO_Port, Motor_D3_Pin, GPIO_PIN_SET);
        }
        else if (Direct < 0)
        { // 反转
            HAL_GPIO_WritePin(Motor_A8_GPIO_Port, Motor_A8_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_D3_GPIO_Port, Motor_D3_Pin, GPIO_PIN_RESET);
        }
        else
        { // 刹车
            HAL_GPIO_WritePin(Motor_A8_GPIO_Port, Motor_A8_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_D3_GPIO_Port, Motor_D3_Pin, GPIO_PIN_RESET);
        }
        break;

    default:
        break;
    }
}

uint32_t __Set_Motor_PwmCompare(uint32_t TIM_CHANNEL, uint32_t Compare)
{
    return __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL, Compare);
}

void Motor_Pwm_Start(void)
{
    Motor_Instance.pwm_running = true;
    Motor_Instance.pwm_max_count = Get_Motor_PwmMaxCount();
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // 启动 TIM8 的 CH1 PWM
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/*11/12添加电机初始化函数 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // 启动 TIM8 的 CH1 PWM
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void Motor_Pwm_Stop(void)
{
    // Motor_Instance.pwm_running = false;
    // 将比较值变为 0，可以替代 Stop PWM 输出
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1); //测试发现不能 HAL_TIM_PWM_Stop 否则会始终输出 2.94V 的PWM
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2); //测试发现不能 HAL_TIM_PWM_Stop 否则会始终输出 2.94V 的PWM
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3); //测试发现不能 HAL_TIM_PWM_Stop 否则会始终输出 2.94V 的PWM
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4); //测试发现不能 HAL_TIM_PWM_Stop 否则会始终输出 2.94V 的PWM
}

void Motor_Baker(void)
{
    __Set_Motor_Direct(Motor_A, 0);
    __Set_Motor_Direct(Motor_B, 0);
    __Set_Motor_Direct(Motor_C, 0);
    __Set_Motor_Direct(Motor_D, 0);
}

/**
 * @brief  带有 PWM 的智能"启停开关"功能。
 *      放在 While 的最后一个执行, 可以确保能稳定"停车"
 * @param  无
 */
void Motor_Task_Loop(void)
{
    bool isEnable = Get_Motor_Enable();
    // 能使能
    // 如果电池电压存在异常
    if (Volt < 12.0f && Volt > 5.0f)
    {
        Set_Motor_PWM(Motor_A, 0);
        Set_Motor_PWM(Motor_B, 0);
        Set_Motor_PWM(Motor_C, 0);
        Set_Motor_PWM(Motor_D, 0);
        Buzzer_Start_Once(100);
        return;
    }

    // 如果超声波检测有问题？

    // 开环控制  计算各电机PWM值，PWM代表轮组实际转速
    MOTOR_A.Motor_Pwm = Incremental_no_PI('A', MOTOR_A.Target);
    MOTOR_B.Motor_Pwm = Incremental_no_PI('B', MOTOR_B.Target);
    MOTOR_C.Motor_Pwm = Incremental_no_PI('C', MOTOR_C.Target);
    MOTOR_D.Motor_Pwm = Incremental_no_PI('D', MOTOR_D.Target);

    // 根据不同小车型号设置不同的PWM控制极性
    MOTOR_A.Motor_Pwm = -MOTOR_A.Motor_Pwm;
    MOTOR_B.Motor_Pwm = -MOTOR_B.Motor_Pwm;
    MOTOR_C.Motor_Pwm = -MOTOR_C.Motor_Pwm;
    MOTOR_D.Motor_Pwm = -MOTOR_D.Motor_Pwm;

    // 将目标速度值转换为PWM值
    Set_Motor_PWM(Motor_A, MOTOR_A.Motor_Pwm);
    Set_Motor_PWM(Motor_B, MOTOR_B.Motor_Pwm);
    Set_Motor_PWM(Motor_C, MOTOR_C.Motor_Pwm);
    Set_Motor_PWM(Motor_D, MOTOR_D.Motor_Pwm);
    
    if (isEnable == true)
    {
        // 但是未在运行中
        if (Motor_Instance.pwm_running == false)
        {
            Motor_Pwm_Stop();  // 防止意外启动
            Motor_Pwm_Start(); // 使能 PWM 输出
            Motor_Instance.pwm_running = true;
//            Vofa_Printf(&VOFA3, "Motor Start\r\n");
        }
        // 正常运行中
        else if (Motor_Instance.pwm_running == true)
        {
            __Set_Motor_Direct(Motor_A, Motor_Instance.A.direct);
            __Set_Motor_Direct(Motor_B, Motor_Instance.B.direct);
            __Set_Motor_Direct(Motor_C, Motor_Instance.C.direct);
            __Set_Motor_Direct(Motor_D, Motor_Instance.D.direct);

            __Set_Motor_PwmCompare(Motor_A, Motor_Instance.A.pwm_compare);
            __Set_Motor_PwmCompare(Motor_B, Motor_Instance.B.pwm_compare);
            __Set_Motor_PwmCompare(Motor_C, Motor_Instance.C.pwm_compare);
            __Set_Motor_PwmCompare(Motor_D, Motor_Instance.D.pwm_compare);
        }
    }
    // 未使能(停车键按下)
    else if (isEnable == false)
    {
        // 但是仍然在输出
        if (Motor_Instance.pwm_running == true)
        {
//            Vofa_Printf(&VOFA3, "Motor Stop\r\n");
        }
        // 一直 Stop 防止意外又启动了
        Motor_Pwm_Stop();
        Motor_Instance.pwm_running = false;
    }
}


/**
 * @brief  带有 PWM 的智能"启停开关"功能。
 *      放在 While 的最后一个执行, 可以确保能稳定"停车"
 * @param  无
 */
void Motor_Stop_Loop(void)
{
    bool isEnable = Get_Motor_Enable();
    // 能使能
    // 如果电池电压存在异常
    if (Volt < 12.0f && Volt > 5.0f)
    {
        Set_Motor_PWM(Motor_A, 0);
        Set_Motor_PWM(Motor_B, 0);
        Set_Motor_PWM(Motor_C, 0);
        Set_Motor_PWM(Motor_D, 0);
        Buzzer_Start_Once(100);
        return;
    }
    if (isEnable == false)
    {
        Motor_Pwm_Stop();
        Motor_Instance.pwm_running = false;
    }
}
/*更新功能函数，适配萝卜科技24V驱动四轮驱动函数*/
/*PC6  -- INA1- PC12 INB1-PA8
  PC7  -- INA1- PD10 INB1-PD12
  PC8  -- INA1- PC0  INB1-PB14
  PC9  -- INA1- PB13 INB1-PB12
  四PWM调速四通道,及每个通道对应的控制逻辑
  */
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
    // 电机正反转控制
    if (motor_a > 0)
        AIN1 = 0, AIN2 = 1;
    else
        AIN1 = 1, AIN2 = 0;
    // 电机转速控制
    //	 TIM_SetCompare4(TIM8,myabs(motor_a*A));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, myabs(motor_a));
    
    if (motor_b > 0)
        BIN1 = 0, BIN2 = 1;
    else
        BIN1 = 1, BIN2 = 0;
    // 电机转速控制
    //		TIM_SetCompare3(TIM8,myabs(motor_b*B));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, myabs(motor_b));
    
    // 电机正反转控制
    if (motor_c > 0)
        CIN2 = 0, CIN1 = 1;
    else
        CIN2 = 1, CIN1 = 0;
    // 电机转速控制
    //	 TIM_SetCompare2(TIM8,myabs(motor_c*C));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, myabs(motor_c));
    
    if (motor_d > 0)
        DIN2 = 0, DIN1 = 1;
    else
        DIN2 = 1, DIN1 = 0;
    // 电机转速控制
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, myabs(motor_d));
}

void differential_drive_control(float pid_output, float base_speed) 
{
    float left_speed, right_speed;
    
    // 基础速度 ± PID输出实现转向
    left_speed = base_speed + pid_output;
    right_speed = base_speed - pid_output;
    
    // 设置电机速度
    /*暂时先这样*/
    // Move_X = 0;
    // Move_Z = pid_output/10;
     Set_Pwm(left_speed*100,left_speed*100,right_speed*100,right_speed*100);
}

