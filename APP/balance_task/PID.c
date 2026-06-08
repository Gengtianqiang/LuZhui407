#include "PID.h"

/**************************************************************************
Function: PID调控输出�?
Input   : *PID
Output  : PWM驱动�?
函数功能：�?�目标期望值进行PID调控
入口参数�?
返回  值：�?
**************************************************************************/

void PID_Control(PID_Para_t *PID)
{
    PID->Error1 = PID->Error0;
    PID->Error0 = PID->Target - PID->Actual;
    /*�?分作用判�?*/
    if (PID->Ki != 0)
    {
        PID->ErrorInt += PID->Error0;
        //			if(PID->ErrorInt>30){PID->ErrorInt = 30;}
        //			if(PID->ErrorInt<-30){PID->ErrorInt = -30;}
    }
    else
    {
        PID->ErrorInt = 0;
    }
    PID->Out = PID->Kp * PID->Error0 + PID->Ki * PID->ErrorInt + PID->Kd * (PID->Error0 - PID->Error1);
    /*输出限幅*/
    if (PID->Out > PID->OutMax)
    {
        PID->Out = PID->OutMax;
    }
    if (PID->Out < PID->OutMin)
    {
        PID->Out = PID->OutMin;
    }
}

/**************************************************************************
Function: PI调控输出�?
Input   : 入口参数需初�?�化Kp,Ki,Kd
Output  : 内置PID计算，不需再加�?PID_Control
函数功能：速度�?调控入口
入口参数�?
返回  值：�?
**************************************************************************/

void PID_PI_Control_loop(PID_Para_t *PI_Control_para, float *Speed)
{
    //    PI_Control_para->Kp = 0.38;
    //    PI_Control_para->Ki = 0.08;
    //	  PI_Control_para->Kd = 0.00;

    /*输出限幅*/
    PI_Control_para->OutMax = 16800;
    PI_Control_para->OutMin = -16800;

    PI_Control_para->Actual = *Speed;

    PID_Control(PI_Control_para);
}

/**************************************************************************
Function: PD调控输出�?
Input   : 入口参数需初�?�化Kp,Ki,Kd
Output  : 内置PID计算，不需再加�?PID_Control
函数功能：位�?�?调控入口
入口参数�?
返回  值：�?
**************************************************************************/
PID_Para_t PD_Control_para;
void PID_PD_Control_loop(PID_Para_t *PD_Control_para, float *Location)
{
    PD_Control_para->Kp = 0.3;
    PD_Control_para->Ki = 0.0;
    PD_Control_para->Kd = 0.2;

    /*输出限幅*/
    PD_Control_para->OutMax = 100;
    PD_Control_para->OutMin = -100;

    PD_Control_para->Actual = *Location;

    PID_Control(PD_Control_para);
}

/**************************************************************************

函数功能：�?�量式PI控制�?
入口参数：编码器测量�?(实际速度)，目标速度
返回  值：电机PWM
根据增量式�?�散PID�?�?
pwm+=Kp[e（k�?-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表�?次偏�?
e(k-1)代表上一次的偏差  以�?�类�?
pwm代表增量输出
在我�?的速度控制�?�?系统里面，只使用PI控制
pwm+=Kp[e（k�?-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    Last_bias = Bias; // Save the last deviation //保存上一次偏�?

    return Pwm;
}
int Incremental_PI_B(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    Last_bias = Bias; // Save the last deviation //保存上一次偏�?

    return Pwm;
}
int Incremental_PI_C(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    Last_bias = Bias; // Save the last deviation //保存上一次偏�?

    return Pwm;
}
int Incremental_PI_D(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    Last_bias = Bias; // Save the last deviation //保存上一次偏�?

    return Pwm;
}

/*新�?�PID参数测试*/
PID pid_A, pid_B, pid_C, pid_D;

void PID_Init(PID *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->IntegralLimit = integral_limit;
    pid->OutputLimit = output_limit;
    pid->Integral = 0;
    pid->LastError = 0;
    pid->Output = 0;
}

/* 新�?�PID控制函数 */
float PID_Calculate(PID *pid, float target, float current)
{
    pid->Target = target;
    pid->Current = current;
    pid->Error = pid->Target - pid->Current;

    // �?分项
    pid->Integral += pid->Error;
    // �?分限�?
    if (pid->Integral > pid->IntegralLimit)
        pid->Integral = pid->IntegralLimit;
    if (pid->Integral < -pid->IntegralLimit)
        pid->Integral = -pid->IntegralLimit;

    // �?分项
    float differential = pid->Error - pid->LastError;

    // PID计算
    pid->Output = pid->Kp * pid->Error +
                  pid->Ki * pid->Integral +
                  pid->Kd * differential;

    // 输出限幅
    if (pid->Output > pid->OutputLimit)
        pid->Output = pid->OutputLimit;
    if (pid->Output < -pid->OutputLimit)
        pid->Output = -pid->OutputLimit;

    pid->LastError = pid->Error;

    return pid->Output;
}

void PID_Reset_Integral(PID *pid)
{
    pid->Integral = 0;
    pid->LastError = 0;
    pid->Output = 0;
}

float pid_update(PID_Controller* pid, float error, float dt) 
{
    // ������
    float proportional = pid->kp * error;
    // ������
    pid->integral += error * dt;
    // �����޷�
    if (pid->integral > pid->integral_limit) 
    {
        pid->integral = pid->integral_limit;
    } 
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    
    float integral_term = pid->ki * pid->integral;
    // ΢����
    float derivative = (error - pid->prev_error) / dt;
    float derivative_term = pid->kd * derivative;
    pid->prev_error = error;
    
    // ���������
    float output = proportional + integral_term + derivative_term;
    
    // ����޷�
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}
