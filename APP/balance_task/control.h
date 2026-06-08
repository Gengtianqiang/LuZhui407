#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "filter.h"
#include <stdbool.h>

//高速挡和低速挡，两�?档位，操控不�?
#define RC_Velocity_GAOSU 6000.0f
#define RC_Velocity_DISU  1300.0f

extern float Velocity_KP, Velocity_KI;
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
// Encoder accuracy
// 编码器精�?
extern float Encoder_precision;
// Wheel circumference, unit: m
// �?子周长，单位：m
extern float Wheel_perimeter;
// Drive wheel base, unit: m
// 主动�?�?距，单位：m
extern float Wheel_spacing;
// The wheelbase of the front and rear axles of the trolley, unit: m
// 小车前后轴的轴距，单位：m
extern float Wheel_axlespacing;
// All-directional wheel turning radius, unit: m
// 全向�?�?�?半径，单位：m
extern float Omni_turn_radiaus;
// The encoder octave depends on the encoder initialization Settings
// 编码器倍�?�数，取决于编码器初始化设置
#define EncoderMultiples 4
// Encoder data reading frequency
// 编码器数�?读取频率
#define CONTROL_FREQUENCY 100
/************ 小车型号相关变量 **************************/



//Motor speed control related parameters of the structure
//电机速度控制相关参数结构�?
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Location;
	float Encoder_Rpm;   //Encoder value, motor real-time speed, unit: rpm //编码器数值，电机实时速度，单位：rpm
	float Control_Rpm;   //Control the target speed of the brushless motor //控制无刷电机的目标转�?
	int32_t Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机�?标速度值，控制电机�?标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, 4wd_Car is half wheelspacing //�?�? 四驱车为半轮�?
  float AxleSpacing;       //Axlespacing, 4wd_Car is half axlespacing //轴距 四驱车为半轴�?	
  int GearRatio;           //Motor_gear_ratio //电机减速比
  int EncoderAccuracy;     //Number_of_encoder_lines //编码器精�?(编码器线�?)
  float WheelDiameter;     //Diameter of driving wheel //主动�?直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向�?小车旋转半径
}Robot_Parament_InitTypeDef;


float My_PID_WEIZHI_L(float Qiwang,float Shiji,float kP,float kI,float kD);
#define FILTER_SIZE_LinearAccX 5
static float acc_buffer[FILTER_SIZE_LinearAccX] = {0};
static int acc_index = 0;
static float moving_average_filter_LinearAccX(float new_acc) {
    acc_buffer[acc_index] = new_acc;
    acc_index = (acc_index + 1) % FILTER_SIZE_LinearAccX;  // �?�?存储

    float sum = 0.0f;
    for (int i = 0; i < FILTER_SIZE_LinearAccX; i++) {
        sum += acc_buffer[i];
    }
    return sum / FILTER_SIZE_LinearAccX;
}

#define acc_ALPHA 0.2f  // 0 < ALPHA < 1，值越小，平滑度越�?
static float exp_avg_LinearAccX(float new_value, float prev_avg) {
    return acc_ALPHA * new_value + (1.0f - acc_ALPHA) * prev_avg;
}

#define HPF_ALPHA 0.98f  // 高通滤波系�?
static float hpf_prev_acc_x = 0.0f;
static float hpf_acc_x = 0.0f;
static float high_pass_filter(float new_acc_x) {
    hpf_acc_x = HPF_ALPHA * (hpf_acc_x + new_acc_x - hpf_prev_acc_x);
    hpf_prev_acc_x = new_acc_x;
	return hpf_acc_x;
}
// typedef struct {
//     float alpha;    // 高通滤波系�?
//     float prev_input;  // 上一次输入数�?
//     float prev_output; // 上一次输出数�?
// } HighPassFilter;
// static HighPassFilter hpf_accel = {0};
// // 初�?�化高通滤波器
// static void hp_filter_init(HighPassFilter *hp, float cutoff_freq, float sample_rate) {
//     float dt = 1.0f / sample_rate;
//     float RC = 1.0f / (2.0f * 3.1415926535f * cutoff_freq);
//     hp->alpha = RC / (RC + dt);
//     hp->prev_input = 0.0f;
//     hp->prev_output = 0.0f;
// }
// // 执�?�高通滤�?
// static float hp_filter_apply(HighPassFilter *hp, float input) {
//     float output = hp->alpha * (hp->prev_output + input - hp->prev_input);
//     hp->prev_input = input;
//     hp->prev_output = output;
//     return output;
// }

// typedef struct {
//     float beta;     // 低通滤波系�?
//     float prev_output;
// } LowPassFilter;
// static LowPassFilter lpf_accel = {0};
// // 初�?�化低通滤波器
// static void lp_filter_init(LowPassFilter *lp, float cutoff_freq, float sample_rate) {
//     float dt = 1.0f / sample_rate;
//     float RC = 1.0f / (2.0f * 3.1415926535f * cutoff_freq);
//     lp->beta = dt / (RC + dt);
//     lp->prev_output = 0.0f;
// }
// // 执�?�低通滤�?
// static float lp_filter_apply(LowPassFilter *lp, float input) {
//     lp->prev_output = (1.0f - lp->beta) * lp->prev_output + lp->beta * input;
//     return lp->prev_output;
// }


/*新�??*/
void BLE_PID_control(void);
void Drive_Motor1(float Vx, float Vy, float Vz);


void BLE_control(void);
void BLE_follow_control(void);

//内部�?�?
void Car_init(void);
int32_t Incremental_PI_Left ( float Encoder, float Target );
int32_t Incremental_PI_Right ( float Encoder, float Target );
int32_t Incremental_no_PI (char ABCD, float Target);
void Drive_Motor(float Vx,float Vy,float Vz);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Smooth_control(float vx,float vy,float vz);
void Robot_Init(float wheelspacing, float axlespacing, int gearratio, int Accuracy, float tyre_diameter);
void differential_drive_control(float pid_output, float base_speed) ;

extern float Move_X, Move_Z;
typedef struct {
    float Kp;         // 比例增益
    float pwm_output; // PWM 输出
} SpeedPController;
static SpeedPController  left={0} , right={0};
// **初�?�化 P 控制�?**
static void SpeedP_Init(SpeedPController *p, float kp) {
    p->Kp = kp;
    p->pwm_output = 0;
}
// **更新 P 控制�?**
static float SpeedP_Update(SpeedPController *p, float v_target, float v_current) {
    float error = v_target - v_current; // 计算�?�?
    float pwm = p->Kp * error;          // 计算 PWM 输出
    // 限制 PWM 输出范围，避免超�? 0-100%
    if (pwm > 100) pwm = 100;
    if (pwm < 0) pwm = 0;
    p->pwm_output = pwm;
    return pwm;
}



#endif // CONTROL_H
