#ifndef __PID_H_
#define __PID_H_
#include "main.h"
/*2025/10/13ï¿½ï¿½ï¿½Â´ï¿½ï¿½ë²¿ï¿½ï¿½*/
typedef struct PID_Parameter
{
    float Kp,Ki,Kd;

    float Target,Actual,Out;

    float Error0,Error1,ErrorInt;

    float OutMax,OutMin;
}PID_Para_t;


typedef struct {
    float Target;       // ç›? ‡å€?
    float Current;      // å½“å‰å€?
    float Error;        // è¯?·®
    float LastError;    // ä¸Šæ?è¯?·®
    float Integral;     // ç§?ˆ†é¡?
    float Kp, Ki, Kd;   // PIDå‚æ•°
    float Output;       // è¾“å‡ºå€?
    float IntegralLimit; // ç§?ˆ†é™å¹…
    float OutputLimit;  // è¾“å‡ºé™å¹…
} PID;

typedef struct {

    float kp, ki, kd;   // PIDå‚æ•°
    float integral;       // è¾“å‡ºå€?
    float prev_error; // ç§?ˆ†é™å¹…
    float integral_limit;  // è¾“å‡ºé™å¹…
		float output_limit;
	float dt;
} PID_Controller;
extern PID_Controller heading_pid;

void PID_Control(PID_Para_t *PID);
void PID_PI_Control_loop(PID_Para_t *PI_Control_para ,float *Speed);
void PID_PD_Control_loop(PID_Para_t *PI_Control_para ,float *Location);
/**/

int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);





float pid_update(PID_Controller* pid, float error, float dt) ;

float PID_Calculate(PID *pid, float target, float current);
void PID_Init(PID *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void PID_Reset_Integral(PID *pid);

float pid_update(PID_Controller* pid, float error, float dt) ;
#endif



