#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f4xx_hal.h"
#include "main.h"


struct Motor_x
{
    uint32_t pwm_compare;
    int32_t  direct;

//    float target_speed;    // 目标速度
//    float actual_speed;    // 实际速度（从编码器读取）
//    float pwm_output;      // PID计算出的PWM输出
};


typedef struct Motor_Pwm_HandleTypeDef
{
    bool pwm_running;
    uint32_t pwm_max_count;

    

    struct Motor_x A,B,C,D;

    
}Motor_t;

// 在Motor_Instance结构体定义中添加
// typedef struct {
//     // 原有字段
//     uint8_t direct;
//     uint32_t pwm_compare;
//     bool pwm_running;
    
//     // 新增字段用于PID控制
//     float target_speed;    // 目标速度
//     float actual_speed;    // 实际速度（从编码器读取）
//     float pwm_output;      // PID计算出的PWM输出
    
// } Motor_Instance_TypeDef;


#define  Motor_A   TIM_CHANNEL_4
#define  Motor_B   TIM_CHANNEL_3
#define  Motor_C   TIM_CHANNEL_2
#define  Motor_D   TIM_CHANNEL_1

/*--------Motor_A control pins--------*/
#define PWM_PORTA GPIOC			 //PWMA
#define PWM_PIN_A GPIO_Pin_9//PWMA
#define PWMA 	  TIM8->CCR4	 //PWMA

#define IN1_PORTA GPIOB			 //AIN1
#define IN1_PIN_A GPIO_Pin_13 //AIN1
#define AIN1 	  PBout(13)		 //AIN1

#define IN2_PORTA GPIOB			 //AIN2
#define IN2_PIN_A GPIO_Pin_12 //AIN2
#define AIN2 	  PBout(12)		 //AIN2
/*------------------------------------*/

/*--------Motor_B control pins--------*/
#define PWM_PORTB GPIOC			 //PWMB
#define PWM_PIN_B GPIO_Pin_8 //PWMB
#define PWMB 	  TIM8->CCR3	 //PWMB

#define IN1_PORTB GPIOC			 //BIN1
#define IN1_PIN_B GPIO_Pin_0 //BIN1
#define BIN1 	  PCout(0)		 //BIN1

#define IN2_PORTB GPIOB			 //BIN2
#define IN2_PIN_B GPIO_Pin_14 //BIN2
#define BIN2 	  PBout(14)		 //BIN2
/*------------------------------------*/

/*--------Motor_C control pins--------*/
#define PWM_PORTC GPIOC			 //PWMC
#define PWM_PIN_C GPIO_Pin_7 //PWMC
#define PWMC 	  TIM8->CCR2	 //PWMC

#define IN1_PORTC GPIOD			  //CIN1
#define IN1_PIN_C GPIO_Pin_10	//CIN1
#define CIN1 	  PDout(10)		  //CIN1

#define IN2_PORTC GPIOD			 //CIN2
#define IN2_PIN_C GPIO_Pin_12 //CIN2
#define CIN2 	  PDout(12)		 //CIN2
/*------------------------------------*/

/*--------Motor_D control pins--------*/
#define PWM_PORTD GPIOC			 //PWMD
#define PWM_PIN_D GPIO_Pin_6 //PWMD
#define PWMD 	  TIM8->CCR1	 //PWMD

#define IN1_PORTD GPIOC			  //DIN1
#define IN1_PIN_D GPIO_Pin_12	//DIN1
#define DIN1 	  PCout(12)		  //DIN1

#define IN2_PORTD GPIOA			  //DIN2
#define IN2_PIN_D GPIO_Pin_8	//DIN2
#define DIN2 	  PAout(8)		  //DIN2




/*11/12增*/
void Motor_Init(void);

bool Get_Motor_Enable(void);
uint32_t Get_Motor_PwmMaxCount(void);
uint32_t Get_Motor_PwmCompare(uint32_t TIM_CHANNEL);
void Set_Motor_PWM(uint32_t Motor , int32_t Direct_Compare);

void Motor_Pwm_Start(void);
void Motor_Pwm_Stop(void);
void Motor_Baker(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);

void Motor_Task_Loop(void);
void Motor_Stop_Loop(void);
#endif
