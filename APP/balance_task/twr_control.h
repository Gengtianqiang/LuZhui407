/******************************************************************************
 * Copyright (C) 2026, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file twr_contral.h
 * 
 * @par dependencies 
 * - main.h
 * - stm32f4xx_hal.h
 * - cmsis_os2.h
 * - semphr.h
 * 
 * @author Yharim
 * 
 * @brief TWR Control State Machine Driver | TWR控制状态机驱动
 * @details This header file defines TWR positioning, state machine and motion control parameters | 定义TWR定位、状态机及运动控制相关参数
 * @note 1 tab == 4 spaces!
 * 
 * @version V1.0 2026-3-20
 *
 *****************************************************************************/
#ifndef TWR_CONTRAL_H
#define TWR_CONTRAL_H

/**
 * @brief Header file includes | 头文件包含
 * @note HAL library + RTOS library are mandatory | HAL库与RTOS库为必选依赖
 */
//******************************** Includes *********************************//
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"  
#include "semphr.h"
#include "control.h"
#include "math.h"
#include "bsp_4g.h"
//******************************** Includes *********************************//

/**
 * @brief Macro definitions and type definitions | 宏定义和类型定义
 * @details Define state machine, coordinate and structure for TWR control | 定义TWR控制的状态机、坐标及结构体
 * @note All custom types are prefixed to avoid naming conflict | 所有自定义类型添加前缀，避免命名冲突
 */
//******************************** Defines **********************************//

/**
 * @brief State machine state enumeration | 状态机状态枚举
 * @details Define all running states of TWR control system | 定义TWR控制系统的所有运行状态
 */
typedef enum {
    STATE_IDLE                  = 0,    /* Idle state | 空闲状态 */
    STATE_PERIPHERAL_CHECK      = 1,    /* Peripheral self-check state | 外设检查状态 */
    STATE_MOVE_INIT             = 2,    /* Initial forward state | 初始前进状态 */
    STATE_IMU_CHECK             = 3,    /* IMU self-check state | IMU检查状态 */
    STATE_ROTATE_FIXED          = 4,    /* Fixed angle rotation state | 固定转角状态 */
    STATE_ROTATE_WAIT           = 5,    /* Rotation wait state | 转角等待状态 */
    STATE_MOVE_TO_TARGET        = 6,    /* Target forward state | 目标前进状态 */
	STATE_LINE_TRACKING 		= 7,    /* Line tracking state | 线跟踪状态 */
    STATE_DISTANCE_CHECK        = 8,    /* Distance check state | 距离检查状态 */
    STATE_FINISHED              = 9,    /* Task finished state | 结束状态 */
    STATE_ERROR                 = 10,   /* Error state | 错误状态 */
    STATE_PDOA                  = 11    /* Tracking state | 跟踪状态 */
} StateMachine_State_t;

/**
 * @brief TWR positioning coordinate structure | TWR位置解算坐标结构体
 * @details Store three-point positioning coordinates and angle | 存储三点定位坐标与角度信息
 */
typedef struct {
	float x;                      /* X coordinate | X坐标 */
	float y;                      /* Y坐标 | Y坐标 */
	float angle;                  /* Angle | 角度 */
} coordinate;

/**
 * @brief Midpoint state enumeration | 中点状态枚举
 * @details Define the up/down state of the midpoint | 定义中点的上下状态
 */
typedef enum {
	UP,                         /* Up state | 向上 */
	DOWN,                       /* Down state | 向下 */
	Error                       /* Error state | 错误 */
} middle_state_t;

/**
 * @brief Line parameter structure | 直线参数结构体
 * @details Store linear equation parameters and normalization coefficients | 存储直线方程参数与归一化系数
 */
typedef struct {
    float    A;                      /* Line parameter A | 直线参数A */
    float    B;                      /* Line parameter B | 直线参数B */
    float    C;                      /* Line parameter C | 直线参数C */
    float norm;                   /* Normalization coefficient = sqrt(A?+B?) | 归一化系数 */
} LineParam_t;

/**
 * @brief State machine control structure | 状态机控制结构体
 * @details Adapt to CubeMX RTOS, encapsulate all TWR control parameters | 适配CubeMX RTOS，封装TWR控制所有参数
 */
typedef struct {
    StateMachine_State_t    current_state;    /* Current state | 当前状态 */
    uint8_t             peripheral_status;             /* Peripheral status (1=normal, 0=abnormal) | 外设状态 */
    uint8_t                    imu_status;                    /* IMU status (1=normal, 0=abnormal) | IMU状态 */
    float                 target_distance;                 /* Distance to target (unit: cm) | 到终点的距离(cm) */
    osSemaphoreId_t             start_sem;             /* Start command semaphore | 启动命令信号量 */
    uint8_t                    ahand_flag;      
    uint8_t                   middle_flag;
    uint8_t                   behind_flag;       
	uint16_t                 time_counter;					        /* Timing counter | 启动定时计数信号量 */
    uint32_t               TWR_Time_count;				          /* Timing counter | 定时计数器 */
	coordinate                      coor1;					              /* Three-point coordinate 1 | 三点坐标1 */
	coordinate                      coor2;					              /* Three-point coordinate 2 | 三点坐标2 */
    coordinate                      coor3;					              /* Three-point coordinate 3 | 三点坐标3 */
    middle_state_t           middle_state;			      /* Midpoint position state | 中点位置 */
    float                  distance_coor3;				          /* Real-time distance of stage 3 | 阶段三实时距离 */
    float             fost_distance_coor3;             /* Initial distance of stage 3 | 阶段三初始距离 */
    float             last_distance_coor3;             /* Last distance of stage 3 | 阶段三上一帧距离 */
    SemaphoreHandle_t           twr_mutex;			      /* Mutex for TWR coordinates | TWR坐标互斥锁 */
} StateMachine_Handle_t;

/**
 * @brief Global variables declaration | 全局变量声明
 * @details Expose state machine handle and PID controller | 对外暴露状态机句柄与PID控制器
 */
extern StateMachine_Handle_t g_state_machine;
extern PID_Controller            heading_pid;
extern ProtocolData               proto_data;

/**
 * @brief Differential drive control function | 差速驱动控制函数
 * @param[in] pid_output  : PID calculation output | PID计算输出值
 * @param[in] base_speed  : Base speed of the car | 底盘基准速度
 * @return None
 */
void differential_drive_control(float pid_output, float base_speed);

/**
 * @brief State machine main loop function | 状态机主循环函数
 * @param[in] argument    : Pointer to state machine handle | 状态机句柄指针
 * @param[in] result      : Pointer to bracket data result | 括号数据结果指针
 * @return None
 */
void StateMachine_Loop(StateMachine_Handle_t *argument,BracketContent *result);
void StateMachine_Init(void);
bool line_param_init(StateMachine_Handle_t *hsm, LineParam_t *line);
float Yaw_PD_Ctrl(float curr_yaw, float target_yaw);
static bool calc_angle2_and_position(StateMachine_Handle_t *hsm);
float Line_Track_PD_Ctrl(LineParam_t *line, float curr_x, float curr_y);

//******************************** Defines **********************************//
#endif /* TWR_CONTRAL_H */
