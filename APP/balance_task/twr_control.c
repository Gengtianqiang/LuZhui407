/******************************************************************************
 * Copyright (C) 2026, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file twr_control.c
 * 
 * @par dependencies 
 * - twr_control.h
 * - control.h
 * - math.h
 * - cmsis_os2.h
 * - semphr.h
 * 
 * @author Yharim
 * 
 * @brief TWR Positioning Control State Machine | TWR定位控制状态机实现
 * @details Implement TWR positioning, linear tracking, PD control and state machine scheduling | 实现TWR定位、直线跟踪、PD控制及状态机调度
 * @note 1 tab == 4 spaces! RTOS-based embedded motion control | 基于RTOS的嵌入式运动控制
 * 
 * @version V1.0 2026-3-20
 *
 *****************************************************************************/

/**
 * @brief Header file includes | 头文件包含
 */
//******************************** Includes *********************************//
#include "twr_control.h"
#include "pdoa_control.h"
//******************************** Includes *********************************//

/************************ TWR定位状态机全局变量 ************************/
/**
 * @brief Global state machine handle | 全局状态机句柄
 */
StateMachine_Handle_t g_state_machine;

/**
 * @brief RTOS static mutex buffer | RTOS静态互斥锁缓存
 */
static StaticSemaphore_t twrMutexBuffer;

/**
 * @brief Global reference line parameters | 全局参考直线参数
 */
static LineParam_t g_ref_line = {
  .A    = 0,
  .B    = 0,
  .C    = 0,
  .norm = 0
};


PID_Controller twr_pid = {
   .kp = 0.5f,           // 比例增益 - 需要调试
   .ki = 0.01f,          // 积分增益 - 需要调试  
   .kd = 0.02f,           // 微分增益 - 需要调试
   .integral = 0.0f,
   .prev_error = 0.0f,
   .integral_limit = 10.0f,  // 积分限幅
   .output_limit = 6.0f     // 输出限幅
};

/************************ Line Tracking PD Controller Parameters ************************/
/**
 * @brief Line tracking PD controller | 线跟踪PD控制器参数
 */
#define TRACK_KP          0.6f      /* Proportional coefficient: larger deviation = faster steering | 比例系数：偏离越大，转向越狠 */
#define TRACK_KD          0.05f     /* Differential coefficient: suppress oscillation | 微分系数：抑制摆动 */
#define OFFSET_THRESH     0.05f     /* Deviation dead zone (5cm) | 偏离量死区（5cm，根据定位精度调整） */
#define MAX_Z_SPEED       0.3f      /* Maximum correction steering speed | 最大纠偏转向速度 */

/**
 * @brief Yaw angle PD controller | 偏航角PD控制器参数
 */
float YAW_KP             = 0.1f;    /* Yaw angle proportional coefficient | 偏航角比例系数（可调试） */
float YAW_KD             = 0.0f;   /* Yaw angle differential coefficient | 偏航角微分系数（可调试） */
float MAX_YAW_Z_SPEED    = 0.2f;    /* Maximum yaw correction speed | 偏航角纠偏最大转向速度 */
float YAW_OFFSET_THRESH  = 5.0f;    /* Yaw angle dead zone (degree) | 偏航角死区阈值（度） */


/************************ State Machine Handler Functions (Core Logic) ************************/
/**
 * @brief Idle state handler | 空闲状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_IdleHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    static uint16_t begin_timecounter = 0;
    // Release start semaphore
    osSemaphoreRelease(hsm->start_sem);
    begin_timecounter ++;
    if(begin_timecounter>1000) {
        // Check trigger signal, TWR data valid
         if(my_4g_dtu.start_flag && bracket_data.twr_status) {
        // if(my_4g_dtu.start_flag) {
        //     // Switch to peripheral check state

        //     //unit test
        //     hsm->current_state = STATE_FINISHED;
        //     hsm->ahand_flag = 1;

            //op
            hsm->current_state = STATE_PERIPHERAL_CHECK;
            hsm->coor3.x = my_4g_dtu.point.x;
            hsm->coor3.y = my_4g_dtu.point.y;
            
            // Record the first coordinate point
            hsm->coor1.x = res->x;
            hsm->coor1.y = res->y;
        }
    }
}

/**
 * @brief Peripheral check state handler | 外设检查状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_PeripheralCheckHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    // Check peripheral status (UWB, etc.)
    if (1) {
        // Peripheral normal, switch to move init state
        hsm->current_state = STATE_MOVE_INIT;
    } else {
        // Peripheral error, switch to error state
        hsm->current_state = STATE_ERROR;
    }
}

/**
 * @brief Move initialization state handler | 移动初始化状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_MoveInitHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    if(hsm->time_counter < 200 ) {
        Move_X = -0.4f;
        hsm->time_counter++;
    // After 1 second
    } else if(bracket_data.twr_status) {
        Move_X = 0.0f;
        hsm->ahand_flag = 1;  
        // Record coordinates after forward movement
        hsm->coor2.x = res->x;
        hsm->coor2.y = res->y;
        hsm->coor2.angle = myimu.euler_yaw_Cali;
        
        // Calculate rotation angle
        hsm->coor3.angle = atan2(hsm->coor2.x-hsm->coor1.x, hsm->coor2.y-hsm->coor1.y);
        if(calc_angle2_and_position (hsm));
        hsm->current_state = STATE_IMU_CHECK;
    }
}

/**
 * @brief IMU check state handler | IMU检查状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_ImuCheckHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    // Check IMU status
    if (1) {
        // IMU normal, switch to fixed rotation state
        hsm->current_state = STATE_ROTATE_FIXED;
    } else {
        // IMU error, switch to error state
        hsm->current_state = STATE_ERROR;
    }
}

/**
 * @brief Fixed rotation state handler | 固定转角状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static uint32_t fixed_timecounter = 0;
static void StateMachine_RotateFixedHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    float add_angle =0;
    float angle_error = 0;
    if(hsm->middle_state == UP) { // Clockwise rotation
        add_angle = (hsm->coor2.angle - hsm->coor3.angle);
        if(add_angle<-180)add_angle += 360;
    }
    // Counterclockwise rotation
    else if(hsm->middle_state == DOWN) {
        add_angle = (hsm->coor2.angle + hsm->coor3.angle);
        if(add_angle>180)add_angle -= 360;
    }

    angle_error = add_angle - myimu.euler_yaw_Cali;

    /* Execute steering */
    float pid_output = pid_update(&twr_pid, angle_error, 1.0);
    if (fabs(pid_output) < 1.0f)
    {
        pid_output = 0;
    }
    
    Move_X = 0;
    Move_Z = pid_output/10;

    if(fabs(angle_error) < 10.0f)
    {
        fixed_timecounter ++;
        if(fixed_timecounter>300) {
            Move_X = 0.0f;
            hsm->coor2.x = res->x;
            hsm->coor2.y = res->y;
            // Initialize reference line
            if(line_param_init(hsm, &g_ref_line))
            {
                hsm->coor2.angle = myimu.euler_yaw_Cali;
                hsm->current_state = STATE_LINE_TRACKING;
            }
            else
            {
                hsm->current_state = STATE_ERROR;
            }

            heading_pid.integral = 0;
            Move_X = 0;
            Move_Z = 0;
            hsm->time_counter = 0;
            Buzzer_Start_Once(10);
            return ;
        }
    }
}

/**
 * @brief Rotation wait state handler | 旋转等待状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
// static void StateMachine_RotateWaitHandler(StateMachine_Handle_t *hsm,BracketContent *res)
// {
//     // Wait 0.5s, switch to target move state
//     if(1)
//         hsm->current_state = STATE_MOVE_TO_TARGET;
// }

/**
 * @brief Line tracking state handler | 线跟踪状态处理函数
 * @param[in] hsm       : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res       : Pointer to bracket data | 括号数据指针
 * @param[in] ref_line  : Pointer to reference line | 参考直线指针
 * @return None
 */
static void StateMachine_LineTrackingHandler(StateMachine_Handle_t *hsm, BracketContent *res, LineParam_t *ref_line)
{
    // Fixed forward speed
    Move_X = -0.5f;
    // PD control: fusion of line tracking and yaw control
        Move_Z = Line_Track_PD_Ctrl(ref_line, res->x, res->y) * 0.6f
                + Yaw_PD_Ctrl(myimu.euler_yaw_Cali, hsm->coor2.angle) * 0.4f;

    // Check target arrival
    float dx = res->x - hsm->coor3.x;
    float dy = res->y - hsm->coor3.y;
    float dist_to_target = sqrtf(dx*dx + dy*dy);


    static uint32_t last_titks = 0;
    static float    now_dist   = 10000;
    static float   last_dist   = 10000;
    uint32_t now_titks = osKernelGetTickCount();
    if(now_titks-last_titks>2000) {
        last_titks =      now_titks;
        last_dist  =       now_dist;
        now_dist   = dist_to_target;
    }

    if(now_dist>last_dist) {
        Move_Z = 0;
        Move_X = 0;
        hsm->current_state = STATE_FINISHED;
    }
    

    if(dist_to_target < 0.5f)
    {
        Move_Z = 0;
        Move_X = 0;
        hsm->current_state = STATE_FINISHED;
        // hsm->ahand_flag = 1;
        Buzzer_Start_Once(10);
    }
}

/**
 * @brief Target move state handler | 目标前进状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
// static void StateMachine_MoveToTargetHandler(StateMachine_Handle_t *hsm, BracketContent *res)
// {

// }

/**
 * @brief Finished state handler | 完成状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_FinishedHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    Move_X = 0.0;
    
    // static uint16_t finish_time = 0;
    // finish_time ++;
	// Buzzer_Start_Once(10);
    
    Move_Z = 0;
//    //测试
//    osSemaphoreAcquire (hsm->uart_sem,0);
    //准备与串口任务通信：向后车和中间车发送返回信号
     hsm->current_state = STATE_PDOA;
}

/**
 * @brief PDOA tracking state handler | PDOA跟踪状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_PdoaHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{

    if(1==my_4g_dtu.return_flag) {

        //收到返回指令
        hsm->ahand_flag = 3;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
        pdoa_follow(&proto_data);
    }

	//Buzzer_Start_Once(10);
}

/**
 * @brief Error state handler | 错误状态处理函数
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] res   : Pointer to bracket data | 括号数据指针
 * @return None
 */
static void StateMachine_ErrorHandler(StateMachine_Handle_t *hsm,BracketContent *res)
{
    Buzzer_Start_Once(10);
    Move_X = 0.0;
}


/************************ TWR Positioning State Machine Tool Functions ************************/
/**
 * @brief Initialize reference line from start to target | 初始化起点到终点的参考直线
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @param[in] line  : Pointer to line parameter structure | 直线参数结构体指针
 * @return bool     : Initialization result (true=success) | 初始化结果
 */
bool line_param_init(StateMachine_Handle_t *hsm, LineParam_t *line)
{
    if(hsm == NULL || line == NULL) return false;
    float x1 = hsm->coor2.x, y1 = hsm->coor2.y;
    float x3 = hsm->coor3.x, y3 = hsm->coor3.y;

    line->A = y3 - y1;
    line->B = x1 - x3;
    line->C = x3*y1 - x1*y3;
    line->norm = sqrtf(line->A*line->A + line->B*line->B);

    // Avoid zero line length (start point = target point)
    if(line->norm < 0.01f) return false;
    return true;
}

/**
 * @brief Calculate signed offset from current point to reference line | 计算当前点到参考线的带符号偏离量
 * @param[in] line  : Pointer to line parameter structure | 直线参数结构体指针
 * @param[in] x     : Current X coordinate | 当前X坐标
 * @param[in] y     : Current Y coordinate | 当前Y坐标
 * @return float    : Signed offset value | 带符号偏离量
 */
float calc_line_offset(LineParam_t *line, float x, float y)
{
    if(line == NULL) return 0.0f;
    // Signed offset: sign indicates direction, absolute value indicates distance
    return (line->A * x + line->B * y + line->C) / line->norm;
}

/**
 * @brief Yaw angle PD controller | 偏航角PD控制器
 * @param[in] curr_yaw   : Current yaw angle | 当前偏航角
 * @param[in] target_yaw : Target yaw angle | 目标偏航角
 * @return float         : PD controller output | PD控制器输出值
 */
float curr_yaw_error        = 0.0f;               /* Current yaw error | 毡前偏诮迖铣赢 */
float Yaw_PD_Ctrl(float curr_yaw, float target_yaw)
{
    static float last_yaw_error = 0.0f;               /* Last yaw error | 上一次偏航角误差 */
    
    float pd_output             = 0.0f;               /* PD output | PD控制器输出 */

    // 1. Calculate current error
    curr_yaw_error = curr_yaw - target_yaw;

    if(curr_yaw_error<=-180) curr_yaw_error += 360;
    if(curr_yaw_error>= 180) curr_yaw_error -= 360;

    // 2. PD correction only when error exceeds threshold
    if(fabs(curr_yaw_error) > YAW_OFFSET_THRESH)
    {
        // 3. PD control formula
        pd_output = YAW_KP * curr_yaw_error + YAW_KD * (curr_yaw_error - last_yaw_error);
        
        // 4. Speed limit
        pd_output = (pd_output > MAX_YAW_Z_SPEED)  ?  MAX_YAW_Z_SPEED : pd_output;
        pd_output = (pd_output < -MAX_YAW_Z_SPEED) ? -MAX_YAW_Z_SPEED : pd_output;
    }
    else
    {
        pd_output = 0.0f;
    }

    // 5. Update last error
    last_yaw_error = curr_yaw_error;
    return -pd_output;
}

/**
 * @brief Line tracking PD controller | 线跟踪PD控制器
 * @param[in] line    : Pointer to reference line | 参考直线指针
 * @param[in] curr_x  : Current X coordinate | 当前X坐标
 * @param[in] curr_y  : Current Y coordinate | 当前Y坐标
 * @return float      : PD controller output | PD控制器输出值
 */
float Line_Track_PD_Ctrl(LineParam_t *line, float curr_x, float curr_y)
{
    static float last_offset = 0.0f;   /* Last offset value | 上一次偏离量 */
    float curr_offset = calc_line_offset(line, curr_x, curr_y);
    float pd_output = 0.0f;

    // Correction only when offset exceeds threshold
    if(fabs(curr_offset) > OFFSET_THRESH)
    {
        // PD control formula
        pd_output = TRACK_KP * curr_offset + TRACK_KD * (curr_offset - last_offset);
        // Speed limit
        pd_output = (pd_output > MAX_Z_SPEED) ? MAX_Z_SPEED : pd_output;
        pd_output = (pd_output < -MAX_Z_SPEED) ? -MAX_Z_SPEED : pd_output;
    }
    else
    {
        pd_output = 0.0f;
    }

    last_offset = curr_offset;
    return pd_output;
}

/**
 * @brief Calculate angle and position with three points | 三点坐标计算角度与位置关系
 * @param[in] hsm   : Pointer to state machine handle | 状态机句柄指针
 * @return bool     : Calculation result (true=success) | 计算结果
 */
static bool calc_angle2_and_position(StateMachine_Handle_t *hsm)
{

    /* 1. check */
    if (hsm == NULL) return false;

    /* 2. Get three coordinate points */
    float x1 = hsm->coor1.x, y1 = hsm->coor1.y;
    float x2 = hsm->coor2.x, y2 = hsm->coor2.y;
    float x3 = hsm->coor3.x, y3 = hsm->coor3.y;

    /* 3. Calculate vectors v21 and v23 */
    float v21_x = x1 - x2;
    float v21_y = y1 - y2;
    float v23_x = x3 - x2;
    float v23_y = y3 - y2;

    /* 4. Vector length check */
    float len_v21 = sqrtf(v21_x*v21_x + v21_y*v21_y);
    float len_v23 = sqrtf(v23_x*v23_x + v23_y*v23_y);
    if (len_v21 < 1e-6f || len_v23 < 1e-6f) {
        hsm->current_state = STATE_ERROR;
        return false;
    }

    /* 5. Calculate vector angle (Law of Cosines) */
    float dot_product = v21_x*v23_x + v21_y*v23_y;
    float cos_theta = dot_product / (len_v21 * len_v23);
    // Limit cos range to avoid precision errors
    cos_theta = (cos_theta > 1.0f) ? 1.0f : (cos_theta < -1.0f) ? -1.0f : cos_theta;
    hsm->coor3.angle = 180 - acosf(cos_theta) * (180.0f / 3.1415f);

    /* 6. Judge middle point position (UP/DOWN) */
    float v13_x = x3 - x1;
    float v13_y = y3 - y1;
    float v12_x = x2 - x1;
    float v12_y = y2 - y1;
    
    float cross_product = v13_x * v12_y - v13_y * v12_x;
    if (cross_product > 1e-6f) {
        hsm->middle_state = UP;
    } else if (cross_product < -1e-6f) {
        hsm->middle_state = DOWN;
    } else {
        hsm->middle_state = Error;
    }

    return true;
}


/************************ TWR State Machine Main Loop ************************/
/**
 * @brief State machine main scheduling function | 状态机主调度函数
 * @param[in] hsm       : Pointer to state machine handle | 状态机句柄指针
 * @param[in] result    : Pointer to positioning data | 定位数据指针
 * @return None
 */
void StateMachine_Loop(StateMachine_Handle_t *hsm ,BracketContent *result)
{
    switch (hsm->current_state) {
        case STATE_IDLE:
            StateMachine_IdleHandler(hsm,result);
            break;
        case STATE_PERIPHERAL_CHECK:
            StateMachine_PeripheralCheckHandler(hsm,result);
            break;
        case STATE_MOVE_INIT:
            StateMachine_MoveInitHandler(hsm,result);
            break;
        case STATE_IMU_CHECK:
            StateMachine_ImuCheckHandler(hsm,result);
            break;
        case STATE_ROTATE_FIXED:
            StateMachine_RotateFixedHandler(hsm, result);
            break;
        case STATE_LINE_TRACKING:
            StateMachine_LineTrackingHandler(hsm, result, &g_ref_line);
            break;
        case STATE_FINISHED:
            StateMachine_FinishedHandler(hsm,result);
            break;
        case STATE_PDOA:
            StateMachine_PdoaHandler(hsm,result);
            break;
        case STATE_ERROR:
            StateMachine_ErrorHandler(hsm,result);
            break;
        default:
            hsm->current_state = STATE_ERROR;
            break;
    }
    if(1!=my_4g_dtu.return_flag) {
        Drive_Motor(Move_X,0.0f,Move_Z);
        Motor_Task_Loop();
    }
}

/************************ State Machine Initialization ************************/
/**
 * @brief State machine initialization (RTOS adapted) | 状态机初始化（适配RTOS）
 * @param None
 * @return None
 */
void StateMachine_Init(void)
{
    // ---------------------- RTOS Resources ----------------------
    // Create start semaphore
    g_state_machine.start_sem = osSemaphoreNew(1U, 0U, NULL);

    // Initialize time counter
    g_state_machine.time_counter = 0;
    // Create static mutex
    g_state_machine.twr_mutex = xSemaphoreCreateMutexStatic(&twrMutexBuffer);
    
    // ---------------------- State Initialization ----------------------
    // Set initial state to idle
    g_state_machine.current_state = STATE_IDLE;
    // Initialize target coordinate 3
    g_state_machine.coor3.x = my_4g_dtu.point.x;
    g_state_machine.coor3.y = my_4g_dtu.point.y;
    g_state_machine.last_distance_coor3 = 0;
    g_state_machine.distance_coor3 = 0;

    g_state_machine.ahand_flag = 0;
    g_state_machine.middle_flag = 0;
    g_state_machine.behind_flag = 0;
}
