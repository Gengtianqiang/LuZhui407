/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file System_Init.c
 *
 * @par dependencies
 * - System_Init.h
 * - twr_control.h
 * - freertos.h
 * - queue.h
 * - jdy_driver.h
 * - mesh_mode.h
 * - NRF24L01.h
 * - Vofa.h
 * - uart_callback.h
 *
 * @author Yharim
 *
 * @brief System initialization implementation | 系统初始化实现
 * @details This source file implements system-level hardware initialization, peripheral configuration and RTOS resource creation | 此源文件实现系统级硬件初始化、外设配置及RTOS资源创建
 * @note 1 tab == 4 spaces! This is the system startup entry for all hardware modules | 这是所有硬件模块的系统启动入口
 *
 * @version V1.0 2026-3-20
 *
 *****************************************************************************/

/**
 * @brief Header file includes | 头文件包含
 * @details Include necessary header files for system initialization | 包含系统初始化所需的必要头文件
 * @note All BSP and APP layer headers are mandatory | 所有BSP层与APP层头文件均为必选依赖
 */
//******************************** Includes *********************************//
#include "System_Init.h"
#include "twr_control.h"
#include "freertos.h"
#include "queue.h"
#include "jdy_driver.h"
#include "mesh_mode.h"
#include "NRF24L01.h"
#include "Vofa.h"
#include "uart_callback.h"
//******************************** Includes *********************************//

/**
 * @brief Macro definitions and global variables | 宏定义与全局变量
 * @details Define global instances and RTOS handles for system initialization | 定义系统初始化的全局实例与RTOS句柄
 * @note All global variables are zero-initialized by default | 所有全局变量默认初始化为0
 */
//******************************** Defines **********************************//



//******************************** Defines **********************************//

//******************************** Function Implementations ******************//

/**
 * @brief System hardware initialization function | 系统硬件初始化函数
 * @details Initialize all hardware peripherals in sequence: motor, LED, UART, buzzer, battery, encoder, NRF24L01, ICM20948 and state machine | 按顺序初始化所有硬件外设：电机、LED、串口、蜂鸣器、电池、编码器、NRF24L01、ICM20948及状态机
 * @param None
 * @return None
 * @note This function is called once at system startup | 此函数在系统启动时调用一次
 */
void systemInit(void)
{
    /*************1. 小车参数初始化**************/
    Car_init();
    /*************1. 电机初始化**************/
    Motor_Init();

    /*************2. LED initialization | LED初始化**************/
    LED_Init();
    /*************2. RGB initialization | RGB初始化**************/
    Set_LED_State(&led_R, Off,   500);
    Set_LED_State(&led_G, blink, 500);
    Set_LED_State(&led_B, Off,   500);

    /*************3. UART initialization | 串口初始化**************/
    MY_UART_Init();

    /*************4. Buzzer self-test | 蜂鸣器自检**************/
    Buzzer_Start_Duration(100, 150, 400);
    Vofa_Printf(&VOFA3, "Vofa+ Uart3 Init Success!\r\n");

    /*************5. Battery voltage check | 电池电压检测**************/
    Vofa_Printf(&VOFA3, "Battery Voltage: %.03fV\r\n", Get_battery_volt());

    /*************6. encoder initialization | 编码器初始化**************/
    HAL_Delay (300);
    Encoder_Init ();

    /*************7. NRF24L01 wireless module initialization | NRF24L01无线模块初始化**************/
    NRF24L01_Init();
    
    /*************7. JDY28M initialization | JDY28M初始化**************/
    JDY_Task_Init(&myJDY, ForwardData_On);
    /*************8. ICM20948 IMU initialization with retry | ICM20948 IMU初始化（含重试）**************/
    bool is_icm_success = false;

    is_icm_success = ICM20948_APP_Init();

    int i = 1;
    while (is_icm_success == false) {
        HAL_Delay(10);
        is_icm_success = ICM20948_APP_Init();
        i++;
        if (i > 2) {
            if (is_icm_success == false) {
                my_4g_dtu.imu_error_flag = 1;
            } else {
                my_4g_dtu.imu_error_flag = 0;
            }
            i = 1;
            break;
        }
    }

    /*************9. UWB GPIO pin configuration | UWB GPIO引脚配置**************/
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
#ifdef AHAND_CAR
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,   GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
#endif

    /*************10. State machine initialization | TWR状态机初始化**************/
    StateMachine_Init();
}



/**
 * @brief UWB module configuration instance | UWB模块配置实例
 * @details Stores PDOA/TWR timing and state for UWB module initialization | 存储UWB模块初始化的PDOA/TWR时序与状态
 */
uwb_set uwb_set_t = {
    .pdoa_time  = 0,
    .twr_time   = 0,
    .pdoa_state = false,
    .twr_state  = false,
};

/**
 * @brief UWB module state management function | UWB模块状态管理函数
 * @details Monitor PDOA/TWR initialization timeout and set module states accordingly | 监控PDOA/TWR初始化超时并设置模块状态
 * @param None
 * @return bool : true if both PDOA and TWR are ready, false otherwise | true表示PDOA和TWR均已就绪，false表示未就绪
 * @note Both PDOA and TWR states are checked independently | PDOA与TWR状态独立检查
 */
bool uwb_set_state(void)
{
    /*************1. Check if both not ready yet | 检查两者是否均未就绪**************/
    if (!uwb_set_t.pdoa_state && !uwb_set_t.twr_state) {
        uwb_set_t.pdoa_time++;
        uwb_set_t.twr_time++;

        /*************2. Timeout: force both states to ready | 超时：强制设置两者为就绪**************/
        if (uwb_set_t.pdoa_time > 100) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
            uwb_set_t.pdoa_state = true;
            uwb_set_t.twr_state  = true;
        }
    }

    /*************3. Return combined ready state | 返回组合就绪状态**************/
    if (uwb_set_t.pdoa_state && uwb_set_t.twr_state) {
        return true;
    }
    return false;
}

/**
 * @brief RTOS resource initialization function | RTOS资源初始化函数
 * @details Create RTOS kernel objects such as queues for inter-task communication | 创建任务间通信所需的RTOS内核对象（如队列）
 * @param None
 * @return None
 * @note Must be called before scheduler starts | 必须在调度器启动前调用
 */


//******************************** Function Implementations ******************//