/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file jdy_driver.h
 *
 * @par dependencies
 * - stdio.h
 * - stdint.h
 * - stdbool.h
 * - stm32f4xx_hal.h
 *
 * @author Yharim
 *
 * @brief HAL level JDY-28M driver main interface | HAL层JDY-28M驱动主接口
 * @details This header file encapsulates all status, parameters and interfaces of JDY-28M BLE/MESH module driver | 此头文件封装JDY-28M蓝牙MESH模块驱动的所有状态、参数和接口
 * @note 1 tab == 4 spaces! This is the core header for JDY-28M driver instance management | 这是JDY-28M驱动实例管理的核心头文件
 *
 * @version V1.0 2026-3-5
 *
 *****************************************************************************/

//******************************** Includes *********************************//
#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <stdio.h>
#include <stdint.h>
typedef struct control_core_s control_core_t;

//******************************** Includes *********************************//
//******************************** Defines **********************************//

// 方向枚举：前进/后退/停止
typedef enum {
    MOVE_DIR_FORWARD  =  0,    // 前进
    MOVE_DIR_BACKWARD =  1     // 后退
} move_dir_t;

// 运动模式枚举（6种，可根据业务扩展）
typedef enum {
    MOVE_MODE_1 = 0,   
    MOVE_MODE_2 = 1,     
    MOVE_MODE_3 = 2,       
    MOVE_MODE_4 = 3,     
    MOVE_MODE_5 = 4,       
    MOVE_MODE_6 = 5,    
    MOVE_MODE_X = 6      
} move_mode_t;

typedef struct  {
    void (*set_mode)(control_core_t *cor);
}control_core_set_mode_t ;

typedef struct  {
    void (*send_pdoa)(control_core_t *cor);
} ble_send_pdoa_t;

struct control_core_s {


    uint8_t car_id;

    move_dir_t move_dir;   // 当前运动方向
    move_mode_t move_mode; // 当前运动模式


 
    control_core_set_mode_t* p_set_mode; // 设置运动模式的函数指针

    ble_send_pdoa_t* p_ble_send_pdoa; // 发送PDOA数据的函数指针
    // 其他成员变量和函数指针
};


//******************************** Defines **********************************//









#endif // BALANCE_CORE_H
