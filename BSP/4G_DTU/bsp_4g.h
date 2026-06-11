/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file bsp_4g.h
 *
 * @par dependencies
 * - stdio.h
 * - stdint.h
 * - stdbool.h
 * - stm32f4xx_hal.h
 * - Vofa.h
 * - freertos.h
 * - task.h
 * - uart_task.h
 * - util/RingByteBuffer.h
 *
 * @author Yharim
 *
 * @brief HAL level 4G DTU driver main interface | HAL层4G DTU驱动主接口
 * @details This header file encapsulates all status, parameters and interfaces of 4G DTU module driver | 该头文件封装4G DTU模块驱动的所有状态、参数和接口
 * @note 1 tab == 4 spaces! This is the core header for 4G DTU driver instance management | 这是4G DTU驱动实例管理的核心头文件
 *
 * @version V1.0 2026-6-8
 *
 *****************************************************************************/
#ifndef __BSP_4G_H
#define __BSP_4G_H

/**
 * @brief Header file includes | 头文件包含
 * @details Include necessary header files for 4G DTU driver implementation | 包含4G DTU驱动实现的必要头文件
 * @note Standard library + STM32 HAL library are mandatory | 标准库和STM32 HAL库为必选
 */
//******************************** Includes *********************************//
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Vofa.h"
#include "freertos.h"
#include "task.h"
#include "uart_task.h"
#include "util/RingByteBuffer.h"
//******************************** Includes *********************************//

/**
 * @brief Macro definitions and type definitions | 宏定义和类型定义
 * @details Define compile options, status codes, enumerations and structures for 4G DTU driver | 定义4G DTU驱动的编译选项、状态码、枚举和结构体
 * @note All custom types are prefixed with "DTU" or "MSG_4G" to avoid naming conflict | 所有自定义类型均以"DTU"或"MSG_4G"为前缀，避免命名冲突
 */
//******************************** Defines **********************************//

#define DTU_RX_BUFFER_SIZE 256                     /* DTU receive buffer size | DTU接收缓冲区大小 */
#define FRAME_HEAD       '@'                       /* Frame header byte | 帧头字节 */
#define FRAME_TAIL       'a'                       /* Frame tail byte | 帧尾字节 */

// #define DTU_DEBUG                                  /* Enable debug output for DTU driver | 启用DTU驱动的调试输出 */
#define DTU_DEBUG_OUT(X,...) Vofa_Printf(&vofa_inst_binding_uart3, X, ##__VA_ARGS__);  /* Debug output interface | 调试输出接口 */

/**
 * @brief Forward declaration of DTU driver main structure | DTU驱动主结构体前向声明
 * @details Used for callback function parameter definition | 用于回调函数参数定义
 * @note Must be declared before using as pointer type | 作为指针类型使用前必须先声明
 */
typedef struct dtu_s DTU_t;

/**
 * @brief DTU driver operation status code | DTU驱动操作状态码
 * @details Enumerate all possible return status of DTU driver APIs | 枚举DTU驱动API的所有可能返回状态
 * @note 0x01 indicates success, other values indicate error, reserved for future expansion | 0x01表示成功，其他值表示错误，预留值供未来扩展
 */
typedef enum {
    DTU_OK             = 0x01,      /* DTU Operation completed successfully | DTU操作执行成功 */
    DTU_ERROR          = 0x02,      /* DTU Run-time error without case matched | DTU运行时错误（未匹配场景） */
    DTU_ERRORTIMEOUT   = 0x03,      /* DTU Operation failed with timeout | DTU操作超时失败 */
    DTU_ERRORRESOURCE  = 0x04,      /* DTU Resource not available | DTU资源不可用 */
    DTU_ERRORPARAMETER = 0x05,      /* DTU Parameter error | DTU参数错误 */
    DTU_ERRORNOMEMORY  = 0x06,      /* DTU Out of memory | DTU内存不足 */
    DTU_ERRORISR       = 0x07,      /* DTU Not allowed in ISR context | DTU不允许在中断服务程序(ISR)中执行 */
    DTU_RESERVED       = 0x08,      /* DTU Reserved for future expansion | DTU预留值，供未来扩展 */
} dtu_status_t;

/**
 * @brief 4G DTU message type enumeration | 4G DTU消息类型枚举
 * @details Define all supported message types of 4G DTU module | 定义4G DTU模块支持的所有消息类型
 * @note Used for command dispatch and state management | 用于指令分发和状态管理
 */
typedef enum {
    MSG_4G_BATTERY       = 0x01,     /* Battery level query | 电量查询 */
    MSG_4G_PITCH_ANGLE   = 0x02,     /* Pitch angle query | 俯仰角查询 */
    MSG_4G_ALARM_START   = 0x03,     /* One-key alarm start | 一键报警 */
    MSG_4G_ONEKEY_STOP   = 0x04,     /* One-key stop | 一键停止 */
    MSG_4G_ONEKEY_START  = 0x05,     /* One-key start | 一键出发 */
    MSG_4G_ONEKEY_RETURN = 0x06,     /* One-key return | 一键返回 */
    MSG_4G_SET_XY        = 0x07,     /* Set target XY coordinates | 设置XY坐标目标点 */
    MSG_4G_NONE          = 0x08,     /* No message pending | 无消息需要处理 */
} msg_4g_type_t;

/**
 * @brief DTU time interface structure | DTU时间接口结构体
 * @details Encapsulate time-related interfaces for 4G DTU driver | 封装4G DTU驱动的时间相关接口
 * @note Provides delay and system tick functions | 提供延时和系统tick计数功能
 */
typedef struct {
    void (*my_delay)(uint32_t   ms);          /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    uint32_t (*getSysTickCnt)(void);          /* Get system tick count function | 获取系统tick计数函数 */
} dtu_time_t;

/**
 * @brief DTU driver initialization status | DTU驱动初始化状态
 * @details Enumerate initialization status of 4G DTU driver | 枚举4G DTU驱动的初始化状态
 * @note 0 indicates uninitialized, 1 indicates initialized | 0表示未初始化，1表示已初始化
 */
typedef enum {
    DTU_NOT_INIT   = 0,                       /* DTU driver not initialized | DTU驱动未初始化 */
    DTU_INIT       = 1                        /* DTU driver initialized successfully | DTU驱动初始化成功 */
} dtu_init_t;

/**
 * @brief 2D coordinate point structure | 二维坐标点结构体
 * @details Encapsulate X/Y coordinate for target positioning | 封装目标定位的X/Y坐标
 * @note Both coordinates are float type for precision | 两个坐标均为float类型以保证精度
 */
typedef struct {
    float x;                                  /* X coordinate | X坐标 */
    float y;                                  /* Y coordinate | Y坐标 */
} point_t;

/**
 * @brief DTU driver main structure | DTU驱动主结构体
 * @details Encapsulates all status, parameters and interfaces of 4G DTU driver instance | 封装4G DTU驱动实例的所有状态、参数和接口
 * @note This is the core structure for DTU driver instance management | 这是DTU驱动实例管理的核心结构体
 */
struct dtu_s {
    dtu_init_t                  dtu_init_flag;    /* DTU initialization flag | DTU是否初始化标志 */
    msg_4g_type_t                       state;            /* Current working state | 当前工作状态 */
    uint8_t                           rx_flag;          /* Received message flag | 接收到消息标志 */

    uint8_t                       buzzer_flag;      /* Buzzer toggle flag | 蜂鸣器切换标志 */

    point_t                             point;            /* Target coordinate point | 目标坐标点 */

    uint8_t                         stop_flag;        /* One-key stop flag | 一键停止标志 */

    uint8_t                        start_flag;       /* One-key start flag | 一键出发标志 */

    uint8_t                       return_flag;      /* One-key return flag | 一键返回标志 */

    uint8_t                    imu_error_flag;       /* One-key start flag | 一键出发标志 */

    uint8_t                    jdy_error_flag;      /* One-key return flag | 一键返回标志 */

    dtu_status_t (*send_fun)(uint8_t* buf, uint16_t len);       /* Send function | 发送函数 */

    dtu_time_t*                 p_time;           /* Time interface | 时间接口 */

    dtu_status_t (*parser_fun)(DTU_t* const self, uint8_t* buf); /* Parser function (uses ring buffer) | 解包函数（使用了ring buffer） */

    dtu_status_t (*ack_fun)(DTU_t* const);        /* ACK response function | 应答函数 */
};

/**
 * @brief DTU driver instance initialization function | DTU驱动实例初始化函数
 * @details Dependency injection interface for DTU driver instance | DTU驱动实例的依赖注入接口
 * @param[in] self   : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @param[in] p_time : Pointer to time interface structure | 指向时间接口结构体的指针
 * @return dtu_status_t : Operation status (DTU_OK for success) | 操作状态（DTU_OK表示成功）
 */
dtu_status_t dtu_inst(DTU_t*          const self,
                      dtu_time_t*         p_time
);

/**
 * @brief DTU driver low-level initialization function | DTU驱动底层初始化函数
 * @details Initialize hardware and set default values for DTU driver instance | 初始化硬件并设置DTU驱动实例的默认值
 * @param[in] self : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @return dtu_status_t : Operation status (DTU_OK for success) | 操作状态（DTU_OK表示成功）
 */
dtu_status_t dtu_init(DTU_t*          const self
);

/**
 * @brief Read data from ring buffer to a regular array | 从环形缓冲区取数据到普通数组
 * @details Use RingByteBuffer API to pop all available data | 使用RingByteBuffer API取出所有可用数据
 * @param[in] ring : Pointer to ring buffer instance | 指向环形缓冲区实例的指针
 * @param[out] buf : Pointer to destination buffer | 指向目标缓冲区的指针
 * @return uint8_t : 1 if data was read successfully, 0 if empty or error | 1表示成功读取数据，0表示缓冲区为空或出错
 */
uint8_t dtu_get_data_from_ringbuf(RingByteBuffer *ring, uint8_t* buf);

//******************************** Defines **********************************//
extern DTU_t                           my_4g_dtu;              /* DTU driver instance | DTU驱动实例 */
extern dtu_time_t                dtu_time_config;             /* Time interface configuration | 时间接口配置结构体 */
extern dtu_status_t  dtu_init(DTU_t* const self); /* DTU initialization function | DTU初始化函数 */
extern uint8_t dtu_rx_buffer[DTU_RX_BUFFER_SIZE];/* DTU receive buffer | DTU接收缓冲区 */

#endif /* __BSP_4G_H */