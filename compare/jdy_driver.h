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
#ifndef __JDY_DRIVER_H
#define __JDY_DRIVER_H

/**
 * @brief Header file includes | 头文件包含
 * @details Include necessary header files for JDY-28M driver implementation | 包含JDY-28M驱动实现所需的必要头文件
 * @note Standard library + STM32 HAL library are mandatory | 标准库和STM32 HAL库为必选依赖
 */
//******************************** Includes *********************************//
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "mesh_mode.h"
#include "Vofa.h"
#include "freertos.h"
#include "task.h"
#include "semphr.h"
//******************************** Includes *********************************//

/**
 * @brief Macro definitions and type definitions | 宏定义和类型定义
 * @details Define compile options, status codes, enumerations and structures for JDY-28M driver | 定义JDY-28M驱动的编译选项、状态码、枚举和结构体
 * @note All custom types are prefixed with "JDY" to avoid naming conflict | 所有自定义类型均以"JDY"为前缀，避免命名冲突
 */
//******************************** Defines **********************************//

#define JDY_USE_OS                              /* Enable OS support for JDY driver | 启用JDY驱动的OS支持 */
//#define JDY_DEBUG                               /* Enable debug output for JDY driver | 启用JDY驱动的调试输出 */
#define JDY_DEBUG_OUT(X,...) Vofa_Printf(&vofa_inst_binding_uart3, X, ##__VA_ARGS__);  /* Debug output interface | 调试输出接口 */

/**
 * @brief Forward declaration of JDY driver main structure | JDY驱动主结构体前向声明
 * @details Used for callback function parameter definition | 用于回调函数参数定义
 * @note Must be declared before using as pointer type | 作为指针类型使用前必须声明
 */
typedef struct JDY_s Jdy_t;
typedef struct mesh_datarecv_pkt_s mesh_datarecv_pkt_t;
typedef struct mesh_datasend_pkt_s mesh_datasend_pkt_t;

/**
 * @brief JDY driver operation status code | JDY驱动操作状态码
 * @details Enumerate all possible return status of JDY driver APIs | 枚举JDY驱动所有API的可能返回状态
 * @note 0 indicates success, non-zero indicates error, reserved for future expansion | 0表示成功，非0表示错误，预留值用于未来扩展
 */
typedef enum {
    JDY_OK             = 0,      /* JDY Operation completed successfully | JDY操作执行成功 */
    JDY_ERROR          = 1,      /* JDY Run-time error without case matched | JDY运行时错误，无匹配场景 */
    JDY_ERRORTIMEOUT   = 2,      /* JDY Operation failed with timeout | JDY操作超时失败 */
    JDY_ERRORRESOURCE  = 3,      /* JDY Resource not available | JDY资源不可用 */
    JDY_ERRORPARAMETER = 4,      /* JDY Parameter error | JDY参数错误 */
    JDY_ERRORNOMEMORY  = 5,      /* JDY Out of memory | JDY内存不足 */
    JDY_ERRORISR       = 6,      /* JDY Not allowed in ISR context | JDY操作不允许在中断服务程序（ISR）中执行 */
    JDY_RESERVED       = 7,      /* JDY Reserved for future expansion | JDY预留值用于未来扩展 */
}jdy_status_t;

/**
 * @brief JDY IO type enumeration | JDY IO类型枚举
 * @details Define all supported IO types of JDY-28M module | 定义JDY-28M模块支持的所有IO类型
 * @note Only output and LED IO are supported currently | 当前仅支持输出IO和LED引脚
 */
typedef enum {
    JDY_IO_TYPE_OUTPUT = 0,      /* Output IO | 输出IO */
    JDY_IO_TYPE_LED    = 1,      /* LED control IO | LED控制引脚 */
    JDY_IO_CLOSE       = 2       /* IO close (high level) | IO关闭（高电平） */
} jdy_io_type_t;

/**
 * @brief JDY IO control structure | JDY IO控制结构体
 * @details Encapsulate IO type and set interface for JDY-28M module | 封装JDY-28M模块的IO类型和设置接口
 * @note IO type is passed by pointer for flexible configuration | IO类型通过指针传递，支持灵活配置
 */
typedef struct {
    jdy_status_t (*pf_set_io_handler)(Jdy_t* const, jdy_io_type_t); /* IO state set callback function | IO状态设置回调函数 */
} jdy_io_t;

/**
 * @brief JDY working mode enumeration | JDY工作模式枚举
 * @details Define all supported working modes of JDY-28M module | 定义JDY-28M模块支持的所有工作模式
 * @note BLE master/slave, MESH with frame, MESH without frame are supported | 支持BLE主从、MESH有帧、MESH无帧三种模式
 */
typedef enum {
    JDY_MODE_BLE                = 0,      /* BLE master/slave transparent mode | BLE主从透传模式 */
    JDY_MODE_MESH_WITH_FRAME    = 1,      /* MESH command mode with frame format | MESH有帧格式（指令通信模式） */
    JDY_MODE_CAR                = 2       /* MESH transparent mode without frame format | MESH无帧格式（透传模式） */
} jdy_mode_type_t;

/**
 * @brief JDY data transmit structure | JDY数据发送结构体
 * @details Encapsulate transmit buffer, length, status and callback for JDY-28M module | 封装JDY-28M模块的发送缓冲区、长度、状态和回调
 * @note is_busy flag is used to avoid concurrent transmit | is_busy标志用于避免并发发送
 */
typedef struct {
    bool is_busy;                          /* Transmit status (false=idle, true=busy) | 发送状态（false=空闲，true=发送中） */
    jdy_status_t (*done_cb)(Jdy_t* const, uint8_t* buf, uint16_t len); /* Transmit completion callback function | 发送完成回调函数 */
} jdy_tx_t;

/**
 * @brief JDY data receive structure | JDY数据接收结构体
 * @details Encapsulate receive buffer, length, status and callback for JDY-28M module | 封装JDY-28M模块的接收缓冲区、长度、状态和回调
 * @note is_busy flag is used to avoid concurrent receive | is_busy标志用于避免并发接收
 */
typedef struct {
    bool is_busy;                          /* Receive status (false=idle, true=busy) | 接收状态（false=空闲，true=接收中） */
    jdy_status_t (*done_cb)(Jdy_t* const, uint8_t* buf, uint16_t len); /* Receive completion callback function | 接收完成回调函数 */
} jdy_rx_t;

/**
 * @brief JDY driver initialization status | JDY驱动初始化状态
 * @details Enumerate initialization status of JDY-28M driver | 枚举JDY-28M驱动的初始化状态
 * @note 0 indicates uninitialized, 1 indicates initialized | 0表示未初始化，1表示已初始化
 */
typedef enum {
    JDY_NOT_INIT   = 0,      /* JDY driver not initialized | JDY驱动未初始化 */
    JDY_INIT       = 1       /* JDY driver initialized successfully | JDY驱动初始化成功 */
}jdy_init_t;

/**
 * @brief JDY OS support structure | JDY OS支持结构体
 * @details Encapsulate OS related interfaces for JDY-28M driver (only for USE_OS) | 封装JDY-28M驱动的OS相关接口（仅USE_OS生效）
 * @note Empty structure for future expansion | 空结构体预留，用于未来扩展
 */
#ifdef JDY_USE_OS
typedef struct {
    void (*pf_os_delay_ms)(uint32_t ms);    /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
}jdy_os_support_t;
#endif // JDY_USE_OS

/**
 * @brief JDY extended function structure | JDY拓展功能结构体
 * @details Encapsulate custom function interface for JDY-28M driver | 封装JDY-28M驱动的自定义功能接口
 * @note User can add custom functions through this structure | 用户可通过此结构体添加自定义功能
 */
typedef struct {
    void (*pf_custom_function)(Jdy_t* const); /* User custom function pointer | 用户自定义功能函数指针 */
}jdy_function_t;

/**
 * @brief JDY MESH connection status enumeration | JDY MESH连接状态枚举
 * @details Define MESH network connection status of JDY-28M module | 定义JDY-28M模块的MESH组网连接状态
 * @note 0=disconnected,1=connected,2=meshed,3=unmeshed | 0=未连接，1=已连接，2=已组网，3=未组网
 */
typedef enum {
    JDY_STAT_DISCONNECTED = 0,   /* Disconnected | 未连接 */
    JDY_STAT_CONNECTED    = 1,   /* Connected    | 已连接 */
    JDY_STAT_MESHED       = 2,   /* Meshed       | 已组网 */
    JDY_STAT_UNMESHED     = 3    /* Unmeshed     | 未组网 */
}mesh_STAT_t;

/**
 * @brief JDY flag bit structure | JDY标志位结构体
 * @details Encapsulate status flag bits of JDY-28M module | 封装JDY-28M模块的状态标志位
 * @note All flags are 1-bit wide to save memory | 所有标志位均为1位宽度，节省内存
 */
typedef struct {
    uint8_t        bleUART_tx_flag:1;        /* BLE UART transmit flag | BLE串口发送标志 */
    uint8_t        bleUART_rx_flag:1;        /* BLE UART receive flag  | BLE串口接收标志 */
    uint8_t ble_let_MCU_sleep_flag:1; /* BLE allow MCU sleep flag | BLE允许MCU睡眠标志 */
}jdy_flage_t;

/**
 * @brief JDY working state enumeration | JDY工作状态枚举
 * @details Define runtime working state of JDY-28M driver | 定义JDY-28M驱动的运行时工作状态
 * @note Used for state machine management | 用于状态机管理
 */
typedef enum {
    idle,          /* Idle state | 空闲状态 */
    waiting,       /* Waiting state | 等待状态 */
    handling,      /* Handling state | 处理中状态 */
    mesh_recving   /* MESH receiving state | MESH数据接收中状态 */
}jdy_state_t;

/**
 * @brief MESH data receive packet structure | MESH数据接收数据包结构体
 * @details Encapsulate MESH receive packet format of JDY-28M module | 封装JDY-28M模块的MESH接收数据包格式
 * @note Packed and 1-byte aligned to match hardware protocol | 紧凑排列且1字节对齐，匹配硬件协议格式
 */
struct mesh_datarecv_pkt_s{
    uint16_t     header;                 /* Packet header | 数据包头部 */
    uint8_t       netID;                  /* Network ID | 网络ID */
    uint16_t from_maddr;             /* Source MESH address | 源MESH地址 */
    uint16_t   to_maddr;               /* Target MESH address | 目标MESH地址 */
    uint8_t         key;                    /* User-defined key | 用户自定义密钥 */
    uint8_t           L;                      /* User-defined parameter L | 用户自定义参数L */
    uint8_t           R;                      /* User-defined parameter R | 用户自定义参数R */
    uint16_t        end;                    /* Packet end flag | 数据包结束标志 */
}__attribute__((packed, aligned(1)));

/**
 * @brief MESH data send packet structure | MESH数据发送数据包结构体
 * @details Encapsulate MESH send packet format of JDY-28M module | 封装JDY-28M模块的MESH发送数据包格式
 * @note Packed to match hardware protocol | 紧凑排列，匹配硬件协议格式
 */
struct mesh_datasend_pkt_s{
    uint8_t       valid;                  /* Packet valid flag | 数据包有效标志 */
    uint8_t   header[5];              /* Packet header | 数据包头部 */
    uint16_t   to_maddr;               /* Target MESH address | 目标MESH地址 */
    uint8_t    header_2;               /* Secondary header | 二级头部 */
    uint8_t         key;                    /* User-defined key | 用户自定义密钥 */
    uint8_t           L;                      /* User-defined parameter L | 用户自定义参数L */
    uint8_t           R;                      /* User-defined parameter R | 用户自定义参数R */
    uint16_t        end;                    /* Packet end flag | 数据包结束标志 */
}__attribute__((packed));

/**
 * @brief MESH data parser structure | MESH数据解析结构体
 * @details Encapsulate MESH data receive/send handler for JDY-28M module | 封装JDY-28M模块的MESH数据接收/发送处理函数
 * @note Connects MESH packet structure with processing logic | 关联MESH数据包结构体与处理逻辑
 */
typedef struct {
    mesh_datarecv_pkt_t  recv_pkt;                     /* MESH receive packet | MESH接收数据包 */
    mesh_datasend_pkt_t  send_pkt;                     /* MESH send packet | MESH发送数据包 */
    SemaphoreHandle_t uart_tx_sem;                     /* UART transmit semaphore | UART发送信号量 */
    jdy_status_t (*pf_mesh_datarecv_handler)(Jdy_t* const,      RingByteBuffer*); /* Mesh data receive handler | MESH数据接收处理函数 */
    jdy_status_t (*pf_mesh_datasend_handler)(Jdy_t* const, mesh_datasend_pkt_t*); /* Mesh data send handler | MESH数据发送处理函数 */
}mash_parser_t;

/**
 * @brief JDY MESH submode structure | JDY MESH子模式结构体
 * @details Encapsulate all parameters and interfaces of JDY-28M MESH mode | 封装JDY-28M MESH模式的所有参数和接口
 * @note Core structure for MESH network management | MESH组网管理的核心结构体
 */
typedef struct {
    jdy_init_t         mash_init_flag;    /* MESH initialization flag | MESH是否初始化完成 */
    jdy_state_t                 state;             /* Current working state | 当前工作状态 */
    jdy_flage_t                 flage;             /* Status flag bits | 状态标志位 */
    char                      MAC[13];           /* 12-digit MAC string (0~F) | 12位MAC字符串（字符范围0~F） */
    mesh_STAT_t                  STAT;              /* MESH connection status | MESH连接状态（0未连接；1已连接；2已组网；3未组网） */
    char                     MADDR[5];          /* MESH short address (0000-FFFF) | MESH组网短地址（0000-FFFF，默认MAC后2字节） */
    char                    NETID[11];         /* MESH network ID | MESH组网ID（手柄MAC后4字节+','+A） */
    mash_parser_t*           p_parser;         /* Pointer to MESH parser structure | 指向MESH解析结构体的指针 */
    jdy_status_t (*mesh_init)(Jdy_t *const); /* MESH initialization function | MESH初始化函数 */
}jdy_mesh_t;

/**
 * @brief JDY time interface structure | JDY时间接口结构体
 * @details Encapsulate time-related interfaces for JDY-28M driver | 封装JDY-28M驱动的时间相关接口
 * @note Provides delay and system tick functions | 提供延时和系统滴答计数功能
 */
typedef struct {
    void (*my_delay)(uint32_t   ms);   /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    uint32_t (*getSysTickCnt)(void); /* Get system tick count function | 获取系统滴答计数函数 */
}jdy_time_t;

/**
 * @brief JDY driver main structure | JDY驱动主结构体
 * @details Encapsulates all status, parameters and interfaces of JDY-28M driver instance | 封装JDY-28M驱动实例的所有状态、参数和接口
 * @note This is the core structure for JDY driver instance management | 这是JDY驱动实例管理的核心结构体
 */
struct JDY_s {
    jdy_init_t       init_status;    /* JDY driver initialization status | JDY句柄初始化状态 */
    jdy_mode_type_t         mode;           /* Current working mode | 当前工作模式 */
    jdy_mesh_t*   p_mesh_submode; /* Current MESH submode | MESH模式下的所有操作 */
    jdy_time_t*           p_time;         /* Time structure | 时间相关操作的结构体（时基，延时函数） */
    jdy_tx_t*               p_tx;           /* Pointer to data transmit structure | 指向数据发送结构体的指针 */
    jdy_rx_t*               p_rx;           /* Pointer to data receive structure | 指向数据接收结构体的指针 */
    jdy_function_t*   p_function;     /* Pointer to extended function structure | 指向功能拓展相关结构体的指针 */
}; 

/**
 * @brief JDY driver instance initialization function | JDY驱动实例初始化函数
 * @details Dependency injection interface for JDY driver instance | JDY驱动实例的依赖注入接口
 * @param[in] self           : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] p_mesh_submode : Pointer to MESH submode structure | 指向MESH子模式结构体的指针
 * @param[in] p_tx           : Pointer to transmit structure | 指向发送结构体的指针
 * @param[in] p_rx           : Pointer to receive structure | 指向接收结构体的指针
 * @param[in] p_time         : Pointer to time interface structure | 指向时间接口结构体的指针
 * @param[in] p_function     : Pointer to extended function structure | 指向拓展功能结构体的指针
 * @return jdy_status_t      : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 */
jdy_status_t jdy_inst(Jdy_t*          const self,
                      jdy_mesh_t* p_mesh_submode,
                      jdy_tx_t*             p_tx,
                      jdy_rx_t*             p_rx, 
                      jdy_time_t*         p_time,
                      jdy_function_t* p_function
);

//******************************** Defines **********************************//
extern Jdy_t                   jdy_handle;       /* JDY driver instance | JDY驱动实例 */
extern jdy_mesh_t             mesh_config;      /* MESH configuration structure | MESH配置结构体 */
extern jdy_tx_t                 tx_config;       /* Transmit configuration structure | 发送配置结构体 */
extern jdy_rx_t                 rx_config;       /* Receive configuration structure | 接收配置结构体 */
extern jdy_time_t             time_config;      /* Time interface configuration | 时间接口配置结构体 */
extern jdy_function_t     function_config;      /* Extended function configuration | 拓展功能配置结构体 */
extern UART_HandleTypeDef          huart4;       /* UART4 handle for JDY communication | JDY通信使用的UART4句柄 */
extern UART_HandleTypeDef          huart1;       /* UART1 handle for JDY communication | JDY通信使用的UART1句柄 */

#endif /* __JDY_DRIVER_H */

