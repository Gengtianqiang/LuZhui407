/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file mesh_mode.h
 * 
 * @par dependencies 
 * - string.h
 * - freertos.h
 * - queue.h
 * - RingByteBuffer.h
 * - jdy_driver.h
 * - forward.h
 * 
 * @author Yharim
 * 
 * @brief HAL level JDY-28M MESH mode configuration header | HAL层JDY-28M MESH模式配置头
 * @details This header file encapsulates all MESH mode macros and external declarations of JDY-28M BLE/MESH module driver | 此头文件封装JDY-28M蓝牙MESH模块驱动的所有MESH模式宏定义和外部声明
 * @note 1 tab == 4 spaces! This is the auxiliary header for JDY-28M MESH mode management | 这是JDY-28M MESH模式管理的辅助头文件
 * 
 * @version V1.0 2026-3-5
 *
 *****************************************************************************/
#ifndef __MESH_MODE_H
#define __MESH_MODE_H

//******************************** Includes *********************************//
#include <string.h>
#include "freertos.h"
#include "queue.h"
#include "RingByteBuffer.h"
#include "jdy_driver.h"
#include "forward.h"
#include "System_Init.h"
#include "balance.h"

//******************************** Includes *********************************//

//******************************** Defines **********************************//
/**
 * @brief JDY UART buffer size | JDY串口缓冲区大小
 * @details Minimum size is 25, otherwise error occurs | 最少为25，不然会出错
 */
#define JDY_UART_MAX_SIZE 25

/**
 * @brief JDY UART timeout | JDY串口超时时间
 * @details Timeout for JDY AT command query (40ms) | JDY AT指令查询超时时间（40毫秒）
 */
#define JDY_UART_TIMEOUT 40

/**
 * @brief MESH data send array size | MESH数据发送数组大小
 * @details Size of global array for JDY-28M MESH packet transmission | 用于JDY-28M MESH数据包发送的全局数组大小
 */
#define MESH_TX_ARRAY_SIZE 18

/**
 * @brief MESH data receive array size | MESH数据接收数组大小
 * @details Size of global array for JDY-28M MESH packet reception | 用于JDY-28M MESH数据包接收的全局数组大小
 */
#define MESH_RX_ARRAY_SIZE 13

/**
 * @brief MESH network ID AT command | MESH组网ID AT指令
 * @details Pre-configured AT command for setting MESH network ID | 预配置的MESH组网ID设置AT指令
 */
#define MESH_NETID "AT+NETID0A19132E,6\r\n"
//******************************** Defines **********************************//

//******************************** External Declarations ********************//

extern RingByteBuffer                 ringBuffer_Parser;
extern uint8_t uart4_dma_rx_buffer[UART_RX_BUFFER_SIZE];
extern QueueHandle_t                        xUART4Queue;
uint8_t int_to_uint8                    (int input_int);
uint8_t float_to_uint8              (float input_float);
float uint8_to_float  (uint8_t input_u8, int use_round);
int uint8_to_int                     (uint8_t input_u8);

//******************************** External Declarations ********************//

#endif /* __MESH_MODE_H */
