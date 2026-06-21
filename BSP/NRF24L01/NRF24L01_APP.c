/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file NRF24L01_APP.c
 *
 * @par dependencies
 * - NRF24L01_APP.h
 * - twr_control.h
 * - jdy_driver.h
 *
 * @author Wooncake
 *
 * @brief NRF24L01 wireless module application layer implementation | NRF24L01无线模块应用层实现
 * @details This source file implements inter-vehicle wireless communication logic for forward/return command relay | 此源文件实现车辆间无线通信的前进/返回指令中继逻辑
 * @note 1 tab == 4 spaces! Different car roles (Ahand/Middle/Behind) use different send/receive logic | 不同车辆角色（头车/中间车/后车）使用不同的收发逻辑
 *
 * @version V1.0 2026-6-14
 *
 *****************************************************************************/

/**
 * @brief Header file includes | 头文件包含
 * @details Include necessary header files for NRF24L01 application implementation | 包含NRF24L01应用实现所需的必要头文件
 * @note BSP and APP layer headers are mandatory | BSP层与APP层头文件均为必选依赖
 */
//******************************** Includes *********************************//
#include "NRF24L01_APP.h"
#include "twr_control.h"
#include "jdy_driver.h"
//******************************** Includes *********************************//

/**
 * @brief Global variables for NRF24L01 communication status | NRF24L01通信状态全局变量
 * @details Track send/receive success/failure counts and flags for debugging | 记录发送/接收的成功/失败计数与标志位，用于调试
 */
//******************************** Defines **********************************//

/**
 * @brief Send status variables | 发送状态变量
 */
uint8_t         SendFlag;                               /* Send result flag | 发送结果标志位 */
uint8_t SendSuccessCount;                       /* Send success counter | 发送成功计次 */
uint8_t  SendFailedCount;                        /* Send failure counter | 发送失败计次 */

/**
 * @brief Receive status variables | 接收状态变量
 */
uint8_t         ReceiveFlag;                            /* Receive result flag | 接收结果标志位 */
uint8_t ReceiveSuccessCount;                    /* Receive success counter | 接收成功计次 */
uint8_t  ReceiveFailedCount;                     /* Receive failure counter | 接收失败计次 */

//******************************** Defines **********************************//

//******************************** Function Implementations ******************//

/**
 * @brief Set NRF24L01 communication address | 设置NRF24L01通信地址
 * @details Configure the 5-byte address with fixed prefix (0x00,0x00,0x00,0xAA) and variable last byte | 配置5字节地址，固定前缀（0x00,0x00,0x00,0xAA）+ 可变末字节
 * @param[in] Address   : Pointer to address buffer (5 bytes) | 指向地址缓冲区（5字节）的指针
 * @param[in] Address4  : Variable 4th byte for node selection | 可变的第4字节，用于节点选择
 * @return None
 * @note Different nodes are identified by the last address byte | 不同节点通过末字节地址区分
 */
static void NRF24L01_SetAddress(uint8_t *Address, uint8_t Address4)
{
    Address[0] =     0x00;
    Address[1] =     0x00;
    Address[2] =     0x00;
    Address[3] =     0xAA;
    Address[4] = Address4;
}

/**
 * @brief NRF24L01 application main task function | NRF24L01应用主任务函数
 * @details Execute role-specific wireless communication logic based on compile-time car role selection | 根据编译时车辆角色选择，执行对应的无线通信逻辑
 * @param None
 * @return uint8_t : Task execution status (0 = success) | 任务执行状态（0表示成功）
 * @note Car role is determined by AHAND_CAR / MIDDLE_CAR / MIDDLE_CAR_FIRST / BEHIND_CAR macro | 车辆角色由AHAND_CAR / MIDDLE_CAR / MIDDLE_CAR_FIRST / BEHIND_CAR宏决定
 */
uint8_t NRF24L01_TASK(void)
{
#ifdef AHAND_CAR
    /************* 头车：前进完成后向二号车发送前进指令 **************/
    if (1 == g_state_machine.ahand_flag)
    {
        /*************1. Set target address to car 2 | 设置目标地址为二号车**************/
        NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA2);
        NRF24L01_TxPacket[0] = 0x00;
        NRF24L01_TxPacket[1] = 0x00;
        NRF24L01_TxPacket[2] = 0x00;
        NRF24L01_TxPacket[3] = 0x01;

        /*************2. Send forward command | 发送前进指令**************/
        SendFlag = NRF24L01_Send();
        if (SendFlag == 1)
        {
            JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n",
                NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
        else
        {
            JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n",
                NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
    }
    /************* 头车：收到返回指令后向所有中间车发送返回指令 **************/
    else if (3 == g_state_machine.ahand_flag)
    {
        /*************1. Set target address to car 2 | 设置目标地址为二号车**************/
        NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA2);
        NRF24L01_TxPacket[0] = 0x00;
        NRF24L01_TxPacket[1] = 0x00;
        NRF24L01_TxPacket[2] = 0x00;
        NRF24L01_TxPacket[3] = 0x02;

        /*************2. Send return command | 发送返回指令**************/
        SendFlag = NRF24L01_Send();
        if (SendFlag == 1)
        {
            JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n",
                NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
        else
        {
            JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n",
                NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
    }
#endif /* AHAND_CAR */

#ifdef BEHIND_CAR
    /************* 后车：接收到所有中间车返回信号后返回 **************/
    /*************1. Set receive address and listen | 设置接收地址并监听**************/
    NRF24L01_SetAddress(NRF24L01_RxAddress, 0xA5);
    NRF24L01_UpdateRxAddress();
    ReceiveFlag = NRF24L01_Receive();

    /*************2. Check for return signal (packet[3]==2) | 检查返回信号（packet[3]==2）**************/
    if (1 == ReceiveFlag)
    {
        if (2 == NRF24L01_RxPacket[3])
        {
            g_state_machine.behind_flag = 1;
            JDY_DEBUG_OUT("Behind Car received return signal.\n");
        }
    }
#endif /* BEHIND_CAR */

#ifdef MIDDLE_CAR
    /************* 中间车：预留接收处理逻辑 **************/
#endif /* MIDDLE_CAR */

#ifdef MIDDLE_CAR_FIRST
    /************* 一号中间车：接收到头车信号后执行前进/转发返回指令 **************/
    /*************1. Set receive address (from car 1) and listen | 设置接收地址（来自头车）并监听**************/
    NRF24L01_SetAddress(NRF24L01_RxAddress, 0xA2);
    NRF24L01_UpdateRxAddress();
    ReceiveFlag = NRF24L01_Receive();

    if (1 == ReceiveFlag)
    {
        /*************2. Forward signal (packet[3]==1): start moving | 前进信号（packet[3]==1）：开始前进**************/
        if (1 == NRF24L01_RxPacket[3])
        {
            g_state_machine.middle_flag = 1;
            JDY_DEBUG_OUT("Middle Car 1 received forward signal.\n");
        }
        /*************3. Return signal (packet[3]==2): forward to behind car | 返回信号（packet[3]==2）：转发给后车**************/
        else if (2 == NRF24L01_RxPacket[3])
        {
            g_state_machine.middle_flag = 2;
            JDY_DEBUG_OUT("Middle Car 1 received return signal.\n");

            /*************4. Forward return command to car 5 (behind car) | 转发返回指令给五号车（后车）**************/
            NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA5);
            NRF24L01_TxPacket[0] = 0x00;
            NRF24L01_TxPacket[1] = 0x00;
            NRF24L01_TxPacket[2] = 0x00;
            NRF24L01_TxPacket[3] = 0x02;

            SendFlag = NRF24L01_Send();
            if (SendFlag == 1)
            {
                JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n",
                    NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                    NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
            }
            else
            {
                JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n",
                    NRF24L01_TxPacket[0], NRF24L01_TxPacket[1],
                    NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
            }
        }
    }
#endif /* MIDDLE_CAR_FIRST */

    return 0;
}

//******************************** Function Implementations ******************//