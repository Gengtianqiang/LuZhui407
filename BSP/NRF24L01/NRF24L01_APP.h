/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file NRF24L01_APP.h
 *
 * @par dependencies
 * - stdint.h
 * - NRF24L01_Define.h
 * - NRF24L01.h
 *
 * @author Wooncake
 *
 * @brief NRF24L01 wireless module application layer interface | NRF24L01无线模块应用层接口
 * @details This header file declares the NRF24L01 application task and vehicle-to-vehicle communication interfaces | 此头文件声明NRF24L01应用任务及车车通信接口
 * @note 1 tab == 4 spaces! Used for inter-vehicle wireless command relay | 用于车辆间无线指令中继
 *
 * @version V1.0 2026-6-14
 *
 *****************************************************************************/
#ifndef __NRF24L01_APP_H
#define __NRF24L01_APP_H

/**
 * @brief Header file includes | 头文件包含
 * @details Include necessary header files for NRF24L01 application layer | 包含NRF24L01应用层所需的必要头文件
 * @note Standard library + NRF24L01 driver are mandatory | 标准库与NRF24L01驱动为必选依赖
 */
//******************************** Includes *********************************//
#include <stdint.h>
#include "NRF24L01_Define.h"
#include "NRF24L01.h"
//******************************** Includes *********************************//


/**
 * @brief NRF24L01 application task function | NRF24L01应用任务函数
 * @details Main task for inter-vehicle wireless communication: receives/sends forward and return commands between cars | 车辆间无线通信主任务：收发车辆间前进与返回指令
 * @param None
 * @return uint8_t : Task execution status (0 = success) | 任务执行状态（0表示成功）
 * @note Called periodically from the balance task loop | 由平衡任务循环周期性调用
 */
uint8_t NRF24L01_TASK(void);
void NRF24L01_Screen_Task(void);
void NRF24L01_SetAddress(uint8_t *Address, uint8_t Address4);
//******************************** Defines **********************************//

#endif /* __NRF24L01_APP_H */
