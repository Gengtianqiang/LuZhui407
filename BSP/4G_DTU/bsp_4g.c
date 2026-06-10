/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file bsp_4g.c
 *
 * @par dependencies
 * - bsp_4g.h
 *
 * @author Yharim
 *
 * @brief HAL level 4G DTU driver implementation | HAL层4G DTU驱动实现
 * @details This file implements all interfaces of 4G DTU module driver | 该文件实现4G DTU模块驱动的所有接口
 * @note 1 tab == 4 spaces!
 *
 * @version V1.0 2026-6-8
 *
 *****************************************************************************/

//******************************** Includes *********************************//
#include "bsp_4g.h"
//******************************** Includes *********************************//

/**
 * @brief Global variables and buffer definitions | 全局变量和缓冲区定义
 * @details Define DTU driver instance and receive buffer | 定义DTU驱动实例和接收缓冲区
 */
//******************************** Defines **********************************//
uint8_t dtu_rx_buffer[DTU_RX_BUFFER_SIZE] = {0}; /* receive buffer | 接收缓冲区 */

DTU_t my_4g_dtu;                                 /* DTU driver instance | DTU驱动实例 */
//******************************** Defines **********************************//

/**
 * @brief DTU send callback function | DTU发送回调函数
 * @details Send data via UART5 DMA | 通过UART5 DMA发送数据
 * @param[in] buf : Pointer to data buffer | 指向数据缓冲区的指针
 * @param[in] len : Data length to send | 要发送的数据长度
 * @return dtu_status_t : Operation status | 操作状态
 */
dtu_status_t dtu_send_callback(uint8_t* buf, uint16_t len) {
    HAL_UART_Transmit_DMA(&huart5, buf, len);
    return DTU_OK;
}

/**
 * @brief DTU ACK response function | DTU应答函数
 * @details Generate ACK response based on current state | 根据当前状态生成应答消息
 * @param[in] self : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @return dtu_status_t : Operation status (DTU_OK for success) | 操作状态（DTU_OK表示成功）
 */
char ack_buf[100] = {0};                         /* ACK response buffer | 应答消息缓冲区 */
dtu_status_t dtu_ack(DTU_t* const self) {

    switch (self->state)
    {

    case MSG_4G_BATTERY:                         /* Battery level response | 电量信息响应 */
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "Battery level is %.1fv\n",Volt);
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_PITCH_ANGLE:                     /* Pitch angle response | 俯仰角信息响应 */
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "Pitch angle is %.1f\n",myimu.euler.pitch);
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ALARM_START:                     /* Alarm start response | 报警信息响应 */
        memset(ack_buf, 0, sizeof(ack_buf));
    self->buzzer_flag += 1;
    if(self->buzzer_flag==2)self->buzzer_flag = 0;
        sprintf((char*)ack_buf, "Alarm %s\n", self->buzzer_flag ? "ON" : "OFF");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_STOP:                     /* One-key stop response | 一键停止信息响应 */
        self->stop_flag += 1;
        if(self->stop_flag==2)self->stop_flag = 0;
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key stop %s\n", self->stop_flag ? "Activated" : "Deactivated");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_START:                    /* One-key start response | 一键出发信息响应 */
				my_4g_dtu.start_flag = 1;
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key start received.\n");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_RETURN:                   /* One-key return response | 一键返回信息响应 */
        self->return_flag = 1;
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key return received.\n");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_SET_XY:                          /* Set XY response | 设置XY坐标信息响应 */
        
                // dtu_reast_titk = self->p_time->getSysTickCnt();
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "+++");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));

        break;
    default:
        break;
    }

    self->state = MSG_4G_NONE;                   /* Clear state after processing | 处理完毕后清除状态 */

    return DTU_OK;
}

/**
 * @brief OS delay function wrapper | OS延时函数封装
 * @details Provide OS-based delay implementation for DTU driver | 为DTU驱动提供基于OS的延时实现
 * @param[in] ms : Delay time in milliseconds | 延时时间（毫秒）
 */
void __my_delay_ms(uint32_t ms)
{
    osDelay(ms);

  
        
}

/**
 * @brief Get system tick count function | 获取系统tick计数函数
 * @details Provide system tick count for DTU driver timeout calculation | 为DTU驱动提供系统tick计数用于超时计算
 * @return uint32_t : System tick count | 系统tick计数值
 */
uint32_t __my_GetTick(void)
{

    return xTaskGetTickCount();
    
}




/**
 * @brief DTU frame parser function | DTU帧解析函数
 * @details Parse received frame, validate header/tail, extract command and dispatch | 解析接收到的帧，校验帧头帧尾，提取指令并分发
 * @param[in] self : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @param[in] buf  : Pointer to received frame buffer | 指向接收帧缓冲区的指针
 * @param[in] len  : Frame length | 帧长度
 * @return uint8_t : 1 if frame parsed successfully, 0 if failed | 1表示帧解析成功，0表示失败
 */
uint8_t DTU_ParseFrame(DTU_t* const self,uint8_t *buf,uint16_t len)
{
    /* 1. Validate frame header and tail | 校验帧头和帧尾 */
    if(buf[0] != FRAME_HEAD || buf[len-1] != FRAME_TAIL) {

        return 0;
    }

    /* CRC verification (reserved for future use) | CRC校验（预留） */
//     uint8_t calc_crc = 0;
//     if(buf[1]!=0x08) {
//         calc_crc = CRC8_Calc(buf, 2);
//         if(calc_crc != buf[2])

//             // return 0;
//     }else {


//         calc_crc = CRC8_Calc(buf, 6);
//         if(calc_crc != buf[6]);
//             // return 0;
//     }

    /* Extract command byte and update state | 提取指令字节并更新状态 */
    uint8_t g_dtu_cmd = buf[1];

    switch (g_dtu_cmd)
    {
    case '1':
        self->state = MSG_4G_BATTERY;
        break;
    case '2':
        self->state = MSG_4G_PITCH_ANGLE;
        break;
    case '3':
        self->state = MSG_4G_ALARM_START;
        break;
    case '4':
        self->state = MSG_4G_ONEKEY_STOP;
        break;
    case '5':
        self->state = MSG_4G_ONEKEY_START;
        break;
    case '6':
        self->state = MSG_4G_ONEKEY_RETURN;
        break;
    case '7':
        self->state = MSG_4G_SET_XY;

    default:
        break;
    }

    self->ack_fun(self);                         /* Parse success, call ACK response | 解析成功，调用应答函数 */

#ifdef DTU_DEBUG
    DTU_DEBUG_OUT("Parsed frame with command: 0x%02X, updated state to: %d\n", g_dtu_cmd, self->state);
#endif

    return 1;                                    /* Parse success | 解析成功 */
}

/**
 * @brief DTU parser function (uses ring buffer) | DTU解包函数（使用了ring buffer）
 * @details Read data from ring buffer and parse frames | 从环形缓冲区读取数据并解析帧
 * @param[in] self : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @param[in] buf  : Pointer to temporary buffer for parsing | 指向用于解析的临时缓冲区
 * @return dtu_status_t : Operation status | 操作状态
 */
uint32_t dtu_reast_titk = 0;
dtu_status_t dtu_parser(DTU_t* const self, uint8_t* buf) {

    

   if(my_4g_dtu.rx_flag==1) {
        if(dtu_get_data_from_ringbuf(&ring5_rx_DMA_buf, buf)) {
        }else {
#ifdef DTU_DEBUG
            DTU_DEBUG_OUT("No data received from ring buffer.\n");
#endif
        }
#ifdef DTU_DEBUG
            DTU_DEBUG_OUT("Data received from ring buffer: %s \n", (char*)buf);
#endif
        uint16_t ring_len = strlen((char*)buf);
        if(DTU_ParseFrame(self, buf , ring_len)) {
#ifdef DTU_DEBUG
            DTU_DEBUG_OUT("Parsed valid frame from ring buffer. State updated to: %d\n", self->state);
#endif
        }else {
#ifdef DTU_DEBUG
            DTU_DEBUG_OUT("Failed to parse frame from ring buffer.\n");
#endif
        }

        memset(buf, 0, ring_len); // Clear buffer after parsing | 解析后清空缓冲区
        dtu_reast_titk = self->p_time->getSysTickCnt();
        my_4g_dtu.rx_flag=0;
   }

   //超过五分钟无问答，直接重启
   if(self->p_time->getSysTickCnt()-dtu_reast_titk>300000){
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "+++");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        self->p_time->my_delay(500);
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "atk");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        self->p_time->my_delay(500);
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "AT+PWR\r\n");
        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        self->p_time->my_delay(500);

        self->send_fun((uint8_t*)ack_buf, strlen((char*)ack_buf));
        dtu_reast_titk = self->p_time->getSysTickCnt();
   }

}


/**
 * @brief Read data from ring buffer to a regular array | 从环形缓冲区取数据到普通数组
 * @details Use RingByteBuffer API to pop all available data into a regular array | 使用RingByteBuffer API将所有可用数据取出到普通数组
 * @param[in] ring : Pointer to ring buffer instance | 指向环形缓冲区实例的指针
 * @param[out] buf : Pointer to destination buffer | 指向目标缓冲区的指针
 * @return uint8_t : 1 if data was read successfully, 0 if empty or error | 1表示成功读取数据，0表示缓冲区为空或出错
 */
uint8_t dtu_get_data_from_ringbuf(RingByteBuffer *ring, uint8_t* buf)
{
    if (NULL == ring || NULL == buf )
    {
        return 0;
    }

    if (RingByteBuffer_isEmpty(ring))
    {
        return 0;
    }

    uint16_t avail = RingByteBuffer_size(ring);

    RingByteBuffer_popBuffer(ring, buf, avail);

    return 1;
}


/**
 * @brief CRC8 calculation function | CRC8校验计算函数
 * @details Calculate CRC8 checksum for data verification | 计算CRC8校验值用于数据验证
 * @param[in] data : Pointer to data buffer | 指向数据缓冲区的指针
 * @param[in] len  : Data length | 数据长度
 * @return uint8_t : CRC8 checksum result | CRC8校验结果
 * @note Polynomial: 0x07, Initial value: 0x00 | 多项式: 0x07, 初始值: 0x00
 */
uint8_t CRC8_Calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while(len--)
    {
        crc ^= *data++;
        for(uint8_t i=0; i<8; i++)
        {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief DTU time configuration instance | DTU时间配置实例
 * @details Bind OS delay and system tick functions to time interface | 将OS延时和系统tick函数绑定到时间接口
 */
dtu_time_t dtu_time_config = {
    .my_delay       = __my_delay_ms,             /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    .getSysTickCnt  = __my_GetTick               /* Get system tick count function | 获取系统tick计数函数 */
};

/**
 * @brief DTU driver instance initialization function | DTU驱动实例初始化函数
 * @details Dependency injection: bind interfaces and call init | 依赖注入：绑定接口并调用初始化
 * @param[in] self   : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @param[in] p_time : Pointer to time interface structure | 指向时间接口结构体的指针
 * @return dtu_status_t : Operation status (DTU_OK for success) | 操作状态（DTU_OK表示成功）
 */
dtu_status_t dtu_inst(DTU_t*          const self,
                        dtu_time_t*             p_time
)
{
    /*************1. Checking the input parameters**************/
    dtu_status_t res = DTU_OK;

    if (NULL == self  ||
        NULL == p_time)
    {

#ifdef DTU_DEBUG
        DTU_DEBUG_OUT("Error: Invalid input parameters for DTU instantiation.\n");
#endif
        return DTU_ERRORPARAMETER;
    }
    /*************1. Checking the input parameters**************/

    if (DTU_NOT_INIT != self->dtu_init_flag)
    {

#ifdef DTU_DEBUG
        DTU_DEBUG_OUT("Error: DTU already initialized.\n");
#endif
        return DTU_ERRORRESOURCE;
    }

    /*************2. Binding the interfaces**************/
    self->p_time = p_time;
    /*************2. Binding the interfaces**************/
    self->send_fun = dtu_send_callback;
    self->parser_fun = dtu_parser;
    self->ack_fun = dtu_ack;
    /*************3. Setting default values**************/

    /*************3. Setting default values**************/

    /*************4. Calling the init function**************/
     dtu_init(self);

    /*************4. Calling the init function**************/

    if (DTU_OK != res)
    {
        self->p_time = NULL;
        DTU_DEBUG_OUT("Error: DTU initialization failed.\n");
        return res;
    }

    self->send_fun("The car is ready.\n", strlen("The car is ready.\n"));
    self->dtu_init_flag = DTU_INIT;
    return res;
}

/**
 * @brief DTU driver low-level initialization function | DTU驱动底层初始化函数
 * @details Set default values for all flags and state variables | 设置所有标志位和状态变量的默认值
 * @param[in] self : Pointer to DTU driver instance | 指向DTU驱动实例的指针
 * @return dtu_status_t : Operation status (DTU_OK for success) | 操作状态（DTU_OK表示成功）
 */
dtu_status_t dtu_init(DTU_t* const self)
{
    if (NULL == self)
    {
        return DTU_ERRORPARAMETER;
    }

    self->state   = MSG_4G_NONE;
    self->rx_flag = 0;
    self->buzzer_flag = 0;
    self->stop_flag = 0;
    self->start_flag = 0;
    self->return_flag = 0;

    self->point.x = 2.0;
    self->point.y = 2.0;

    return DTU_OK;
}

