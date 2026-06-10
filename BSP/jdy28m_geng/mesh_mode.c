/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file mesh_mode.c
 *
 * @par dependencies
 * - mesh_mode.h
 * - freertos.h
 * - queue.h
 * - System_Init.h
 *
 * @author Yharim
 *
 * @brief HAL level JDY-28M MESH mode implementation | HAL层JDY-28M MESH模式实现
 * @details This source file implements all MESH mode interfaces of JDY-28M BLE/MESH module driver | 此源文件实现JDY-28M蓝牙MESH模块驱动的所有MESH模式接口
 * @note 1 tab == 4 spaces! This is the core source for JDY-28M MESH mode management | 这是JDY-28M MESH模式管理的核心源文件
 *
 * @version V1.0 2026-3-5
 *
 *****************************************************************************/

//******************************** Includes *********************************//
#include "mesh_mode.h"
#include "twr_control.h"
//******************************** Includes *********************************//

//******************************** Global Variables **************************//
/**
 * @brief JDY UART send buffer | JDY串口发送缓冲区
 * @details Global buffer for JDY UART AT command transmission | 用于JDY串口AT指令发送的全局缓冲区
 */
char jdy_uart_send[JDY_UART_MAX_SIZE] = {0};

/**
 * @brief JDY UART receive buffer | JDY串口接收缓冲区
 * @details Global buffer for JDY UART AT command response reception | 用于JDY串口AT指令响应接收的全局缓冲区
 */
char jdy_uart_recv[JDY_UART_MAX_SIZE] = {0};

/**
 * @brief MESH data send array | MESH数据发送数组
 * @details Global array for JDY-28M MESH packet transmission | 用于JDY-28M MESH数据包发送的全局数组
 */
uint8_t mesh_send_array[MESH_TX_ARRAY_SIZE] = {0};

/**
 * @brief MESH data receive array | MESH数据接收数组
 * @details Global array for JDY-28M MESH packet reception | 用于JDY-28M MESH数据包接收的全局数组
 */
uint8_t mesh_recv_array[MESH_RX_ARRAY_SIZE] = {0};

/**
 * @brief Default MESH send packet | 默认MESH发送数据包
 * @details Pre-configured MESH packet for JDY-28M transmission | 预配置的JDY-28M MESH发送数据包
 */
mesh_datasend_pkt_t my_mesh_send_pkt = {
    .valid = 0x00,       /* Packet valid flag (invalid) | 数据包有效标志（无效） */
    .header[0] = 0x41,   /* Packet header byte 0 | 数据包头部字节0 */
    .header[1] = 0x54,   /* Packet header byte 1 | 数据包头部字节1 */
    .header[2] = 0x2B,   /* Packet header byte 2 | 数据包头部字节2 */
    .header[3] = 0xAA,   /* Packet header byte 3 | 数据包头部字节3 */
    .header[4] = 0xA1,   /* Packet header byte 4 | 数据包头部字节4 */
    .to_maddr = 0x0000U, /* Target MESH address (user-defined, select_node_M()) | 目标MESH地址（用户自定义，select_node_M()） */
    .header_2 = 0xA0,    /* Secondary header | 二级头部 */
    .key = 0x0F,         /* User-defined key | 用户自定义密钥 */
    .L = 0x7F,           /* User-defined parameter L (joystick) | 用户自定义参数L（摇杆） */
    .R = 0x7F,           /* User-defined parameter R (joystick) | 用户自定义参数R（摇杆） */
    .end = 0x0D0AU       /* Packet end flag (CR+LF) | 数据包结束标志（回车+换行） */
};
//******************************** Global Variables **************************//

//******************************** Function Implementations ******************//
/**
 * @brief Set character buffer | 设置字符缓冲区
 * @details Clear destination buffer and copy source string to it | 清空目标缓冲区并将源字符串复制到其中
 * @param[in] dest      : Pointer to destination buffer | 指向目标缓冲区的指针
 * @param[in] dest_char : Pointer to source string | 指向源字符串的指针
 * @return none
 * @note Uses strlcpy to avoid buffer overflow | 使用strlcpy避免缓冲区溢出
 */
void setCharBuff(char *dest, const char *dest_char)
{
    memset(dest, 0, JDY_UART_MAX_SIZE);
    strlcpy(dest, dest_char, JDY_UART_MAX_SIZE);
}

/**
 * @brief JDY interrupt handler | JDY中断处理函数
 * @details Handle JDY-28M UART interrupt data reception | 处理JDY-28M串口中断数据接收
 * @param[in] self             : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] ble_datarev_buff : Pointer to BLE receive buffer | 指向BLE接收缓冲区的指针
 * @param[in] received         : Length of received data | 接收数据的长度
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 */
jdy_status_t JDY_func_it(Jdy_t *const self, uint8_t *ble_datarev_buff, uint16_t received)
{
    jdy_status_t res = JDY_OK;

    /*************1. Checking the input parameters**************/
    if (NULL == self ||
        NULL == ble_datarev_buff)
    {
        res = JDY_ERRORPARAMETER;
        return res;
    }
    /*************1. Checking the input parameters**************/

    /*************2. Checking MESH initialization status**************/
    if (JDY_INIT == self->p_mesh_submode->mash_init_flag)
    {
        res = JDY_OK;
        RingByteBuffer_pushBuffer(&ringBuffer4, ble_datarev_buff, received);
        return res;
    }
    /*************2. Checking MESH initialization status**************/

    /*************3. Handling waiting state data**************/
    if (self->p_mesh_submode->state == waiting)
    {
        for (int i = 0; i < received; i++)
        {
            uint8_t byte = ble_datarev_buff[i];
            jdy_uart_recv[i] = byte;
        }
        self->p_mesh_submode->state = handling;
    }
    /*************3. Handling waiting state data**************/

    return res;
}

/**
 * @brief Send JDY AT command without response | 发送无响应的JDY AT指令
 * @details Send AT command to JDY-28M and do not wait for response | 向JDY-28M发送AT指令，不等待响应
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] cmd  : Pointer to AT command string | 指向AT指令字符串的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 */
jdy_status_t dev_OK(Jdy_t *const self, const char *cmd)
{
    jdy_status_t ret = JDY_OK;

    /*************1. Checking current state**************/
    if (self->p_mesh_submode->state != idle)
    {
        ret = JDY_ERROR;
        return ret;
    }
    /*************1. Checking current state**************/

    /*************2. Sending AT command**************/
    self->p_tx->done_cb(self, (uint8_t *)cmd, strlen(cmd));
    /*************2. Sending AT command**************/

    return ret;
}

/**
 * @brief Send JDY AT command and wait for response | 发送有响应的JDY AT指令
 * @details Send AT command to JDY-28M and wait for response with timeout | 向JDY-28M发送AT指令，带超时等待响应
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] cmd  : Pointer to AT command string | 指向AT指令字符串的指针
 * @param[out] dest: Pointer to response buffer | 指向响应缓冲区的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note Blocking mode with JDY_UART_TIMEOUT | 阻塞模式，超时时间为JDY_UART_TIMEOUT
 */
jdy_status_t dev_query(Jdy_t *const self, const char *cmd, char *dest)
{
    uint32_t k = 0U;
    jdy_status_t ret = JDY_OK;
    size_t len = 0;

    /*************1. Waiting for idle state**************/
    k = self->p_time->getSysTickCnt();
    while (self->p_mesh_submode->state != idle)
    {
        if ((self->p_time->getSysTickCnt() - k) > JDY_UART_TIMEOUT)
        {
            ret = JDY_ERROR;
#ifdef JDY_DEBUG
            JDY_DEBUG_OUT("Error: JDY query timeout (waiting for idle).\n");
#endif
            return ret;
        }
    }
    /*************1. Waiting for idle state**************/

    /*************2. Preparing and sending AT command**************/
    self->p_mesh_submode->state = waiting;
    setCharBuff(jdy_uart_send, cmd);
    self->p_tx->done_cb(self, (uint8_t *)jdy_uart_send, strlen(jdy_uart_send));
    /*************2. Preparing and sending AT command**************/

    /*************3. Waiting for response with timeout**************/
    k = self->p_time->getSysTickCnt();
    while (self->p_mesh_submode->state == waiting)
    {
        if ((self->p_time->getSysTickCnt() - k) > JDY_UART_TIMEOUT)
        {
            self->p_mesh_submode->state = idle;
            ret = JDY_ERROR;
#ifdef JDY_DEBUG
            JDY_DEBUG_OUT("Error: JDY query timeout (waiting for response).\n");
#endif
            return ret;
        }
    }
    /*************3. Waiting for response with timeout**************/

    /*************4. Handling response data**************/
    if (self->p_mesh_submode->state == handling)
    {
        len = strlen(cmd) - 3;
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Received data length:    %d\n", len);
        JDY_DEBUG_OUT("Received data: %s\n", jdy_uart_recv);
#endif

        taskENTER_CRITICAL();
        strlcpy(dest, jdy_uart_recv + len, strlen(jdy_uart_recv) - len - 1);
        self->p_mesh_submode->state = idle;
        taskEXIT_CRITICAL();

#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Query result: %s\n", dest);
#endif

        ret = JDY_OK;
        return ret;
    }
    /*************4. Handling response data**************/

    return ret;
}

/**
 * @brief Initialize JDY-28M basic parameters | 初始化JDY-28M基础参数
 * @details Send a series of AT commands to initialize JDY-28M basic settings | 发送一系列AT指令初始化JDY-28M基础设置
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note 100ms/300ms/500ms delays are mandatory for stable operation | 100ms/300ms/500ms延时是稳定运行的必要条件
 */
jdy_status_t dev_init(Jdy_t *const self)
{
    // 实测，100ms延时是必须的
    dev_OK(self, "AT\r\n");
    self->p_time->my_delay(100);
    // dev_OK(self, "AT+DEFAULT\r\n"); // 默认，很关键
    // self->p_time->my_delay(100);
    dev_OK(self, "AT+RESET\r\n");
    self->p_time->my_delay(500); // 实测，300ms延时是必须的

    dev_OK(self, "AT\r\n");
    self->p_time->my_delay(100);
    dev_OK(self, "AT+ROLE2\r\n");
    self->p_time->my_delay(100);
    // dev_OK(self, "AT+ALED1\r\n");
    // self->p_time->my_delay(100);
    dev_OK(self, "AT+ENLOG0\r\n");
    self->p_time->my_delay(100);

    return JDY_OK;
}

/**
 * @brief Initialize JDY-28M MESH mode | 初始化JDY-28M MESH模式
 * @details Initialize JDY-28M basic parameters and configure MESH network | 初始化JDY-28M基础参数并配置MESH网络
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note MESH NETID can only be set, not queried | MESH NETID只能设置，不能查询
 */
jdy_status_t JDY_func_init(Jdy_t *const self)
{
    jdy_status_t res = JDY_OK;
    char NETID_buff[JDY_UART_MAX_SIZE] = {0};

    /*************1. Checking the input parameters**************/
    if (NULL == self)
    {
        res = JDY_ERRORPARAMETER;
        return res;
    }
    /*************1. Checking the input parameters**************/

    /*************2. Initializing JDY-28M basic parameters**************/
    dev_init(self);
    self->p_time->my_delay(100);
    /*************2. Initializing JDY-28M basic parameters**************/

    /*************3. Querying MAC and MADDR**************/
    dev_OK(self, "AT\r\n");
    self->p_time->my_delay(100);
    dev_query(self, "AT+MAC\r\n", self->p_mesh_submode->MAC);
#ifdef JDY_DEBUG
    JDY_DEBUG_OUT("MAC: %s\n", self->p_mesh_submode->MAC);
#endif
    self->p_time->my_delay(200);
    dev_query(self, "AT+MADDR\r\n", self->p_mesh_submode->MADDR);
    self->p_time->my_delay(200);
    /*************3. Querying MAC and MADDR**************/

    /*************4. Configuring MESH NETID**************/
    // 组网ID，只能设置不能查询
    snprintf(self->p_mesh_submode->NETID, 11, "%s,6", self->p_mesh_submode->MAC + 4);
    memset(NETID_buff, 0, JDY_UART_MAX_SIZE);
    snprintf(NETID_buff, JDY_UART_MAX_SIZE, "AT+NETID%s\r\n", self->p_mesh_submode->NETID);
    // 设置组网ID号 全部一样
    res = dev_OK(self, MESH_NETID);
    self->p_time->my_delay(200);
    res = dev_OK(self, MESH_MADDR);
    self->p_time->my_delay(200);
    // while (JDY_ERROR == res)
    // {
    //     res = dev_OK(self, MESH_MADDR);
    //     self->p_time->my_delay(300);
    // }
    /*************4. Configuring MESH NETID**************/

    /*************5. Resetting JDY-28M and marking initialization complete**************/
    dev_OK(self, "AT+RESET\r\n");
    self->p_time->my_delay(1000);
    self->p_mesh_submode->mash_init_flag = JDY_INIT; // 标记MESH初始化完成
    /*************5. Resetting JDY-28M and marking initialization complete**************/

    return res;
}

/**
 * @brief JDY MESH data receive handler | JDY MESH数据接收处理函数
 * @details Parse MESH packet from ring buffer | 从环形缓冲区解析MESH数据包
 * @param[in] self   : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] Parser : Pointer to ring buffer for MESH data | 指向MESH数据环形缓冲区的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 */
jdy_status_t my_mesh_datarecv(Jdy_t *const self, RingByteBuffer *Parser)
{
    mesh_datarecv_pkt_t pkt;
    uint8_t byte;
    uint8_t byte1;

    /*************1. Checking the input parameters and ring buffer**************/
    if (NULL == self ||
        NULL == Parser)
    {
        return JDY_ERROR;
    }

    if (RingByteBuffer_isEmpty(Parser))
    {
        return JDY_ERROR;
    }
    /*************1. Checking the input parameters and ring buffer**************/

    /*************2. Parsing MESH packet header**************/
    byte = RingByteBuffer_popByte(Parser);

HOP:
    if (0xF1 == byte)
    {

        /* 检查环形缓冲区长度 */
        if (RingByteBuffer_size(Parser) < (sizeof(mesh_datarecv_pkt) - 1))
        {
            return JDY_ERROR;
        }
        /* 取出 1 字节数据 */
        byte = RingByteBuffer_popByte(Parser);
        if (0xDD == byte)
        {
            pkt.header = 0xF1DD;
            byte = RingByteBuffer_popByte(Parser);
            pkt.netID = byte;
        }
        else
        {
            goto HOP;
        }
    }
    else
    {
        return JDY_ERROR;
    }
    /*************2. Parsing MESH packet header**************/

    /*************3. Parsing from_maddr**************/
    /* 取出 2 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    byte1 = RingByteBuffer_popByte(Parser);
    pkt.from_maddr = (byte << 8) | byte1;
    /*************3. Parsing from_maddr**************/

    /*************4. Parsing to_maddr**************/
    /* 取出 2 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    byte1 = RingByteBuffer_popByte(Parser);
    pkt.to_maddr = (byte << 8) | byte1;
    /*************4. Parsing to_maddr**************/

    /*************5. Parsing key**************/
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    pkt.key = byte;
    /*************5. Parsing key**************/

    /*************6. Parsing L**************/
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    pkt.L = byte;
    /*************6. Parsing L**************/

    /*************7. Parsing R**************/
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    pkt.R = byte;
    /*************7. Parsing R**************/

    /*************8. Parsing end and verifying**************/
    /* 取出 2 字节数据 */
    byte = RingByteBuffer_popByte(Parser);
    byte1 = RingByteBuffer_popByte(Parser);
    pkt.end = (byte << 8) | byte1;

    if (0x0D0A != pkt.end)
    {
        return JDY_ERROR;
    }
    /*************8. Parsing end and verifying**************/

    /*************9. Storing parsed packet**************/
    self->p_mesh_submode->p_parser->recv_pkt = pkt;
    /*************9. Storing parsed packet**************/

    return JDY_OK;
}

/**
 * @brief JDY MESH data send handler | JDY MESH数据发送处理函数
 * @details Assemble MESH packet and send to JDY-28M | 组装MESH数据包并发送到JDY-28M
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] pkt  : Pointer to MESH send packet | 指向MESH发送数据包的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note Send AT\r\n first for stable operation | 先发送AT\r\n以保证稳定运行
 */
jdy_status_t my_mesh_datasend(Jdy_t *const self, mesh_datasend_pkt_t *pkt)
{
    /*************1. Checking the input parameters**************/
    if (pkt->to_maddr == 0x0000U)
    {
        return JDY_ERROR;
    }
    if (pkt->valid == 0U)
    {
        return JDY_ERROR;
    }
    /*************1. Checking the input parameters**************/

    /*************2. Assembling MESH packet**************/
    memset(mesh_send_array, 0, MESH_TX_ARRAY_SIZE);
    // 实测，发送数据前先发给AT\r\n，才能稳定运行
    mesh_send_array[0] = 'A';
    mesh_send_array[1] = 'T';
    mesh_send_array[2] = '\r';
    mesh_send_array[3] = '\n';
    mesh_send_array[4] = pkt->header[0];
    mesh_send_array[5] = pkt->header[1];
    mesh_send_array[6] = pkt->header[2];
    mesh_send_array[7] = pkt->header[3];
    mesh_send_array[8] = pkt->header[4];
    mesh_send_array[9] = ((pkt->to_maddr) & 0xFF00) >> 8;
    mesh_send_array[10] = (pkt->to_maddr) & 0x00FF;
    mesh_send_array[11] = pkt->header_2;
    mesh_send_array[12] = pkt->key;
    mesh_send_array[13] = pkt->L;
    mesh_send_array[14] = pkt->R;
    mesh_send_array[15] = (pkt->end & 0xFF00) >> 8;
    mesh_send_array[16] = (pkt->end & 0x00FF);
    mesh_send_array[17] = '\0';

    /*************2. Assembling MESH packet**************/

    /*************3. Sending MESH packet**************/
    self->p_tx->done_cb(self, mesh_send_array, MESH_TX_ARRAY_SIZE);
    /*************3. Sending MESH packet**************/

    return JDY_OK;
}

/**
 * @brief  将int转换为uint8_t（最高位bit7为符号位，低7位存储数值）
 * @param  input_int: 输入的int值
 * @retval 转换后的uint8_t值（bit7=符号位，bit0~6=数值位）
 */
uint8_t int_to_uint8(int input_int)
{
    uint8_t sign_bit = 0;   // 符号位（bit7）：0=正数，1=负数
    uint8_t value_bits = 0; // 数值位（bit0~6）：范围 0~127
    int abs_val = 0;        // 输入int的绝对值

    // 1. 设置符号位 + 取绝对值
    if (input_int < 0)
    {
        sign_bit = 1; // 负数 → 符号位=1
        abs_val = -input_int;
    }
    else
    {
        sign_bit = 0; // 正数/0 → 符号位=0
        abs_val = input_int;
    }

    // 2. 限制数值范围：0~127（7位最大值）
    if (abs_val > 127)
    {
        value_bits = 127; // 超出上限 → 截断为127
    }
    else
    {
        value_bits = (uint8_t)abs_val; // 正常范围直接赋值
    }

    // 3. 组合：符号位左移7位 | 数值位
    uint8_t result = (sign_bit << 7) | value_bits;

    return result;
}

/**
 * @brief  将float转换为uint8_t（先取整，再限制0~255范围）
 * @param  input_float: 输入的float值
 * @retval 转换后的uint8_t值
 */
uint8_t float_to_uint8(float input_float)
{
    uint8_t sign_bit = 0;   // 符号位（bit7）：0=正，1=负
    uint8_t value_bits = 0; // 数值位（bit0~6）：0~127
    float abs_float = 0.0f; // float的绝对值

    // 1. 判断正负，设置符号位
    if (input_float < 0)
    {
        sign_bit = 1;             // 负数→bit7=1
        abs_float = -input_float; // 取绝对值
    }
    else
    {
        sign_bit = 0; // 正数→bit7=0
        abs_float = input_float;
    }

    // 2. 绝对值取整（默认：强制转换舍弃小数；如需四舍五入，改用round()）
    // 方式1：强制转换（舍弃小数，如3.9→3，127.8→127）
    int int_val = (int)abs_float;
    // 方式2：四舍五入（取消注释启用）
    // int int_val = (int)round(abs_float);

    // 3. 限制数值位范围：0~127（7位最大值）
    if (int_val > 127)
    {
        value_bits = 127; // 超出则截断为127
    }
    else if (int_val < 0)
    {
        value_bits = 0; // 理论上不会触发（已取绝对值），兜底
    }
    else
    {
        value_bits = (uint8_t)int_val;
    }

    // 4. 组合符号位和数值位：符号位左移7位 + 数值位
    uint8_t result = (sign_bit << 7) | value_bits;

    return result;
}

// ==================== 新增：反向还原函数 ====================
/**
 * @brief  uint8_t 反推回 int（对应修改后的 int_to_uint8）
 * @param  input_u8: 编码后的uint8_t
 * @retval 还原后的int值
 */
int uint8_to_int(uint8_t input_u8)
{
    // 拆分符号位 + 数值位
    uint8_t sign = (input_u8 >> 7) & 0x01;
    uint8_t val = input_u8 & 0x7F;

    // 还原正负
    if (sign == 1)
    {
        return -(int)val;
    }
    else
    {
        return (int)val;
    }
}

/**
 * @brief  uint8_t反推回float（对应float_to_uint8）
 * @param  input_u8: 正向转换后的uint8_t
 * @param  use_round: 0=舍弃小数还原，1=四舍五入还原（与正向编码对应）
 * @retval 还原后的float值
 */
float uint8_to_float(uint8_t input_u8, int use_round)
{
    // 1. 拆分符号位（bit7）和数值位（bit0~6）
    uint8_t sign_bit = (input_u8 >> 7) & 0x01; // 提取最高位
    uint8_t value_bits = input_u8 & 0x7F;      // 提取低7位（0~127）

    // 2. 还原数值
    float value = (float)value_bits;

    // 3. 还原符号
    if (sign_bit == 1)
    {
        value = -value;
    }

    // 4. 小数补偿（与正向编码的取整方式对应）
    if (use_round)
    {
        // 正向用了round()四舍五入 → 反推±0.5区间
        // 这里返回整数即可，代表原值在 [x-0.5, x+0.5)
        return value;
    }
    else
    {
        // 正向用了强制转int（舍弃小数）→ 反推代表原值在 [x, x+1)
        return value;
    }
}

float ceshi_angle = 0;
int ceshi_distance = 0;
extern StateMachine_Handle_t g_state_machine;
/*MADDR : 0x0001U 0x0002U 0x0003U 0x0004U */
jdy_status_t jdy_task(Jdy_t *const self, mesh_datasend_pkt_t *pkt, ProtocolData *data)
{
    jdy_status_t res;

    self->p_mesh_submode->p_parser->pf_mesh_datarecv_handler(self, &ringBuffer4);

/*头车就位后向一号中间车发送前进信号，*/    
/*头车接收到4G返回信号后向所有中间车发送返回信号，*/    
#ifdef AHAND_CAR
    if (1 == g_state_machine.ahand_flag)
    {
        pkt->to_maddr = 0x0002U; // user  //select_node_M()
        pkt->L = 1;
        pkt->R = 1;
        pkt->valid = 0x01; // user  //key
        res = jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(self, pkt);

        if (2 == self->p_mesh_submode->p_parser->recv_pkt.L)
        {
            g_state_machine.ahand_flag = 2;
            Buzzer_Start_Once(100);
        }
        //向所有中间车发送返回指令
    }else if (3 == g_state_machine.ahand_flag)
    {
        static uint8_t car_id = 0;
        car_id++; 

        if(car_id==0) {
            pkt->to_maddr = 0x0002U; // user  //select_node_M()
        }else if(car_id==1) {
            pkt->to_maddr = 0x0003U; // user  //select_node_M()
        }else {
            car_id=0;
        }

        
        pkt->L = 2;
        pkt->R = 2;
        pkt->valid = 0x01; // user  //key
        res = jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(self, pkt);

    }
#endif
/*后车接收到所有中间车返回信号后返回*/
#ifdef BEHIND_CAR

    static uint8_t car_id_2=0,car_id_3=0;
    if (3 == self->p_mesh_submode->p_parser->recv_pkt.L) car_id_2 = 1;
    if (4 == self->p_mesh_submode->p_parser->recv_pkt.L) car_id_3 = 1;

    if(car_id_2) {
        g_state_machine.behind_flag = 1;
    }


#endif



/*所有中间车接收到头车返回信号后集体向后车发送返回信号*/  
#ifdef MIDDLE_CAR
    if (2 == self->p_mesh_submode->p_parser->recv_pkt.L)
    {
        
        g_state_machine.middle_flag = 1;
        pkt->to_maddr = 0x0004U; // user  //select_node_M()
        pkt->L = 4;
        pkt->R = 4;
        pkt->valid = 0x01; // user  //key
        res = jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(self, pkt);
    }

#endif

 /*一号中间车接收到头车前进信号后前进*/ 
#ifdef MIDDLE_CAR_FIRST
    if (1 == self->p_mesh_submode->p_parser->recv_pkt.L)
    {
        
        g_state_machine.middle_flag = 1;

    }else if (2 == self->p_mesh_submode->p_parser->recv_pkt.L)
    {
        g_state_machine.middle_flag = 2;
        pkt->to_maddr = 0x0004U; // user  //select_node_M()
        pkt->L = 3;
        pkt->R = 3;
        pkt->valid = 0x01; // user  //key
        res = jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(self, pkt);
    }

#endif

    return res;
}
//******************************** Function Implementations ******************//