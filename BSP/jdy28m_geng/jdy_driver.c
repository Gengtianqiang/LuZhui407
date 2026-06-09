/******************************************************************************
 * Copyright (C) 2024 gengshuaige, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file jdy_driver.c
 *
 * @par dependencies
 * - jdy_driver.h
 *
 * @author Yharim
 *
 * @brief HAL level JDY-28M driver implementation | HAL层JDY-28M驱动实现
 * @details This source file implements all interfaces of JDY-28M BLE/MESH module driver | 此源文件实现JDY-28M蓝牙MESH模块驱动的所有接口
 * @note 1 tab == 4 spaces! This is the core source for JDY-28M driver instance management | 这是JDY-28M驱动实例管理的核心源文件
 *
 * @version V1.0 2026-3-5
 *
 *****************************************************************************/

//******************************** Includes *********************************//
#include "jdy_driver.h"
//******************************** Includes *********************************//

//******************************** Defines **********************************//

/**
 * @brief JDY MESH initialization function | JDY MESH初始化函数
 * @details Default implementation of MESH initialization | MESH初始化的默认实现
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
jdy_status_t my_mesh_init(Jdy_t *const self)
{
    // 用户需在实例化时提供具体的MESH初始化函数实现
    return JDY_func_init(self);
}

/**
 * @brief JDY transmit completion callback function | JDY发送完成回调函数
 * @details Default implementation of transmit completion callback | 发送完成回调的默认实现
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] buf  : Pointer to transmit data buffer | 指向发送数据缓冲区的指针
 * @param[in] len  : Length of transmit data | 发送数据的长度
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
jdy_status_t my_send_callback(Jdy_t *const self, uint8_t *buf, uint16_t len)
{
    if (NULL == self ||
        NULL == buf  ||
        0    == len)
    {
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Error: Invalid input parameter for send callback.\n");
#endif
        return JDY_ERRORPARAMETER;
    }

    if (self->p_tx->is_busy)
    {
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Error: Transmit is busy, cannot send data.\n");
#endif
        return JDY_ERRORRESOURCE;
    }
    // 用户需在实例化时提供具体的发送/接收完成回调函数实现
    //
    taskENTER_CRITICAL();
    HAL_UART_Transmit_DMA(&huart4, buf, len);
    taskEXIT_CRITICAL();

    return JDY_OK;
}

/**
 * @brief JDY receive completion callback function | JDY接收完成回调函数
 * @details Default implementation of receive completion callback | 接收完成回调的默认实现
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] buf  : Pointer to receive data buffer | 指向接收数据缓冲区的指针
 * @param[in] len  : Length of receive data | 接收数据的长度
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
__weak jdy_status_t my_recv_callback(Jdy_t *const self, uint8_t *buf, uint16_t len)
{
    // 用户需在实例化时提供具体的发送/接收完成回调函数实现
    return JDY_OK;
}

/**
 * @brief JDY custom function | JDY自定义功能函数
 * @details Default implementation of custom function | 自定义功能的默认实现
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @return none
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
void my_custom_function(Jdy_t *const self)
{
    // 用户需在实例化时提供具体的自定义功能函数实现
}

/**
 * @brief JDY OS delay function | JDY OS延时函数
 * @details Default implementation of OS delay | OS延时的默认实现
 * @param[in] ms : Delay time in milliseconds | 延时时间（毫秒）
 * @return none
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
void my_delay_ms(uint32_t ms)
{
    // 用户需在实例化时提供具体的OS延时函数实现
    // osDelay( ms);
    HAL_Delay(ms);
}

/**
 * @brief JDY get system tick count function | JDY获取系统滴答计数函数
 * @details Default implementation of get system tick count | 获取系统滴答计数的默认实现
 * @return uint32_t : Current system tick count | 当前系统滴答计数
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
uint32_t my_GetTick(void)
{
    // 用户需在实例化时提供具体的获取系统滴答计数函数实现
    return xTaskGetTickCount();
}

/**
 * @brief JDY MESH data receive handler | JDY MESH数据接收处理函数
 * @details Default implementation of MESH data receive handler | MESH数据接收处理的默认实现
 * @param[in] self             : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] ringBuffer_Parser : Pointer to ring buffer for MESH data | 指向MESH数据环形缓冲区的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
__weak jdy_status_t my_mesh_datarecv(Jdy_t* const self, RingByteBuffer *ringBuffer_Parser)
{
    // 用户需在实例化时提供具体的MESH数据接收处理函数实现
    return JDY_OK;
}

/**
 * @brief JDY MESH data send handler | JDY MESH数据发送处理函数
 * @details Default implementation of MESH data send handler | MESH数据发送处理的默认实现
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @param[in] pkt  : Pointer to MESH send packet | 指向MESH发送数据包的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note User can override this function with custom implementation | 用户可通过自定义实现覆盖此函数
 */
__weak jdy_status_t my_mesh_datasend(Jdy_t* const self, mesh_datasend_pkt_t* pkt)
{
    // 用户需在实例化时提供具体的MESH数据发送处理函数实现
    return JDY_OK;
}

/**
 * @brief JDY time interface configuration | JDY时间接口配置
 * @details Default configuration of time interface | 时间接口的默认配置
 */
jdy_time_t time_config = {
    .my_delay       = my_delay_ms,   /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    .getSysTickCnt  = my_GetTick     /* Get system tick count function | 获取系统滴答计数函数 */
};

/**
 * @brief JDY transmit interface configuration | JDY发送接口配置
 * @details Default configuration of transmit interface | 发送接口的默认配置
 */
jdy_tx_t tx_config = {
    .is_busy  = false,            /* Default transmit status (idle) | 默认发送状态（空闲） */
    .done_cb  = my_send_callback  /* Transmit completion callback function | 发送完成回调函数 */
};

/**
 * @brief JDY receive interface configuration | JDY接收接口配置
 * @details Default configuration of receive interface | 接收接口的默认配置
 */
jdy_rx_t rx_config = {
    .is_busy  = false,            /* Default receive status (idle) | 默认接收状态（空闲） */
    .done_cb  = my_recv_callback  /* Receive completion callback function | 接收完成回调函数 */
};

/**
 * @brief JDY extended function configuration | JDY拓展功能配置
 * @details Default configuration of extended function | 拓展功能的默认配置
 */
jdy_function_t function_config = {
    .pf_custom_function = my_custom_function /* User custom function pointer | 用户自定义功能函数指针 */
};

/**
 * @brief JDY MESH parser configuration | JDY MESH解析器配置
 * @details Default configuration of MESH parser | MESH解析器的默认配置
 */
mash_parser_t mesh_parser_config = {
    .pf_mesh_datarecv_handler = my_mesh_datarecv, /* MESH data receive handler | MESH数据接收处理函数 */
    .pf_mesh_datasend_handler = my_mesh_datasend  /* MESH data send handler | MESH数据发送处理函数 */
};

/**
 * @brief JDY MESH submode configuration | JDY MESH子模式配置
 * @details Default configuration of MESH submode | MESH子模式的默认配置
 */
jdy_mesh_t mesh_config = {
    .mash_init_flag =          JDY_NOT_INIT,       /* Default MESH initialization status (not initialized) | 默认MESH初始化状态（未完成） */
    .state          =                  idle,                /* Default MESH working state (idle) | 默认MESH工作状态（空闲） */
    .flage          =                   {0},                 /* Default status flag bits (all 0) | 默认状态标志位（全0） */
    .MAC            =                   {0},                 /* Default MAC address (empty) | 默认MAC地址（空） */
    .STAT           = JDY_STAT_DISCONNECTED, /* Default MESH connection status (disconnected) | 默认MESH连接状态（未连接） */
    .MADDR          =                   {0},                 /* Default MESH short address (empty) | 默认MESH短地址（空） */
    .NETID          =                   {0},                 /* Default MESH network ID (empty) | 默认MESH组网ID（空） */
    .mesh_init      =          my_mesh_init,        /* MESH initialization function | MESH初始化函数 */
    .p_parser       =   &mesh_parser_config  /* Pointer to MESH parser structure | 指向MESH解析结构体的指针 */
};

/**
 * @brief JDY driver instance | JDY驱动实例
 * @details Global JDY driver instance | 全局JDY驱动实例
 */
Jdy_t jdy_handle = {0};

/**
 * @brief JDY driver initialization function | JDY驱动初始化函数
 * @details Internal initialization function for JDY driver instance | JDY驱动实例的内部初始化函数
 * @param[in] self : Pointer to JDY driver instance | 指向JDY驱动实例的指针
 * @return jdy_status_t : Operation status (JDY_OK for success) | 操作状态（JDY_OK表示成功）
 * @note Called by jdy_inst() automatically | 由jdy_inst()自动调用
 */
jdy_status_t jdy_init(Jdy_t *const self)
{
    jdy_status_t status = JDY_OK;

    /*************1. Checking the input parameters**************/
    if (NULL == self)
    {
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Error: Invalid input parameter for JDY driver initialization.\n");
        status = JDY_ERRORPARAMETER;
#endif
        return JDY_ERRORPARAMETER;
    }
    /*************1. Checking the input parameters**************/

    self->mode          = JDY_MODE_MESH_WITH_FRAME; /* Default working mode (MESH with frame) | 默认工作模式（MESH有帧格式） */
    self->p_rx->is_busy =                    false;            /* Default receive status (idle) | 默认接收状态（空闲） */
    self->p_tx->is_busy =                    false;            /* Default transmit status (idle) | 默认发送状态（空闲） */
    self->p_mesh_submode->mash_init_flag = JDY_NOT_INIT; /* Default MESH initialization status (not initialized) | 默认MESH初始化状态（未完成） */
        // 初始释放一次信号量，让第一次发送可以正常执行
    xSemaphoreGive(self->p_mesh_submode->p_parser->uart_tx_sem);
#ifdef JDY_DEBUG
    JDY_DEBUG_OUT("init success.\n");
#endif

    return status;
}

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
jdy_status_t jdy_inst(Jdy_t *const          self,
                      jdy_mesh_t* p_mesh_submode,
                      jdy_tx_t*             p_tx,
                      jdy_rx_t*             p_rx,
                      jdy_time_t*         p_time,
                      jdy_function_t* p_function
)
{
    /*************1. Checking the input parameters**************/
    jdy_status_t res = JDY_OK;

    if (NULL == self           ||
        NULL == p_mesh_submode ||
        NULL == p_tx           ||
        NULL == p_rx           ||
        NULL == p_time         ||
        NULL == p_function)
    {
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Error: Invalid input parameters for JDY driver instantiation.\n");
#endif
        return JDY_ERRORPARAMETER;
    }
    /*************1. Checking the input parameters**************/

    if (JDY_NOT_INIT != self->init_status)
    {
        res = JDY_ERRORRESOURCE;
        return res;
    }

    /*************2. Binding the interfaces**************/
    self->p_tx           =           p_tx;
    self->p_rx           =           p_rx;
    self->p_mesh_submode = p_mesh_submode;
    self->p_time         =         p_time;
    self->p_function     =     p_function;
    /*************2. Binding the interfaces**************/
    //使用操作系统分配堆内存
    self->p_mesh_submode->p_parser->uart_tx_sem = xSemaphoreCreateBinary();
    /*************3. Initializing the driver**************/
    jdy_init(self);
    /*************3. Initializing the driver**************/

    if (JDY_OK != res)
    {
        self->p_mesh_submode = NULL;
        self->p_tx           = NULL;
        self->p_rx           = NULL;
        self->p_time         = NULL;
        self->p_function     = NULL;
#ifdef JDY_DEBUG
        JDY_DEBUG_OUT("Error: JDY driver initialization failed.\n");
#endif
        return res;
    }

#ifdef JDY_DEBUG
    JDY_DEBUG_OUT("JDY driver instantiation completed.\n");
#endif

    self->init_status = JDY_INIT;
    return res;
}

//******************************** Defines **********************************//

