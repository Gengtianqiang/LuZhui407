#ifndef _BSP_4G_H
#define _BSP_4G_H


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Vofa.h"
#include "freertos.h"
#include "task.h"
#include "uart_task.h"
#include "util/RingByteBuffer.h"


#define DTU_RX_BUFFER_SIZE 256
#define FRAME_HEAD       '@'  // 帧头
#define FRAME_TAIL       'a'   // 帧尾

#define DTU_DEBUG
#define DTU_DEBUG_OUT(X,...) Vofa_Printf(&vofa_inst_binding_uart3, X, ##__VA_ARGS__);  /* Debug output interface | 调试输出接口 */



typedef struct dtu_s DTU_t;

/**
 * @brief JDY driver operation status code | JDY驱动操作状态码
 * @details Enumerate all possible return status of JDY driver APIs | 枚举JDY驱动所有API的可能返回状态
 * @note 0 indicates success, non-zero indicates error, reserved for future expansion | 0表示成功，非0表示错误，预留值用于未来扩展
 */
typedef enum {
    DTU_OK             = 0x01,      /* DTU Operation completed successfully | DTU操作执行成功 */
    DTU_ERROR          = 0x02,      /* DTU Run-time error without case matched | DTU运行时错误，无匹配场景 */
    DTU_ERRORTIMEOUT   = 0x03,      /* DTU Operation failed with timeout | DTU操作超时失败 */
    DTU_ERRORRESOURCE  = 0x04,      /* DTU Resource not available | DTU资源不可用 */
    DTU_ERRORPARAMETER = 0x05,      /* DTU Parameter error | DTU参数错误 */
    DTU_ERRORNOMEMORY  = 0x06,      /* DTU Out of memory | DTU内存不足 */
    DTU_ERRORISR       = 0x07,      /* DTU Not allowed in ISR context | DTU操作不允许在中断服务程序（ISR）中执行 */
    DTU_RESERVED       = 0x08,      /* DTU Reserved for future expansion | DTU预留值用于未来扩展 */
} dtu_status_t;



typedef enum {
    MSG_4G_BATTERY       = 0x01,  // 电量
    MSG_4G_PITCH_ANGLE   = 0x02,  // 俯仰角
    MSG_4G_ALARM_START   = 0x03,  // 开始报警
    MSG_4G_ONEKEY_STOP   = 0x04,  // 一键停止
    MSG_4G_ONEKEY_START  = 0x05,  // 一键出发
    MSG_4G_ONEKEY_RETURN = 0x06,  // 一键返回
    MSG_4G_SET_XY        = 0x07,  // 设置XY坐标
    MSG_4G_NONE          = 0x08,  // 无消息需要处理
} msg_4g_type_t;

typedef struct {
    void (*my_delay)(uint32_t   ms);   /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    uint32_t (*getSysTickCnt)(void); /* Get system tick count function | 获取系统滴答计数函数 */
}dtu_time_t;


typedef enum {
    DTU_NOT_INIT   = 0,      /* DTU driver not initialized | DTU驱动未初始化 */
    DTU_INIT       = 1       /* DTU driver initialized successfully | DTU驱动初始化成功 */
}dtu_init_t;

typedef struct {
    float x;
    float y;
}point_t;


struct dtu_s{
    dtu_init_t         dtu_init_flag;    /* DTU initialization flag | DTU是否初始化完成 */
    msg_4g_type_t              state;             /* Current working state | 当前工作状态 */
    uint8_t                    rx_flag;             /* 收否收到消息 */

    uint8_t                    buzzer_flag;             /* 收否收到消息 */

    point_t                point;

    //是否一键停止
    uint8_t                    stop_flag;  

    //出发
    uint8_t                    start_flag;  

     //返回
    uint8_t                    return_flag;
    

    //发送函数
    dtu_status_t (*send_fun)(uint8_t* buf, uint16_t len); 

    //时基
    dtu_time_t*        p_time;
    //解包函数（使用了ring buffer）
    dtu_status_t (*parser_fun)(DTU_t* const self, uint8_t* buf);
    //应答函数
    dtu_status_t (*ack_fun)(DTU_t* const);

};



dtu_status_t dtu_inst(DTU_t*          const self,
                        dtu_time_t*             p_time
);

dtu_status_t dtu_init(DTU_t*          const self
);
uint8_t dtu_get_data_from_ringbuf(RingByteBuffer *ring, uint8_t* buf);

extern DTU_t my_4g_dtu;
extern dtu_time_t dtu_time_config;
extern dtu_status_t dtu_init(DTU_t* const self);
extern uint8_t dtu_rx_buffer[DTU_RX_BUFFER_SIZE];

#endif