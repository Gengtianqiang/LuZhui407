#include "bsp_4g.h"

uint8_t dtu_rx_buffer[DTU_RX_BUFFER_SIZE] = {0}; // UART5接收缓冲区

DTU_t my_4g_dtu;




//发送函数
dtu_status_t dtu_send_callback(uint8_t* buf, uint16_t len) {
    HAL_UART_Transmit_DMA(&huart5, buf, len);
}


//应答函数
char ack_buf[100] = {0};
dtu_status_t dtu_ack(DTU_t* const self) {

    

    switch (self->state)
    {

    case MSG_4G_BATTERY:
        // 处理电量消息
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "Battery level is %.1fv\n",Volt);
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;  
    case MSG_4G_PITCH_ANGLE:
        // 处理俯仰角消息
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "Pitch angle is %.1f\n",myimu.euler.pitch);
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ALARM_START:
        // 处理开始报警消息
        memset(ack_buf, 0, sizeof(ack_buf));
    self->buzzer_flag += 1;
    if(self->buzzer_flag==2)self->buzzer_flag = 0;
        sprintf((char*)ack_buf, "Alarm %s\n", self->buzzer_flag ? "ON" : "OFF");
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_STOP:
        // 处理一键停止消息
        self->stop_flag += 1;
        if(self->stop_flag==2)self->stop_flag = 0;
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key stop %s\n", self->stop_flag ? "Activated" : "Deactivated");
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_START:
        // 处理一键出发消息
            
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key start received.\n");
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_ONEKEY_RETURN:
        // 处理一键返回消息
        
        self->return_flag = 1;
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "One-key return received.\n");
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    case MSG_4G_SET_XY:
        // 处理设置XY坐标消息
        memset(ack_buf, 0, sizeof(ack_buf));
        sprintf((char*)ack_buf, "Set XY coordinates received.\n");
        self->send_fun(ack_buf, strlen((char*)ack_buf));
        break;
    default:
        break;
    }

    self->state = MSG_4G_NONE; // 处理完消息后重置状态

    return DTU_OK;
}

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

uint8_t DTU_ParseFrame(DTU_t* const self,uint8_t *buf,uint16_t len)
{
    // 1. 检查帧头、帧尾、长度
    if(buf[0] != FRAME_HEAD || buf[len-1] != FRAME_TAIL) {



        return 0;
    }
        
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



    // 5. 校验全部正确 → 存入全局变量
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
    
    self->ack_fun(self); // 解析成功后立即调用应答函数

#ifdef DTU_DEBUG
    DTU_DEBUG_OUT("Parsed frame with command: 0x%02X, updated state to: %d\n", g_dtu_cmd, self->state);
#endif

    return 1; // 解析成功
}

//解包函数（使用了ring buffer）
dtu_status_t dtu_parser(DTU_t* const self, uint8_t* buf) {
   if(my_4g_dtu.rx_flag==1) {
        if(dtu_get_data_from_ringbuf(&ring5_rx_DMA_buf, buf)) {
        }else {
#ifdef DTU_DEBUG
            DTU_DEBUG_OUT("No data received from ring buffer.\n");  
#endif
        }

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
    

    my_4g_dtu.rx_flag=0;
   }

    
}

void __my_delay_ms(uint32_t ms)
{
    // 用户需在实例化时提供具体的OS延时函数实现
    osDelay( ms);
}


uint32_t __my_GetTick(void)
{
    // 用户需在实例化时提供具体的获取系统滴答计数函数实现
    return xTaskGetTickCount();
}


//将环形缓冲区数据接出来
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




dtu_time_t dtu_time_config = {
    .my_delay       = __my_delay_ms,   /* OS delay function (delay in milliseconds) | OS延时函数（以毫秒为单位） */
    .getSysTickCnt  = __my_GetTick     /* Get system tick count function | 获取系统滴答计数函数 */
};





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



