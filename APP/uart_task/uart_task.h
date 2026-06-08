#ifndef __UART_TASK_H
#define __UART_TASK_H 
#include "stdint.h"
#include "util/RingByteBuffer.h"
#include "stdint.h"
#include "usart.h"
#include "util/RingByteBuffer.h"
#include "Vofa.h"
#include "forward.h"
#include "forward.h"
#include "Parser.h"
#include "twr_control.h"
#include "jdy_driver.h"
#include "mesh_mode.h"
#include "bsp_4g.h"

#define RECEIVE_USART1_BUFFER 128
#define Transmit_USART1_BUFFER 128
// 环形队列缓冲区大小（可根据需求修改）
#define RING_BUFFER_SIZE 128

#define RXUART2_BUFFER 128
#define RXUART3_BUFFER 512

// 环形队列结构体
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];  // 数据缓冲区
    uint16_t head;                     // 队头指针（读取位置）
    uint16_t tail;                     // 队尾指针（写入位置）
    uint16_t len;                      // 当前数据长度
} RingBuffer_t;

// 环形队列操作函数
void RingBuffer_Init(RingBuffer_t *rb);                  // 初始化环形队列
uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb);            // 判断队列是否为空
uint8_t RingBuffer_IsFull(RingBuffer_t *rb);             // 判断队列是否为满
uint8_t RingBuffer_Write(RingBuffer_t *rb, uint8_t data); // 写入一个字节
uint8_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data); // 读取一个字节
uint16_t RingBuffer_GetLength(RingBuffer_t *rb);         // 获取队列中数据长度
jdy_status_t jdy_task(Jdy_t *const self, mesh_datasend_pkt_t *pkt, ProtocolData *data); /* JDY driver main task function | JDY驱动主任务函数 */
double TIM4_GetPWMFreq(void);
double TIM4_GetPWMCycle(void);

extern RingBuffer_t uart_ringbuffer;

extern uint32_t Encoder_voctor;
extern uint8_t dma_buffer_receive_usart1[RECEIVE_USART1_BUFFER];
extern uint8_t dma_buffer_Transmit_usart1[Transmit_USART1_BUFFER];
extern uint8_t usart2_rx_DMA_buffer[RXUART2_BUFFER];
extern uint8_t u2_ring_DMA_buffer[RXUART2_BUFFER];
extern RingBuffer_t uart2_ringbuffer;
extern uint8_t u3_ring_DMA_buffer[RXUART3_BUFFER];
extern uint8_t usart3_rx_DMA_buffer[RXUART3_BUFFER];
// 声明环形缓冲区管理变量
extern RingByteBuffer ring_rx_DMA_buf;
extern RingByteBuffer ring3_rx_DMA_buf;

// 声明底层存储数组
extern uint8_t usart2_rx_DMA_buffer[RXUART2_BUFFER];
extern uint8_t usart3_rx_DMA_buffer[RXUART3_BUFFER];
extern uint8_t u2_ring_DMA_buffer[RXUART2_BUFFER];
extern uint8_t u3_ring_DMA_buffer[RXUART3_BUFFER];
// 声明环形缓冲区管理变量
extern RingByteBuffer ring_rx_DMA_buf;
extern RingByteBuffer ring3_rx_DMA_buf;


#endif
