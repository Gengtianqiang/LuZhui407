#ifndef __FORWARD_H
#define __FORWARD_H

#include "util/RingByteBuffer.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define UART_RX_BUFFER_SIZE 256
#define RING_BUFFER_SIZE 256
#define RXUART2_BUFFER 128
#define RXUART3_BUFFER 512
#define RXUART5_BUFFER 512
extern uint8_t Vofa_RxFlag;
extern uint8_t Vofa_Buffer[UART_RX_BUFFER_SIZE];
extern RingByteBuffer ring_rx_DMA_buf;
extern RingByteBuffer ring3_rx_DMA_buf;
extern RingByteBuffer ring5_rx_DMA_buf;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern uint8_t ring_buffer_data_Vofa[64];
extern RingByteBuffer ringBuffer_Vofa;

extern uint8_t uart1_dma_rx_buffer[UART_RX_BUFFER_SIZE];
extern uint8_t uart4_dma_rx_buffer[UART_RX_BUFFER_SIZE];
extern uint8_t ring_buffer_data_1[RING_BUFFER_SIZE];
extern uint8_t ring_buffer_data_4[RING_BUFFER_SIZE];
extern uint8_t ring_buffer_data_Parser[RING_BUFFER_SIZE];
extern RingByteBuffer ringBuffer1;
extern RingByteBuffer ringBuffer4;
extern RingByteBuffer ringBuffer_Parser;
extern uint8_t usart2_rx_DMA_buffer[RXUART2_BUFFER];
extern uint8_t usart3_rx_DMA_buffer[RXUART3_BUFFER];
extern uint8_t u2_ring_DMA_buffer[RXUART2_BUFFER];
extern uint8_t u3_ring_DMA_buffer[RXUART3_BUFFER];
extern uint8_t u5_ring_DMA_buffer[RXUART5_BUFFER];



enum ForwardData_Key
{
    ForwardData_Off = 0,
    ForwardData_On
};

void UART_DMA_Init(enum ForwardData_Key On_Off);
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);
void ForwardData_LOOP(void);

#endif
