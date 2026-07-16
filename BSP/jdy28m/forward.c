#include "forward.h"
#include "Vofa.h"
#include "uart_callback.h"

uint8_t uart1_dma_rx_buffer[UART_RX_BUFFER_SIZE]; 
uint8_t uart4_dma_rx_buffer[UART_RX_BUFFER_SIZE]; 
uint8_t ring_buffer_data_1[RING_BUFFER_SIZE];      
uint8_t ring_buffer_data_4[RING_BUFFER_SIZE];     
uint8_t ring_buffer_data_Parser[RING_BUFFER_SIZE]; 
RingByteBuffer ringBuffer1;
RingByteBuffer ringBuffer4;
RingByteBuffer ringBuffer_Parser;
uint8_t usart2_rx_DMA_buffer[RXUART2_BUFFER];
uint8_t usart3_rx_DMA_buffer[RXUART3_BUFFER];
uint8_t u2_ring_DMA_buffer[RXUART2_BUFFER];
uint8_t u3_ring_DMA_buffer[RXUART3_BUFFER];
uint8_t u5_ring_DMA_buffer[RXUART3_BUFFER];

RingByteBuffer ring_rx_DMA_buf;
RingByteBuffer ring3_rx_DMA_buf;
RingByteBuffer ring5_rx_DMA_buf;


uint8_t Vofa_RxFlag = 0;
uint8_t Vofa_Buffer[UART_RX_BUFFER_SIZE] = {0};
HAL_StatusTypeDef u3_ceshi;

void UART_DMA_Init(enum ForwardData_Key On_Off)
{

    if (On_Off == ForwardData_On)
        myForward.isForward = true;
    else
        myForward.isForward = false;

    
    RingByteBuffer_init(&ringBuffer1, ring_buffer_data_1, RING_BUFFER_SIZE);
    RingByteBuffer_init(&ringBuffer4, ring_buffer_data_4, RING_BUFFER_SIZE);
    RingByteBuffer_init(&ringBuffer_Parser, ring_buffer_data_Parser, RING_BUFFER_SIZE);
    RingByteBuffer_init(&ring_rx_DMA_buf, u2_ring_DMA_buffer, RXUART2_BUFFER);
    RingByteBuffer_init(&ring3_rx_DMA_buf, u3_ring_DMA_buffer, RXUART3_BUFFER);
    RingByteBuffer_init(&ring5_rx_DMA_buf, u5_ring_DMA_buffer, RXUART5_BUFFER);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buffer, UART_RX_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_dma_rx_buffer, UART_RX_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_DMA_buffer, RXUART2_BUFFER);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_DMA_buffer, RXUART3_BUFFER);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, u5_ring_DMA_buffer, RXUART5_BUFFER);
}




void ForwardData_LOOP(void)
{
    uint16_t len;

    static uint8_t temp_buffer_4[UART_RX_BUFFER_SIZE];
    static uint8_t temp_buffer_1[UART_RX_BUFFER_SIZE]; 

    if (myForward.isForward != true)
        return;

    len = RingByteBuffer_size(&ringBuffer4);
    if (len > 0)
    {
        len = (len > sizeof(temp_buffer_4)) ? sizeof(temp_buffer_4) : len;
        RingByteBuffer_popBuffer(&ringBuffer4, temp_buffer_4, len);
        uint32_t lasttime = HAL_GetTick();
        while (huart1.gState != HAL_UART_STATE_READY)
        {
            if (HAL_GetTick() - lasttime > 1000u)
            {
                break;
            }
        }
        HAL_UART_Transmit_DMA(&huart1, temp_buffer_4, len);
    }

    len = RingByteBuffer_size(&ringBuffer1);
    if (len > 0)
    {
        len = (len > sizeof(temp_buffer_1)) ? sizeof(temp_buffer_1) : len;
        RingByteBuffer_popBuffer(&ringBuffer1, temp_buffer_1, len);
        uint32_t lasttime = HAL_GetTick();
        while (huart4.gState != HAL_UART_STATE_READY)
        {
            if (HAL_GetTick() - lasttime > 1000u)
            {
                break;
            }
        }
        HAL_UART_Transmit_DMA(&huart4, temp_buffer_1, len);
    }
}
