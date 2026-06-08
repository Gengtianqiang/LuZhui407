// file encoding utf-8
// Thanks To jelin-sh/VOFA-Protocol-Driver

#include "Vofa.h"

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "util/RingByteBuffer.h"

// #define VOFA_RING_BUFFER_SIZE  64   // DMA缓冲区大小
uint8_t ring_buffer_data_Vofa[64] = {0};  // USART1 → Vofa 缓冲区
RingByteBuffer ringBuffer_Vofa = {0};     // USART1 → Vofa 环形缓冲区

void Vofa_if_Init(Vofa_HandleTypedef *handle)
{
    Vofa_Init(handle, VOFA_MODE_SKIP);
    RingByteBuffer_init(&ringBuffer_Vofa, ring_buffer_data_Vofa, 64);// 初始化环形缓冲区
    __HAL_UART_ENABLE_IT(&VOFA3_HUART, UART_IT_RXNE);
}

void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
    UNUSED(handle);
    uint32_t lasttime = HAL_GetTick();
    while( VOFA3_HUART.gState != HAL_UART_STATE_READY ){
        if( HAL_GetTick() - lasttime > 1000u ){ break; } }
    HAL_UART_Transmit(&VOFA3_HUART, data, length, 1000u);
	return;
}

uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
    return RingByteBuffer_popByte(&ringBuffer_Vofa);
}
