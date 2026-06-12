#include "uart_callback.h"
#include "jdy_driver.h"
#include "Vofa.h"
#include "bsp_4g.h"

Forward myForward = {0};



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t received = Size;

  if (huart->Instance == USART1)
  {
		RingByteBuffer_pushBuffer(&ring_rx_DMA_buf, uart1_dma_rx_buffer, received);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buffer, UART_RX_BUFFER_SIZE);
  }

  if (huart->Instance == UART4)
  {
    //        if (myForward.isForward == true)
    //            RingByteBuffer_pushBuffer(&ringBuffer4, uart4_dma_rx_buffer, received);
    //配合dev_query
    
    JDY_func_it(&jdy_handle, uart4_dma_rx_buffer, received);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_dma_rx_buffer, UART_RX_BUFFER_SIZE);
  }

  if (huart->Instance == USART2)
  {
		//pdoa
    if('M'==usart2_rx_DMA_buffer[0]&&'P'==usart2_rx_DMA_buffer[1]) {
        RingByteBuffer_pushBuffer(&ring_rx_DMA_buf, usart2_rx_DMA_buffer, received);
    }else {
        memset(usart2_rx_DMA_buffer,0,received);
    }
    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_DMA_buffer, RXUART2_BUFFER);
  }
  if (huart->Instance == USART3)
  {
		//twr
    
    RingByteBuffer_pushBuffer(&ring3_rx_DMA_buf, usart3_rx_DMA_buffer, received);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_DMA_buffer, RXUART3_BUFFER);
  }

  if (huart->Instance == UART5)
  {    
    my_4g_dtu.rx_flag = 1; 
    RingByteBuffer_pushBuffer(&ring5_rx_DMA_buf, u5_ring_DMA_buffer, received);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, u5_ring_DMA_buffer, RXUART5_BUFFER);
   }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    // 清除错误标志
    uint32_t sr = huart->Instance->SR;
    uint8_t dr = huart->Instance->DR;
    (void)sr;
    (void)dr;

    // 恢复空闲DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_DMA_buffer, RXUART3_BUFFER);
  }

  if (huart->Instance == USART2)
  {
    // 清除错误标志
    uint32_t sr = huart->Instance->SR;
    uint8_t dr = huart->Instance->DR;
    (void)sr;
    (void)dr;

    // 恢复空闲DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_DMA_buffer, RXUART2_BUFFER);
  }
	
	if(huart->Instance == USART1) {
		    // 清除错误标志
    uint32_t sr = huart->Instance->SR;
    uint8_t dr = huart->Instance->DR;
    (void)sr;
    (void)dr;
		
	}
  	if(huart->Instance == UART4) {
		    // 清除错误标志
    uint32_t sr = huart->Instance->SR;
    uint8_t dr = huart->Instance->DR;
    (void)sr;
    (void)dr;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_dma_rx_buffer, UART_RX_BUFFER_SIZE);
	}
}






// // 重写HAL库官方回调函数
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//     // 判断是串口4发送完成
//     if(huart == &huart4)
//     {
//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//         // 中断中释放信号量，通知任务「发送完成」
//         xSemaphoreGiveFromISR(jdy_handle.p_mesh_submode->p_parser->uart_tx_sem, &xHigherPriorityTaskWoken);

//         // 切换任务（必要操作）
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
// }



void UART_Init(void)
{
    
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

