#include "uart_callback.h"
#include "jdy_driver.h"
#include "Vofa.h"
#include "bsp_4g.h"
#include "mesh_mode.h"
#include "main.h"

Forward myForward = {0};
uint8_t rx_flag = 0;

/* 此工程运行在 Slot A，串口命令的目标为 Slot B。 */
#define APP_UPDATE_REQUEST_ADDR          0x2001FFF0UL
#define APP_UPDATE_REQUEST_MAGIC         0x55504454UL
#define APP_REQUEST_ACTION_IAP           0x49415031UL
#define APP_REQUEST_ACTION_SWITCH_OR_IAP 0x53574950UL
#define APP_SLOT_B                       1UL

typedef enum
{
    APP_OTA_COMMAND_NONE = 0U,
    APP_OTA_COMMAND_SWITCH_OR_IAP,
    APP_OTA_COMMAND_FORCE_IAP
} app_ota_command_t;

/* DMA 回调和 USART 任务之间共享；8 位读写在 Cortex-M 上是原子的。 */
static volatile uint8_t app_ota_command = APP_OTA_COMMAND_NONE;

/*
 * 允许串口助手发送 U、U\r、U\n 或 U\r\n。
 * 只接受首字节为命令、其余字节仅为行结束符的短帧，业务报文不会被误判。
 */
static bool App_IsUsart1OtaFrame(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == NULL) || (length == 0U) || (length > 3U))
    {
        return false;
    }

    for (index = 1U; index < length; index++)
    {
        if ((data[index] != '\r') && (data[index] != '\n'))
        {
            return false;
        }
    }

    return true;
}

static void App_RequestBootloader(uint32_t action, uint32_t target_slot)
{
    volatile uint32_t *request = (volatile uint32_t *)APP_UPDATE_REQUEST_ADDR;

    __disable_irq();

    /*
     * 与 Boot_TakeApplicationRequest() 的协议一致：
     * 先写入动作和目标分区，最后才写 magic，防止复位发生在半写入状态。
     */
    request[0] = 0U;
    request[1] = 0U;
    request[2] = action;
    request[3] = target_slot;
    __DSB();
    request[1] = ~APP_UPDATE_REQUEST_MAGIC;
    request[0] = APP_UPDATE_REQUEST_MAGIC;
    __DSB();

    NVIC_SystemReset();
    while (1)
    {
    }
}

void App_ProcessUsart1OtaCommand(void)
{
    app_ota_command_t command = (app_ota_command_t)app_ota_command;

    if (command == APP_OTA_COMMAND_NONE)
    {
        return;
    }

    /* 先清除标志，避免复位请求被其他任务再次处理。 */
    app_ota_command = APP_OTA_COMMAND_NONE;

    if (command == APP_OTA_COMMAND_SWITCH_OR_IAP)
    {
        App_RequestBootloader(APP_REQUEST_ACTION_SWITCH_OR_IAP, APP_SLOT_B);
    }

    App_RequestBootloader(APP_REQUEST_ACTION_IAP, APP_SLOT_B);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t received = Size;

  if (huart->Instance == USART1)
  {
		//RingByteBuffer_pushBuffer(&ring_rx_DMA_buf, uart1_dma_rx_buffer, received);
    rx_flag = 1;

    /*
     * USART1 同时承担业务通信。仅接受“命令字符 + 可选 CR/LF”的短帧，
     * 避免业务报文中的 U/I/B 被误识别，同时兼容串口助手自动附加换行。
     */
    if (App_IsUsart1OtaFrame(uart1_dma_rx_buffer, received))
    {
        switch (uart1_dma_rx_buffer[0])
        {
            case 'U':
            case 'u':
                app_ota_command = APP_OTA_COMMAND_SWITCH_OR_IAP;
                break;

            case 'I':
            case 'i':
            case 'B':
            case 'b':
                app_ota_command = APP_OTA_COMMAND_FORCE_IAP;
                break;

            default:
                break;
        }
    }

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


    if (App_IsUsart1OtaFrame(u5_ring_DMA_buffer, received))
    {
        switch (u5_ring_DMA_buffer[0])
        {
            case 'U':
            case 'u':
                app_ota_command = APP_OTA_COMMAND_SWITCH_OR_IAP;
                break;

            case 'I':
            case 'i':
            case 'B':
            case 'b':
                app_ota_command = APP_OTA_COMMAND_FORCE_IAP;
                break;

            default:
                break;
        }
    }

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
		/* USART1 出错后必须重新启动 DMA，否则 OTA/业务接收都会停止。 */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buffer, UART_RX_BUFFER_SIZE);
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



void MY_UART_Init(void)
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

