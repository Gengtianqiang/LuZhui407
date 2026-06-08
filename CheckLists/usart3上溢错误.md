# 🛠️ 问题排查记录

---

## 📝 一、问题描述

| 项目     | 内容填写区                                                   |
| -------- | ------------------------------------------------------------ |
| 问题表现 | stm32f407,  HAL,  usart3,  波特率115200，ORE寄存器置位       |
| 问题描述 | A是带电池的串口设备，B是stm32f407单片机，A向B通信时会引发上溢错误 |
| 核心信息 | `void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)` |

通信数据：

上电时会有一次580字节的数据量：

```
**********************DEVICE CONFIG**********************
* model = LD_PA
* firmware = V71
* role = TAG
* addr = 0
* blink = 0
* max_anc_num = 4
* max_tag_num = 4
* uwb_data_rate = 110K
* channel = CH2
* update_frequency = 8Hz
* update_Period = 112ms
* kalmanfilter = 1
* ant_dly = 16485
* tx_power = 9c9c9c9c
* use_ext_eeprom = 1
* use_imu = 9
* group_id = 0
* pan_id = 57034
* A0(0.00, 0.00, 1.00) A1(10.00, 0.00, 1.00)
* A2(0.00, 10.00, 1.00) A3(10.00, 10.00, 1.00)
***************************END***************************
```

没有基站时会持续产生58字节的错误信息，周期100ms

```
$RANGE_ERROR,ID=0,rb=15,rangetime=5702,battery=0,sos=00
```

有基站时会持续产生113字节的正确信息，周期100ms

```
mc 00 ffffffff 00000172 ffffffff ffffffff 0069 13 00078806 t0:0 0000
$KT0,null,0.37,null,null,LO=[no solution]

```



使用空闲中断+DMA+环形缓冲区：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)

{

 uint16_t received = Size;

 if (huart->Instance == USART1)

 {

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buffer, UART_RX_BUFFER_SIZE);

 }

 if (huart->Instance == UART4)

 {

  JDY_func_it(&jdy_handle, uart4_dma_rx_buffer, received);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_dma_rx_buffer, UART_RX_BUFFER_SIZE);

 }

 if (huart->Instance == USART2)

 {

  //pdoa

  RingByteBuffer_pushBuffer(&ring_rx_DMA_buf, usart2_rx_DMA_buffer, received);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_DMA_buffer, RXUART2_BUFFER);

 }

 if (huart->Instance == USART3)

 {

  //twr
  RingByteBuffer_pushBuffer(&ring3_rx_DMA_buf, usart3_rx_DMA_buffer, received);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_DMA_buffer, RXUART3_BUFFER);

 }

}


```

环形缓冲区实现：

```c
void RingByteBuffer_pushBuffer(RingByteBuffer *self, const void *data,
                               uint16_t len)
{
	const uint8_t *byte;
	uint16_t i;

	InvASSERT(self);
	InvASSERT(data);

	byte = (const uint8_t *)data;

	for (i = 0; i < len; ++i) {
		RingByteBuffer_pushByte(self, byte[i]);
	}
}
```

对于环形缓冲区的消费：

```
//在StartUsartTask任务中，对环形缓冲区数据进行解析
//UsartTask的任务栈为1024*4字节
```



---

## 🔍 二、问题分析（初步排查）

| 排查项                                         | 已确认 | 备注/异常记录            | 截图证据 |
| ---------------------------------------------- | :----: | ------------------------ | -------- |
| 排除硬件问题                                   | 已确认 | 更换主控板，依旧存在错误 |          |
| 单片机初始化前串口设备就在发送可能会有问题     | 有问题 |                          |          |
| 数据量过大导致在中断里存入环形缓冲区时耗时过长 |        |                          |          |
| 初始化时有问题                                 |        |                          |          |
| Harddefault                                    | 已确认 | 无硬件错误               |          |
| 缓冲区大小不足                                 |        |                          |          |
| DMA 传输未及时完成                             |        |                          |          |
|                                                |        |                          |          |
|                                                |        |                          |          |
|                                                |        |                          |          |
|                                                |        |                          |          |
|                                                |        |                          |          |

## 🧪 三、实验设计

| 项目1    | 怀疑单片机初始化串口前，串口设备就在发送数据，导致移位寄存器无法搬运到DR |
| -------- | ------------------------------------------------------------ |
| 技术方案 | 串口助手模拟串口设备，以100ms为周期，在上电前分别向串口1发送数据量为580,68,113的信息 |
| 实验结果 | 三次均会进入HAL_UART_ErrorCallback，触发ORE                  |
| 结论     |                                                              |

```c
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
}

```



## 

| 项目2    | 数据量过大导致在中断里存入环形缓冲区时耗时过长               |
| -------- | ------------------------------------------------------------ |
| 技术方案 | 串口助手模拟串口设备，以100ms为周期，向串口1发送数据量为580的信息 |
| 实验结果 | 不会触发ORE                                                  |
| 结论     |                                                              |

---

## 

## 

---
