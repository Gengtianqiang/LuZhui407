#include "Vofa+.h"


#define USART1_REC_LEN 256
uint8_t UART1_RX_DATA1[USART1_REC_LEN];
/*Vofa+支持三种模式*/
/* FireWater格式
   数据格式："<any>:ch0,ch1,ch2,...,chN\n" 
   实现方式：可以直接利用Printf重定向串口，打印对应的数据
   例如： Printf("<any>:%f,%f,%f",Kp,Ki,Kd) 
   注意：由于字符串解析消耗更多的运算资源（无论在上位机还是下位机），建议仅在通道数量不多、发送频率不高的时候使用*/

/* JustFloat格式
   数据格式：fdata为小端浮点数组，里面放着需要发送的CH_COUNT个数据
            float fdata[CH_COUNT];
            unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f}; 这个包尾是必要的，发送一组数据需要以这组数据为包尾
   实现方式：利用串口发送浮点型数据，同样可以利用串口接收浮点型数据以适应Vofa+控件发过来的数据（PID实时调控的必要）
   例如：“https://www.vofa.plus/plugin_detail/?name=justfloat#heading-2.1” 例程可以参考官网
   注意：本协议是小端浮点数组形式的字节流协议，纯十六进制浮点传输，节省带宽。此协议非常适合用在通道数量多、发送频率高的时候*/

/* RawData格式
   无数据格式
   实现方式：类似于普通的串口助手调试，不会显示波形（类似正点原子串口助手等等）  */

/* 初始化选择数据模式*/
void Vofa_Init_c(uint8_t Data_Mode , uint32_t Banud)
{
//   uart1_init(Banud); //使用串口1 DMA发送+接收 加快数据传输的速度
   if(Data_Mode == FireWater_Mode)
   {
      printf("FireWater数据格式\r\n");
   }
   if(Data_Mode == JustFloat_Mode)
   {
//      printf("JustFloat数据格式\r\n");
   }
}

/* FireWater格式 可以使用此函数*/
void Vofa_Printf_c(char *format, ...)
{
   char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	//Serial_SendArray(String);
}

/* JustFloat格式接口函数*/
void Vofa_JustFloat_SendData(float send_value)
{
   uint8_t four_byte_buffer[4] = {0};
   Vofa_float_to_Bytes(send_value,four_byte_buffer);
	 Serial_SendArray(four_byte_buffer,4);
}

void Vofa_JustFloat_END(void)
{
   uint8_t data_end[4] = {0x00, 0x00, 0x80, 0x7f};
   Serial_SendArray(data_end,4); //每次都需要发送包尾
}

/* JustFloat格式 处理控件接收数据*/
void Vofa_JustFloat_RecvData(float *Target ,float *Kp ,float *Ki ,float *Kd )
{
   if(Serial_GetRxFlag() == 1)
   {
      if(Vofa_Buffer[0] == 0x40 && Vofa_Buffer[6] == 0xFF)
      {
         switch (Vofa_Buffer[1])
         {
         case 0x01:
            *Target = Vofa_bytes_to_float(&Vofa_Buffer[2]);
						//usart1_send(0x01);
				break;
         case 0x02:
            *Kp = Vofa_bytes_to_float(&Vofa_Buffer[2]);
						/*回传测试*/
						//usart1_send(0x02);
            break;
         case 0x03:
            *Ki = Vofa_bytes_to_float(&Vofa_Buffer[2]);
						//usart1_send(0x03);
            break;
         case 0x04:
            *Kd = Vofa_bytes_to_float(&Vofa_Buffer[2]);
						//usart1_send(0x04);
            break;
         default:
            memset(Vofa_Buffer,0,USART1_REC_LEN);
            break;
         }
				 memset(Vofa_Buffer,0,UART_RX_BUFFER_SIZE);
      }
   }


}

uint8_t Serial_GetRxFlag(void)
{
	if(Vofa_RxFlag == 1)
	{
		Vofa_RxFlag = 0;
		return 1;
	}
	return 0;
}


/*       转换功能函数      */
void Vofa_float_to_Bytes(float value , uint8_t *Byte)
{
   memcpy(Byte, &value, 4);
}

/*要找好数组地址的关系 由于浮点型为4字节一组，固数组地址需没隔4组取一组*/
float Vofa_bytes_to_float(uint8_t *data)
{
    float result;
    memcpy(&result, data, 4);
    return result;
}

/*参数含义
  data : 需转换的数组（即串口接收到的数据）
  output : 输出到的数组（即存放浮点数的数据）
  count : 有多少组浮点数（4个字节为一组）*/
void Vofa_parse_float_array(const uint8_t *data, float *output, int count) 
{
    memcpy(output, data, count * 4);
}


void usart1_send(uint8_t data)
{
	while((USART1->SR&0x40)==0);//循环发送,直到发送完毕
	USART1->DR = data;	
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		usart1_send(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}


