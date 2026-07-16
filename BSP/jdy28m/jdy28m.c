#include "jdy28m.h"
#include "string.h"

extern UART_HandleTypeDef huart4;

JDY_t   myJDY = {0};


void JDY_SendCmd(const uint8_t* cmd){
    // 获取字符串长度
    uint16_t length = (uint16_t)( 0xFFFF & strlen((const char*)cmd));
    uint32_t lasttime = HAL_GetTick();
    while( huart4.gState != HAL_UART_STATE_READY ){
        if( HAL_GetTick() - lasttime > 1000u ){ break; } }
    HAL_UART_Transmit(&huart4, cmd, length, 1000u);
}


void JDY_Task_Init(JDY_t *self, enum ForwardData_Key on_off)
{
    // Init Pointer
    self->myParser   = &myParser;
    self->myTransmit = &myTransmit;
    self->myResponse = &myResponse;
    self->output     = &myResponse.newData.Data;

    // uart4 <--> usart1   true:forwardData  false:do not forwardData
    UART_DMA_Init(on_off);

    // 初始化JDY28M
	JDY_SendCmd("AT\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+DEFAULT\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+RESET\r\n");
    HAL_Delay(300);
    JDY_SendCmd("AT+BAUD\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+MAC\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+MADDR000A\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+ALED0\r\n");
    HAL_Delay(1);
    JDY_SendCmd("AT+ENLOG0\r\n"); // 关闭日志输出
    HAL_Delay(1);
    JDY_SendCmd("AT+ROLE2\r\n"); // 设置组网类型 (2:mesh)
    HAL_Delay(1);
    JDY_SendCmd(MESH_NETID); // 设置组网ID
    HAL_Delay(1);
    JDY_SendCmd("AT+RESET\r\n");
    // HAL_Delay(250);
}

void JDY_Task_LOOP(JDY_t *self)
{
    // uart4 <--> usart1   forwardData
//    ForwardData_LOOP();

    // 解析BLE数据
    ParserData_LOOP(self->myParser);

    // 解析遥感数据
//    ResponseMesh_LOOP(self->myResponse, self->myParser, self->myTransmit);
//    ResponseAT_LOOP(self->myResponse, self->myParser);
//    
    // 遇到需要发送数据时，轮询发送（mesh_send_valid），无需关心
    TransmitData_LOOP(self->myTransmit);
		// 发送心跳包
		//需要发数据时
		//self.mesh_send_valid = true
		//self.to_maddr = target
		//self.key
		//data
	
}

bool JDY_isDataReady(JDY_t *self){
    return Response_isNewData(self->myResponse);
}

BleData JDY_GetData(JDY_t *self){
    return Response_GetNewData(self->myResponse);
}
