#include "Parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>  // 包含FILE类型的定义
#include "usart.h"
#include "jdy_driver.h"


extern uint8_t ring_buffer_data_Parser[RING_BUFFER_SIZE];  // UART4  → Parser 缓冲区
extern RingByteBuffer ringBuffer_Parser;
extern RingByteBuffer ring_rx_DMA_buf;
extern RingByteBuffer ring3_rx_DMA_buf;
Parser myParser = {0};
BracketContent bracket_data;

uint32_t test_size;
ProtocolData proto_data = {0};
ProtocolData retuen_proto_data = {0};

void ParserData_LOOP(Parser *self)
{
    mesh_datarecv_pkt pkt = {0};
    uint8_t byte;
    uint8_t byte1;

    /* 1. 检查环形缓冲区是否有数据 */
    if (RingByteBuffer_isEmpty(&ringBuffer_Parser)) {
        return;
    }

    /* 2. 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(&ringBuffer_Parser);

    /* 3. 解析帧头 */
    HOP:
    if(byte == 0xF1) {
        /* 检查环形缓冲区长度 */
        if( RingByteBuffer_size(&ringBuffer_Parser) < (sizeof(mesh_datarecv_pkt)-1) ) {
            return;
        }
        /* 取出 1 字节数据 */
        byte = RingByteBuffer_popByte(&ringBuffer_Parser);
        if (byte == 0xDD){
            pkt.header = 0xF1DD;
            /* 取出 1 字节数据 */
            byte = RingByteBuffer_popByte(&ringBuffer_Parser);
            pkt.netID = byte;
        } else {
            goto HOP;
        }
        
    }
    else if(byte == '+'){
        /* 检查环形缓冲区长度 */
        if( RingByteBuffer_size(&ringBuffer_Parser) < (5-1) ) {  // +OK\r\n  最少是 5 个字节
            return;}
        
        uint16_t i = 0u;
        do{
            self->AT.buf [i++] = byte;
            byte = RingByteBuffer_popByte(&ringBuffer_Parser);
        } while( byte != '\n' );
        
        self->AT.buf [i++] = byte;
        self->AT.len = i;
        self->AT.isAvailable = true;
        return;
    }
    else{
        return;
    }

    /* 4. 解析 from_maddr */
    /* 取出 2 字节数据 */
    byte  = RingByteBuffer_popByte(&ringBuffer_Parser);
    byte1 = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.from_maddr = (byte << 8) | byte1;

    /* 5. 解析 to_maddr */
    /* 取出 2 字节数据 */
    byte  = RingByteBuffer_popByte(&ringBuffer_Parser);
    byte1 = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.to_maddr = (byte << 8) | byte1;

    /* 6. 解析 key */
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.key = byte;

    /* 7. 解析 L */
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.L = byte;

    /* 7. 解析 R */
    /* 取出 1 字节数据 */
    byte = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.R = byte;

    /* 8. 解析 end */
    /* 取出 2 字节数据 */
    byte = RingByteBuffer_popByte(&ringBuffer_Parser);
    byte1 = RingByteBuffer_popByte(&ringBuffer_Parser);
    pkt.end = (byte << 8) | byte1;
    
    /* 7. 解析完成 */
    self->mesh.pkt = pkt;
    self->mesh.isAvailable = true;
    return;

}



bool ParseTwrProtocol(RingByteBuffer *ring, BracketContent *result) {
	
		if (ring == NULL || result == NULL || RingByteBuffer_isEmpty(ring)) {
        return false;
    }
	taskENTER_CRITICAL();
    uint16_t ring_len = RingByteBuffer_size(ring);
	taskEXIT_CRITICAL();
    if (ring_len < 2) return false;  // 至少需要两个字节才可能包含[]

    uint8_t temp_buf[30];  // 临时存储数据
    uint16_t temp_idx = 0;
    bool in_bracket = false;  // 是否进入[]区域
    bool found_end = false;   // 是否找到结束符]

    // 清空结果
    memset(result->content, 0, sizeof(result->content));
    result->len = 0;

    // 逐个读取缓冲区数据
		taskENTER_CRITICAL();
    while (ring_len > 0) {
			 
        uint8_t byte = RingByteBuffer_popByte(ring);
			 
        ring_len--;

        // 检测起始符[
        if (!in_bracket) {
            if (byte == '[') {
                in_bracket = true;

            }
            continue;  // 未进入括号前不处理其他字符
        }

        // 已进入[，处理直到遇到]
        if (byte == ']') {
            found_end = true;
            break;  // 结束提取
        }
				
				
				// 超过20字节未找到']'
				if(temp_idx>20) {
					
					return false;
					
				}
        // 存储括号内的字符（防止缓冲区溢出）
        if (temp_idx < sizeof(temp_buf) - 1) {
            temp_buf[temp_idx++] = byte;
        }
    }
		taskEXIT_CRITICAL();
		
    // 提取成功，复制结果
    if (found_end && temp_idx > 0) {
        result->len = temp_idx;
        memcpy(result->content, temp_buf, temp_idx);
        result->content[temp_idx] = '\0';  // 添加字符串结束符
        return true;
    }
    return false;
}

uint8_t proser_test = 0;
bool Protocol_Parse(RingByteBuffer *ring, ProtocolData *data) {
    data->PdoaisAvailable = true;
    uint16_t ring_len = RingByteBuffer_size(ring);
	char *saveptr = NULL; // 上下文指针
    if (ring_len < 6)  {
        data->PdoaisAvailable = false;
        return false;  // 帧头"MPxxxx"至少6字节（"MP"+4位长度）
        
    }
    data->PdoaisAvailable = true;
    // 步骤1：查找帧头"MP"并解析长度字段
    uint8_t header_buf[6] = {0};  // 存储"MPxxxx"
    bool header_found = false;

    // 循环查找"MP"开头（跳过无效数据）
    while (ring_len >= 2) {
        uint8_t first = RingByteBuffer_popByte(ring);
        ring_len--;

        if (first == 'M' && ring_len >= 1) {  // 检查下一个字节是否为'P'
            uint8_t second = RingByteBuffer_popByte(ring);
            ring_len--;
            if (second == 'P') {
                // 找到"MP"，读取后续4位长度字段
                if (ring_len < 4) {  // 长度字段不完整，推回"MP"
					data->PdoaisAvailable = false;
                    return false;
                }
                // 读取4位长度字段（xxxx）
                RingByteBuffer_popBuffer(ring, header_buf + 2, 4);
                ring_len -= 4;
                header_buf[0] = 'M';
                header_buf[1] = 'P';
                header_found = true;
                break;
            }
        }
        // 不是'M'，直接丢弃（无需处理）
    }

    if (!header_found)  {
			data->PdoaisAvailable = false;
			return false;  // 未找到有效帧头
		}

    // 步骤2：解析长度字段（xxxx -> 整数，数据段字节数）
    char len_str[5] = {0};
    memcpy(len_str, header_buf + 2, 4);  // 取"xxxx"
    int data_len = atoi(len_str)+1;        // 转换为数据长度（如"0034"->34）

    // 校验数据长度合理性（防止缓冲区溢出）
    if (data_len <= 0 || data_len > RXUART2_BUFFER) {
		data->PdoaisAvailable = false;
        return false;  // 无效长度，丢弃当前帧
    }

    // 步骤3：读取数据段（长度为data_len）并检查帧尾'\n'
    if (ring_len < data_len + 1) {  // 数据段+帧尾'\n'不足
        data->PdoaisAvailable = false;
        return false;
    }

    uint8_t data_buf[RXUART2_BUFFER + 1] = {0};  // 存储数据段
    RingByteBuffer_popBuffer(ring, data_buf, data_len);  // 读取数据段
		data_buf[data_len] = '\0';
    ring_len -= data_len;

    uint8_t frame_end = RingByteBuffer_popByte(ring);  // 读取帧尾
		proser_test = frame_end;
		
    ring_len--;
    if (frame_end != '\n') {  // 帧尾不正确，丢弃当前帧
		data->PdoaisAvailable = false;
        return false;
    }
    // 步骤4：按逗号分割数据段字段
    char *tokens[16] = {0};  // 最多16个字段（含可选字段）
    int token_count = 0;

    char *data_str = (char*)data_buf;
    char *token = strtok_r(data_str, "," ,&saveptr);
    while (token != NULL && token_count < 16) {
        tokens[token_count++] = token;
        token = strtok_r(NULL, "," ,&saveptr);
    }

    // 步骤5：校验必填字段数量（至少11个：tag_id到key）
    if (token_count < 11) {
		data->PdoaisAvailable = false;
        return false;
    }

    // 步骤6：解析必填字段
    data->tag_id             = atoi(tokens[0]);
    data->x_cm               = atoi(tokens[1]);
    data->y_cm               = atoi(tokens[2]);
    data->distance_cm        = atoi(tokens[3]);
    data->range_number       = atoi(tokens[4]);
    data->pdoa_deg           = atof(tokens[5]);
    data->aoa_deg            = atof(tokens[6]);
    data->distance_offset_cm = atoi(tokens[7]);
    data->pdoa_offset_deg    = atoi(tokens[8]);
    data->distance_cm_A1     = atoi(tokens[9]);
    data->key                = atoi(tokens[10]);

    // 步骤7：解析可选字段aoa_deg_A1（若存在）
    data->has_aoa_A1 = (token_count >= 12);
    if (data->has_aoa_A1) {
        data->aoa_deg_A1 = atof(tokens[11]);
    } else {
        data->aoa_deg_A1 = 0.0f;  // 默认值
    }
    return true;  // 解析成功
}

//解析坐标
bool ParseBracketContentToFloats(BracketContent *bc_ptr) {
    // 1. 入参合法性校验
    if (bc_ptr == NULL) {
        return false;
    }
    // 检查内容长度：至少需 "0,0,0"（5字节），最长不超过结构体content长度
    if (bc_ptr->len == 0 || bc_ptr->len >= sizeof(bc_ptr->content)) {
        return false;
    }

    // 2. 构造以'\0'结尾的字符串（避免乱码，STM32字符串操作必须以'\0'结束）
    char temp_str[sizeof(bc_ptr->content) + 1];  // 临时缓冲区（+1留结束符）
    memset(temp_str, 0, sizeof(temp_str));
    memcpy(temp_str, bc_ptr->content, bc_ptr->len);  // 拷贝内容
    temp_str[bc_ptr->len] = '\0';  // 强制添加字符串结束符

    // 3. 查找逗号分隔符，校验分隔符数量（必须是2个逗号）
    char *comma1 = strchr(temp_str, ',');
    if (comma1 == NULL) return false;  // 无第一个逗号
    char *comma2 = strchr(comma1 + 1, ',');
    if (comma2 == NULL) return false;  // 无第二个逗号
    if (strchr(comma2 + 1, ',') != NULL) return false;  // 超过2个逗号

    // 4. 拆分三个子字符串（替换逗号为'\0'，分割出独立数值字符串）
    *comma1 = '\0';  // 第一个数值结束
    *comma2 = '\0';  // 第二个数值结束
    char *str1 = temp_str;        // 第一个数值字符串（如"-2.45"）
    char *str2 = comma1 + 1;      // 第二个数值字符串（如"5.44"）
    char *str3 = comma2 + 1;      // 第三个数值字符串（如"1.43"）

    // 5. 校验子字符串非空（避免",5.44,1.43"或"-2.45,,1.43"等无效格式）
    if (strlen(str1) == 0 || strlen(str2) == 0 || strlen(str3) == 0) {
        return false;
    }

    // 6. 字符串转float（STM32需确保启用浮点库，或替换为自定义浮点转换）
    bc_ptr->x = atof(str1);
    bc_ptr->y = atof(str2);
    bc_ptr->z = atof(str3);

    // 7. 最终校验（可选：防止转换出NaN/INF，适配严格场景）
    if (bc_ptr->x != bc_ptr->x || bc_ptr->y != bc_ptr->y || bc_ptr->z != bc_ptr->z) {  // 判断是否为NaN
        return false;
    }

    return true;
}


bool MeshData_isReady(Parser *self){
    return self->mesh.isAvailable;
}

bool ATData_isReady(Parser *self){
    return self->AT.isAvailable;
}


mesh_datarecv_pkt* Get_MeshData(Parser *self){
    self->mesh.isAvailable = false;
    return &self->mesh.pkt;
}

uint8_t* Get_ATData(Parser *self){
    self->AT.isAvailable = false;
    return self->AT.buf;
}

uint16_t Get_ATData_len(Parser *self){
    return self->AT.len;
}

