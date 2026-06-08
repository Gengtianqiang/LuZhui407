#include "transmit.h"
#include <stdlib.h>
#include <string.h>

Transmit myTransmit = {
    .mesh_send_valid = false,
    .mesh_send_pkt  = 
    {
        .header   = {0x41, 0x54, 0x2B, 0xAA, 0xA1},
        .to_maddr = 0x0000,
        .header_2 = 0xA0,   // A0 表示Mesh指令模式  F0 表示控制io模式
        .key      = MESH_KEY_NONE,
        .L        = 0x7F,
        .R        = 0x7F,
        .end      = 0x0D0A,
    }
};

//Heartbeat_packet myheartbeat = {
//		.key = HEARTBEAT_KEY_PDOA,
//	  .mesh_send_valid = false,
//    .header   = {0x41, 0x54, 0x2B, 0xAA, 0xA1},
//    .to_maddr = 0x0000,
//        .pdoa_data = {
//					.distance = 0,
//					.angle_int = 0,
//					.angle_dec = 0,
//					
//				},
//				
//				.state_data = {
//					.battery = 0,
//					
//				},
//				.header_2 = 0xA0,   // A0 表示Mesh指令模式  F0 表示控制io模式
//        .end      = 0x0D0A,	
//};


extern UART_HandleTypeDef huart4;


// 小车的发送只关注 to_maddr 和 key
void TransmitData_Updata(Transmit *self, uint16_t to_maddr, uint8_t key){
    self->mesh_send_pkt.to_maddr = to_maddr;
    self->mesh_send_pkt.key = key;
    self->mesh_send_valid = true;
}

//// 发送心跳包
//void heartbeat_packet_Updata(Heartbeat_packet *self, uint16_t to_maddr, uint8_t key){
//    self->to_maddr = to_maddr;
//    self->key = key;
//    self->mesh_send_valid = true;
//}


// 生成 0 ~ max 的数
inline uint32_t get_random_max(uint32_t max) {
    return rand() % (max + 1);  
}

// 生成 min ~ max 的数. 更均匀的 rand() 版本（减少模偏差）
inline int get_random_better(int min, int max) {
    int range = max - min + 1;
    int limit = RAND_MAX - (RAND_MAX % range);
    int rnd;

    do {
        rnd = rand();
    } while (rnd >= limit);

    return min + (rnd % range);
}

void  TransmitData_LOOP(Transmit *self)
{
    static uint8_t      mesh_send_array[sizeof(self->mesh_send_pkt)+4] = {0};
//    static uint32_t     last_time = 0;
//    static uint32_t     Interval = 0;  // 间隔ms
    static bool         isReady = true;
//    static bool         isRAND = false;

    if(self->mesh_send_valid == false){return;}
    if(self->mesh_send_pkt.to_maddr == 0x0000u){return;}

    //还有数据要发.将缓冲区发送出去
    mesh_send_array[0]  = 'A';
    mesh_send_array[1]  = 'T';
    mesh_send_array[2]  = '\r';
    mesh_send_array[3]  = '\n';
    // memcpy(mesh_send_array+4, &self->mesh_send_pkt, sizeof(self->mesh_send_pkt));  // 拷贝数据到数组中 (uint16_t小端，不行)
    mesh_send_array[4]  = self->mesh_send_pkt.header[0];
    mesh_send_array[5]  = self->mesh_send_pkt.header[1];
    mesh_send_array[6]  = self->mesh_send_pkt.header[2];
    mesh_send_array[7]  = self->mesh_send_pkt.header[3];
    mesh_send_array[8]  = self->mesh_send_pkt.header[4];
    mesh_send_array[9]  = (self->mesh_send_pkt.to_maddr & 0xFF00)>>8;
    mesh_send_array[10]  = self->mesh_send_pkt.to_maddr & 0x00FF;
    mesh_send_array[11]  = self->mesh_send_pkt.header_2;
    mesh_send_array[12]  = self->mesh_send_pkt.key;
    mesh_send_array[13]  = self->mesh_send_pkt.L;
    mesh_send_array[14]  = self->mesh_send_pkt.R;
    mesh_send_array[15]  = (self->mesh_send_pkt.end & 0xFF00)>>8;
    mesh_send_array[16]  =  self->mesh_send_pkt.end & 0x00FF;

    HAL_UART_Transmit_DMA(&huart4, mesh_send_array, sizeof(mesh_send_array));
    
    // 清除标记位
    self->mesh_send_valid = false;
    isReady = false;
}

//需要发数据时
//self.mesh_send_valid = true
//self.to_maddr = target
//self.key
//data

//int tr_test;
//void  TransmitHeartData_LOOP(Heartbeat_packet *self)
//{
//    static uint8_t      mesh_send_array[20] = {0};
//    static uint32_t     last_time = 0;
//    static uint32_t     Interval = 0;  // 间隔ms
//    static bool         isReady = true;
//    static bool         isRAND = false;
//		
//    if(self->mesh_send_valid == false){return;}
//    if(self->to_maddr == 0x0000u){return;}
//	  
//    //还有数据要发.将缓冲区发送出去
//    mesh_send_array[0]  = 'A';
//    mesh_send_array[1]  = 'T';
//    mesh_send_array[2]  = '\r';
//    mesh_send_array[3]  = '\n';
//    // memcpy(mesh_send_array+4, &self->mesh_send_pkt, sizeof(self->mesh_send_pkt));  // 拷贝数据到数组中 (uint16_t小端，不行)
//    mesh_send_array[4]  = self->header[0];
//    mesh_send_array[5]  = self->header[1];
//    mesh_send_array[6]  = self->header[2];
//    mesh_send_array[7]  = self->header[3];
//    mesh_send_array[8]  = self->header[4];
//    mesh_send_array[9]  = (self->to_maddr & 0xFF00)>>8;
//    mesh_send_array[10]  = self->to_maddr & 0x00FF;
//    mesh_send_array[11]  = self->header_2;
//		if(self->key==HEARTBEAT_KEY_PDOA) {
//			mesh_send_array[12]  = self->pdoa_data.distance;
//			mesh_send_array[13]  = self->pdoa_data.angle_int;
//			mesh_send_array[14]  = self->pdoa_data.angle_dec;
//		}else {
//			mesh_send_array[12]  = self->state_data.battery;
//		}
//			mesh_send_array[15]  = (self->end & 0xFF00)>>8;
//			mesh_send_array[16]  =  self->end & 0x00FF;
//		

//    HAL_UART_Transmit_DMA(&huart4, mesh_send_array, sizeof(mesh_send_array));
//    
//    // 清除标记位
//    self->mesh_send_valid = false;
//    isReady = false;
//}

