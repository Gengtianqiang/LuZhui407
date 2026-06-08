#ifndef RESPONSE_H
#define RESPONSE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#include "mesh_cmd.h"
#include "parser.h"
#include "transmit.h"

typedef struct BleData{
    uint8_t  key;
    uint8_t  L;
    uint8_t  R;
}BleData;

typedef struct response
{
    // 自身状态
    struct{
        bool     isConnected;   // 是否已经配对手柄
        uint16_t Master_maddr;  // 手柄的addr  (Master) (Hex型)
        uint16_t Slave_maddr;   // 小车自身的maddr (Slave) (Hex型)
        uint8_t  netID;
    } link;
    // 蓝牙dev数据(字符型)
    struct{
        uint8_t  MAC[12];       // 12位字符串，字符范围0~F
        uint8_t  STAT;          // 状态。0未连接；1已连接；2已组网；3未组网
        uint8_t  MADDR[4];      // MESH 组网短地址,范围(0000-FFFF) //默认：MAC 地址后 2 字节
        uint8_t  BAUD;
        uint8_t  ROLE;
        uint8_t  NETID[10];
        uint8_t  ENLOG;
    } dev;
    // 获取到的新的手柄数据 //
    struct{
        bool  isUpdata;
        BleData  Data;
    } newData;
}
Response;


// 消息类型
typedef enum {
    MSG_TYPE_START = 0x01,    // 启动信号（无数据体）
    MSG_TYPE_FEEDBACK = 0x02, // 启动反馈（无数据体）
    MSG_TYPE_HEARTBEAT = 0x03,// 心跳包（带心跳数据）
    MSG_TYPE_PODA = 0x04      // PODA数据（带角度+距离数据）
} mesh_msg_type_t;

typedef struct {
    uint8_t battery;  // 电量（0-100）
    bool is_fallen;   // 是否倒下（true/false）
} heartbeat_data_t;

// 统一的蓝牙APP消息结构体
typedef struct {

    bool is_4g_signal;    // 是否有4G信号
    uint8_t src_node;         // 源节点ID（1-5，标识哪个节点发的）
    uint8_t dst_node;         // 目标节点ID（1-5，0表示广播）
    mesh_msg_type_t msg_type; // 消息类型（决定union里用哪个字段）
    union {                   // 联合体：同一时刻仅存储一种消息数据，节省内存
        heartbeat_data_t heartbeat; // 心跳包数据（msg_type=MSG_TYPE_HEARTBEAT时用）
       uint8_t pdoa_packed_3bytes[3]; // pdoa打包后的3字节数据
        // 启动/反馈信号无数据体，无需额外字段
    } msg_body;

    uint32_t timecounter; 
} mesh_msg_t;

extern Response myResponse;

void ResponseMesh_LOOP( Response *myResponse, Parser *myParser, Transmit *myTransmit );
void ResponseAT_LOOP( Response *myResponse, Parser *myParser );

bool Response_isNewData(Response *myResponse);
BleData Response_GetNewData(Response *myResponse);

#endif
