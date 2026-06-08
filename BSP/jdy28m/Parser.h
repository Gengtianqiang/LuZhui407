#ifndef __PARSER_H
#define __PARSER_H

#include "forward.h"
#include "util/RingByteBuffer.h"
#include "stm32f4xx_hal.h"

//目标初始角度
#define FOLLOW_ANGLE_INIT -5
//目标跟随距离(cm)
#define FOLLOW_DISTANCE 50
//跟随距离允许最大误差
#define FOLLOW_DISTANCE_ERROR 10

//跟随角度允许最大误差
#define FOLLOW_ANGLE_ERROR 20

typedef struct {
    uint8_t content[30];  // 存储提取的内容
    uint8_t len;       // 内容长度
		float x;
		float y;
		float z;
		bool twr_status;
} BracketContent;
extern BracketContent bracket_data;
// 协议数据结构体（PDOA基站）
typedef struct {
    int tag_id;               // 标签ID（整数）
    int x_cm;                 // X坐标(cm)（整数）
    int y_cm;                 // Y坐标(cm)（整数）
    int distance_cm;          // 到主基站距离(cm)（整数）
		int last_distance_cm; 
    int range_number;         // 测距流水号（0-255）
    float pdoa_deg;           // PDOA角度(°)（浮点）
    float aoa_deg;            // AOA角度(°)（浮点）
    int distance_offset_cm;   // 距离校准值(cm)（整数）
    int pdoa_offset_deg;      // PDOA校准值(°)（整数）
    int distance_cm_A1;       // 到A1基站距离(cm)（整数）
    int key;                  // 按键值（整数）
    float aoa_deg_A1;         // 到A1的AOA角度(°)（可选，浮点）
    bool has_aoa_A1;          // 标记是否存在aoa_deg_A1字段
	bool PdoaisAvailable;
} ProtocolData;
extern ProtocolData proto_data;
extern ProtocolData retuen_proto_data;
struct  mesh_datarecv_pkt{
    uint16_t header;
    uint8_t  netID;
    uint16_t from_maddr;
    uint16_t to_maddr;
    uint8_t  key;   //user
    uint8_t  L;   //user
    uint8_t  R;   //user
    uint16_t end;
}__attribute__((packed, aligned(1)));
typedef struct mesh_datarecv_pkt     mesh_datarecv_pkt;


struct Parser
{
    struct{
        bool                isAvailable;
        mesh_datarecv_pkt   pkt;
    } mesh;
    
    struct{
        bool     isAvailable;
        uint8_t  buf[UART_RX_BUFFER_SIZE];
        uint16_t len;
    } AT;
} ;
typedef struct Parser   Parser;


extern Parser myParser;

void ParserData_LOOP(Parser *self);

bool MeshData_isReady(Parser *self);
mesh_datarecv_pkt* Get_MeshData(Parser *self);
bool     ATData_isReady(Parser *self);
uint8_t* Get_ATData(Parser *self);
uint16_t Get_ATData_len(Parser *self);
bool ParseTwrProtocol(RingByteBuffer *ring, BracketContent *result);
bool Protocol_Parse(RingByteBuffer *ring, ProtocolData *data);
bool ParseBracketContentToFloats(BracketContent *bc_ptr);

#endif // __PARSER_H
