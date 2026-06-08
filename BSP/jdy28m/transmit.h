#ifndef __TRANSMIT_H__
#define __TRANSMIT_H__

#include "stm32f4xx_hal.h"
#include "mesh_cmd.h"
#include <stdbool.h>
extern UART_HandleTypeDef huart1;
struct  mesh_datasend_pkt{
    uint8_t  header[5];
    uint16_t to_maddr;
    uint8_t  header_2;
    uint8_t  key;   //user
    uint8_t  L;     //user
    uint8_t  R;     //user
    uint16_t end;
} __attribute__((packed, aligned(1))) ;
typedef struct  mesh_datasend_pkt  mesh_datasend_pkt;


typedef struct  Transmit{
    bool  mesh_send_valid;
    mesh_datasend_pkt mesh_send_pkt; 


} Transmit;

//typedef struct pdoa_heartbeat_packet{
//    uint8_t  distance;   
//    uint8_t  angle_int;     
//    uint8_t  angle_dec;     
//} pdoa_heartbeat_packet;

//typedef struct state_heartbeat_packet{
//    uint8_t  battery;       
//} state_heartbeat_packet;

//typedef enum {
//    HEARTBEAT_KEY_PDOA,
//    HEARTBEAT_KEY_STATE,
//} Heartbeat_Key;

//typedef struct Heartbeat_packet{

//    Heartbeat_Key key;
//		bool  mesh_send_valid;
//    uint8_t  header[5];
//    uint16_t to_maddr;
//		uint8_t  header_2;
//    pdoa_heartbeat_packet   pdoa_data;
//    state_heartbeat_packet  state_data;
//    uint16_t end;
//} Heartbeat_packet;



extern Transmit myTransmit;
//extern Heartbeat_packet myheartbeat;

void  TransmitData_Updata(Transmit *self, uint16_t to_maddr, uint8_t key);
void  TransmitData_LOOP(Transmit *self);
//void  TransmitHeartData_LOOP(Heartbeat_packet *self);

#endif

