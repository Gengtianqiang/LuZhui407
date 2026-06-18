#ifndef __NRF24L01_APP_H
#define __NRF24L01_APP_H


#include "NRF24L01_Define.h"
#include "NRF24L01.h"


// #ifdef AHAND_CAR
// uint8_t NRF24L01_TxAddress[5] = {0x00, 0x00, 0x00, 0xAA, 0xA2};

// #endif

// #ifdef MIDDLE_CAR_FIRST
// #define MESH_MADDR "AT+MADDR0002\r\n"
// #endif

// #ifdef MIDDLE_CAR
// #define MESH_MADDR "AT+MADDR0003\r\n"
// #endif

// #ifdef BEHIND_CAR
// #define MESH_MADDR "AT+MADDR0004\r\n"
// #endif

uint8_t NRF24L01_TASK(void);





#endif /* __NRF24L01_APP_H */

