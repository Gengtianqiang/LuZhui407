#ifndef JDY_28M_H
#define JDY_28M_H

#include "stm32f4xx_hal.h"
#include "forward.h"
#include "mesh_cmd.h"
#include "Parser.h"
#include "Transmit.h"
#include "Response.h"

typedef struct {
    // pointer
    Parser   *myParser;
    Transmit *myTransmit;
    Response *myResponse;

    // 摇控
    BleData   *output;

} JDY_t;

extern JDY_t   myJDY;

void JDY_Task_Init(JDY_t *self, enum ForwardData_Key on_off);
void JDY_Task_LOOP(JDY_t *self);


void JDY_SendCmd(const uint8_t* cmd);
bool JDY_isDataReady(JDY_t *self);
BleData JDY_GetData(JDY_t *self);

#endif
