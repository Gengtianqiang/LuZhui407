#ifndef UART_CALLBACK_H
#define UART_CALLBACK_H


#include "forward.h"
#include "app_otaset.h"

extern uint8_t rx_flag;

typedef struct Forward
{
    bool isForward;
} Forward;
extern Forward myForward;


void MY_UART_Init(void);
void App_ProcessUsart1OtaCommand(void);

#endif /* UART_CALLBACK_H */
