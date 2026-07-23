#ifndef __APP_OTASET_H
#define __APP_OTASET_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


#include "app_otaset.h"
#include <stddef.h>
#include "main.h"


#include "ab_confirm.h"
#include "app_update_request.h"

/* 此工程运行在 Slot A，串口命令的目标为 Slot B。 */
#define APP_UPDATE_REQUEST_ADDR          0x2001FFF0UL
#define APP_UPDATE_REQUEST_MAGIC         0x55504454UL
#define APP_REQUEST_ACTION_IAP           0x49415031UL
#define APP_REQUEST_ACTION_SWITCH_OR_IAP 0x53574950UL
#define APP_SLOT_B                       1UL


typedef enum
{
    APP_OTA_COMMAND_NONE = 0U,
    APP_OTA_COMMAND_SWITCH_OR_IAP,
    APP_OTA_COMMAND_FORCE_IAP
} app_ota_command_t;


extern volatile uint8_t app_ota_command ;


bool App_IsUsart1OtaFrame(const uint8_t *data, uint16_t length);
 void App_RequestBootloader(uint32_t action, uint32_t target_slot);
 void App_ProcessUsart1OtaCommand(void);


#endif /* __APP_OTASET_H */
