#include "app_update_request.h"

#include "main.h"

#define APP_UPDATE_REQUEST_ADDR  0x2001FFF0UL
#define APP_UPDATE_REQUEST_MAGIC 0x55504454UL
#define APP_REQUEST_ACTION_IAP   0x49415031UL
#define APP_REQUEST_ACTION_BOOT  0x424F4F54UL
#define APP_REQUEST_ACTION_SWITCH_OR_IAP 0x53574950UL
#define APP_SLOT_A               0U
#define APP_SLOT_B               1U


__weak bool App_IsUpdateRequested(void)
{
    return false;
}

static void App_RequestBootloader(uint32_t action, uint32_t target_slot)
{
    volatile uint32_t *request = (volatile uint32_t *)APP_UPDATE_REQUEST_ADDR;

    __disable_irq();
    /* Match Boot_TakeApplicationRequest(): write action/slot first, magic last as commit. */
    request[0] = 0U;
    request[1] = 0U;
    request[2] = action;
    request[3] = target_slot;
    __DSB();
    request[1] = ~APP_UPDATE_REQUEST_MAGIC;
    request[0] = APP_UPDATE_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();

    while (1)
    {
    }
}

void App_RequestBootloaderUpdate(void)
{
    App_RequestBootloaderUpdateSlotB();
}

void App_RequestBootloaderUpdateSlotB(void)
{
    App_RequestBootloader(APP_REQUEST_ACTION_IAP, APP_SLOT_B);
}

void App_RequestBootloaderUpdateSlotA(void)
{
    App_RequestBootloader(APP_REQUEST_ACTION_IAP, APP_SLOT_A);
}

void App_RequestSwitchOrUpdateSlotB(void)
{
    App_RequestBootloader(APP_REQUEST_ACTION_SWITCH_OR_IAP, APP_SLOT_B);
}

void App_RequestSwitchOrUpdateSlotA(void)
{
    App_RequestBootloader(APP_REQUEST_ACTION_SWITCH_OR_IAP, APP_SLOT_A);
}

void App_RequestBootSlotA(void)
{
    App_RequestBootloader(APP_REQUEST_ACTION_BOOT, APP_SLOT_A);
}
