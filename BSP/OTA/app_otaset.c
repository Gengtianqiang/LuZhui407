#include "app_otaset.h"



/* DMA 回调和 USART 任务之间共享；8 位读写在 Cortex-M 上是原子的。 */
volatile uint8_t app_ota_command = APP_OTA_COMMAND_NONE;

/*
 * 允许串口助手发送 U、U\r、U\n 或 U\r\n。
 * 只接受首字节为命令、其余字节仅为行结束符的短帧，业务报文不会被误判。
 */
bool App_IsUsart1OtaFrame(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == NULL) || (length == 0U) || (length > 3U))
    {
        return false;
    }

    for (index = 1U; index < length; index++)
    {
        if ((data[index] != '\r') && (data[index] != '\n'))
        {
            return false;
        }
    }

    return true;
}

 void App_RequestBootloader(uint32_t action, uint32_t target_slot)
{
    volatile uint32_t *request = (volatile uint32_t *)APP_UPDATE_REQUEST_ADDR;

    __disable_irq();

    /*
     * 与 Boot_TakeApplicationRequest() 的协议一致：
     * 先写入动作和目标分区，最后才写 magic，防止复位发生在半写入状态。
     */
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

void App_ProcessUsart1OtaCommand(void)
{
    app_ota_command_t command = (app_ota_command_t)app_ota_command;

    if (command == APP_OTA_COMMAND_NONE)
    {
        return;
    }

    /* 先清除标志，避免复位请求被其他任务再次处理。 */
    app_ota_command = APP_OTA_COMMAND_NONE;

    if (command == APP_OTA_COMMAND_SWITCH_OR_IAP)
    {
        App_RequestBootloader(APP_REQUEST_ACTION_SWITCH_OR_IAP, APP_SLOT_B);
    }

    App_RequestBootloader(APP_REQUEST_ACTION_IAP, APP_SLOT_B);
}