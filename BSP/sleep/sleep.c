#include "sleep/sleep.h"



/**
 * @brief 主要用于判断 本次MCU启动是否是唤醒事件引起的。
 * @return 
 */
bool Get_isWeakup(void)
{
    // PWR_FLAG_WU 主要用于判断 是否有唤醒事件。
    // PWR_FLAG_SB 主要用于判断 上一次 MCU 是否进入 Standby.
    if(__HAL_PWR_GET_FLAG(PWR_FLAG_WU) != RESET)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Set_Standby(void)
{    
    // 延时一段时间
    HAL_Delay(500U);

    // 复位所有 GPIO 端口
    __HAL_RCC_GPIOA_FORCE_RESET();
    __HAL_RCC_GPIOB_FORCE_RESET();
    __HAL_RCC_GPIOC_FORCE_RESET();
    __HAL_RCC_GPIOD_FORCE_RESET();
    __HAL_RCC_GPIOE_FORCE_RESET();
    __HAL_RCC_GPIOF_FORCE_RESET();
    __HAL_RCC_GPIOG_FORCE_RESET();
    __HAL_RCC_GPIOH_FORCE_RESET();
    __HAL_RCC_GPIOI_FORCE_RESET();

    __HAL_RCC_GPIOA_RELEASE_RESET();
    __HAL_RCC_GPIOB_RELEASE_RESET();
    __HAL_RCC_GPIOC_RELEASE_RESET();
    __HAL_RCC_GPIOD_RELEASE_RESET();
    __HAL_RCC_GPIOE_RELEASE_RESET();
    __HAL_RCC_GPIOF_RELEASE_RESET();
    __HAL_RCC_GPIOG_RELEASE_RESET();
    __HAL_RCC_GPIOH_RELEASE_RESET();
    __HAL_RCC_GPIOI_RELEASE_RESET();
    // 使能 PWR 时钟
    __HAL_RCC_PWR_CLK_ENABLE();
    // 后备区域访问使能
    HAL_PWR_EnableBkUpAccess();
    // 使能唤醒引脚
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); // PWR_WAKEUP_PIN1 其实是 PA0
    // 清除 Wakeup 标志
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    // 进入 Standby 模式
    HAL_PWR_EnterSTANDBYMode();
}

