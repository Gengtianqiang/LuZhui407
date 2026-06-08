#include "battery/battery.h"
#include "led/led.h"

extern ADC_HandleTypeDef hadc1;

const float Revise = 1.0f;

float Volt = 0.0f;

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
函数功能：�?取电池电�?
入口参数：无
返回  值：电池电压，单位v
**************************************************************************/
float Get_battery_volt(void)
{
    uint32_t Volt_tmp;

    /*
     * ADC1 channel 8 is used to read the battery voltage
     */
    HAL_ADC_Start(&hadc1);                   // Start ADC conversion
    HAL_ADC_PollForConversion(&hadc1, 100U); // Poll for conversion completion
    Volt_tmp = HAL_ADC_GetValue(&hadc1);     // Get ADC value
    HAL_ADC_Stop(&hadc1);                    // Stop ADC conversion

    // The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
    // 电阻分压，具体根�?��理图简单分析可以得�?
    Volt_tmp = Volt_tmp & 0x0FFF; // Mask the lower 12 bits
    Volt = (float)Volt_tmp * 3.24f * 11 * Revise / 4096;
    return Volt;
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
函数功能：采集�?�?DC值求平均值函�?, 内有延时不能用于while�?
入口参数：ADC通道和采集�?�?
�? �? 值：AD�?��结果
**************************************************************************/
float Get_battery_volt_Average(uint8_t times)
{
    double temp_val; // 需要使用double类型，避免溢�?
    uint8_t t;
    if (times > 200) // 限制采集次数，避免溢�?
    {
        times = 200;
    }
    for (t = 0; t < times; t++)
    {
        temp_val += Get_battery_volt();
        // Delay 5 milliseconds.
        HAL_Delay(5U);
    }
    return Volt = (float)(temp_val / times);
}

void Set_battery_led(void)
{
    if (Volt >= 22.0f)
    {
        Set_LED_State(&led_R, Off, 500);
        Set_LED_State(&led_G, blink, 500);
        Set_LED_State(&led_B, Off, 500);
    }
    else if (Volt >= 20.0f)
    {
        Set_LED_State(&led_R, Off, 500);
        Set_LED_State(&led_G, Off, 500);
        Set_LED_State(&led_B, blink, 500);
    }
    else
    {
        Set_LED_State(&led_R, blink, 500);
        Set_LED_State(&led_G, Off, 500);
        Set_LED_State(&led_B, Off, 500);
    }
}
