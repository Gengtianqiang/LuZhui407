#include "buzzer.h"
#include "timer/tim.h"
#include <string.h>
#include "freertos.h"

struct buzzer
{
    // 工作模式
    enum{
        Off = 0,
        Once,
        Circle,
        Duration
    } mode;
    // 循环beep
    bool On_Off;
    uint32_t beep_time_ms;
    uint32_t Off_time_ms;
    uint64_t statistics_ms;// 响了多久了
    uint64_t statistics_target_ms; //设定值
} buzzer;


void Buzzer_Start_Once(uint32_t beep_time_ms)
{
    buzzer.mode = Once;
    buzzer.On_Off = true;
    buzzer.beep_time_ms = beep_time_ms;
    buzzer.Off_time_ms  = 0u;
    buzzer.statistics_target_ms = 0u;
}

void Buzzer_Start_Circle(uint32_t beep_time_ms, uint32_t Off_time_ms)
{
    buzzer.mode = Circle;
    buzzer.On_Off = true;
    buzzer.beep_time_ms = beep_time_ms;
    buzzer.Off_time_ms  = Off_time_ms;
    buzzer.statistics_target_ms = 0u;
}

void Buzzer_Start_Duration(uint32_t beep_time_ms, uint32_t Off_time_ms, uint64_t total_ms)
{
    buzzer.mode = Duration;
    buzzer.On_Off = true;
    buzzer.beep_time_ms = beep_time_ms;
    buzzer.Off_time_ms  = Off_time_ms;
    buzzer.statistics_target_ms = total_ms;
}

void Buzzer_Stop(void)
{
    buzzer.On_Off = false;
}


void Buzzer_LOOP(void)
{
    static uint32_t last_time  = 0u;
    static bool     isWorking  = false;
    static bool     isBeepping = false;

    if(buzzer.On_Off){

        // 刚开始
        if(isWorking == false){
            isWorking = true;
            last_time = xTaskGetTickCount();
            // 统计时间
            buzzer.statistics_ms = 0u;
            isBeepping = true;
            HAL_TIM_Base_Start_IT(&htim6);
        }
        // beep 响
        if(isBeepping == true){
            // 已经响了一段时间了
            if(xTaskGetTickCount() - last_time >= buzzer.beep_time_ms){
                HAL_TIM_Base_Stop_IT(&htim6);
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
                last_time = xTaskGetTickCount();
                isBeepping = false;
                // 如果只是单次
                if(buzzer.mode == Once){
                    buzzer.On_Off = false;
                }
                // 统计时间
                buzzer.statistics_ms += buzzer.beep_time_ms;
            }
        }
        // beep 不响
        else{
            // 已经一段时间没响了，接下来开始响
            if(xTaskGetTickCount() - last_time >= buzzer.Off_time_ms){
                HAL_TIM_Base_Start_IT(&htim6);
                last_time = xTaskGetTickCount();
                isBeepping = true;
                // 统计时间
                buzzer.statistics_ms += buzzer.Off_time_ms;
            }
        }
        // Duration 模式
        if(buzzer.mode == Duration){
            // 已经持续一段时间了
            if(buzzer.statistics_ms >= buzzer.statistics_target_ms){
                buzzer.On_Off = false;
            } 
        }

	}
    else{
        HAL_TIM_Base_Stop_IT(&htim6);
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
        // 清除标记位
        last_time  = 0u;
        isWorking = false;
        isBeepping = false;
        // 清除结构体
        memset(&buzzer, 0, sizeof(struct buzzer));
    }
}

uint64_t Get_Buzzer_Statistics(void){
    return buzzer.statistics_ms;
}

