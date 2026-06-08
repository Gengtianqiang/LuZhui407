#include "led/led.h"
#include "stm32f4xx_hal.h"

#pragma diag_suppress 188

led_t
led_R = {0}, 
led_G = {0}, 
led_B = {0};





void  Set_LED_State(led_t* led_inst, led_st_t ST, uint32_t Parameter)
{
    led_inst->ST = ST;
    if(ST == blink){
        led_inst->blink.MaxPeriod = Parameter;
    }
    else if( ST == pwm){
        if(Parameter >= 100){
            led_inst->pwm.OnPeriod = 10;
            led_inst->pwm.OffPeriod = 0;
        }else{
            // Parameter = Parameter * 2;
            led_inst->pwm.OnPeriod = Parameter / 10;
            led_inst->pwm.OffPeriod = 10 - led_inst->pwm.OnPeriod;
        }
    }
    else{
        (void)Parameter;
    }
}


void  LED_Init(void)
{
    led_R.dev.GPIOx = LED_R_PIN_GPIO_Port;
    led_R.dev.GPIO_PIN_x = LED_R_PIN_Pin;

    led_G.dev.GPIOx = LED_G_PIN_GPIO_Port;
    led_G.dev.GPIO_PIN_x = LED_G_PIN_Pin;

    led_B.dev.GPIOx = LED_B_PIN_GPIO_Port;
    led_B.dev.GPIO_PIN_x = LED_B_PIN_Pin;
}

void  LED_Loop(led_t* led_inst)
{
    uint32_t current_ms = HAL_GetTick();

    // led blinking
    if(led_inst->ST == blink){
        if(current_ms - led_inst->blink.last_ms >= led_inst->blink.MaxPeriod){
            led_inst->blink.last_ms = current_ms;
            HAL_GPIO_TogglePin(led_inst->dev.GPIOx, led_inst->dev.GPIO_PIN_x);
        }
    }

    // led pwm
    else if(led_inst->ST == pwm){
        if(led_inst->pwm.On == 0u){
            if(current_ms - led_inst->pwm.last_ms >= led_inst->pwm.OffPeriod){
                HAL_GPIO_WritePin(led_inst->dev.GPIOx, led_inst->dev.GPIO_PIN_x, GPIO_PIN_RESET); 
                led_inst->pwm.last_ms = current_ms;
                led_inst->pwm.On = 1u;
            }
        }else{
            if(current_ms - led_inst->pwm.last_ms >= led_inst->pwm.OnPeriod){
                HAL_GPIO_WritePin(led_inst->dev.GPIOx, led_inst->dev.GPIO_PIN_x, GPIO_PIN_SET);
                led_inst->pwm.last_ms = current_ms;
                led_inst->pwm.On = 0u;
            }
        }
    }

    // led on/off
    else if(led_inst->ST == On){
        HAL_GPIO_WritePin(led_inst->dev.GPIOx, led_inst->dev.GPIO_PIN_x, GPIO_PIN_RESET);  
    }
    else if(led_inst->ST == Off){
        HAL_GPIO_WritePin(led_inst->dev.GPIOx, led_inst->dev.GPIO_PIN_x, GPIO_PIN_SET); 
    }
}


