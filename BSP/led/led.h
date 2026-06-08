#ifndef __LED_H_
#define __LED_H_

#include "main.h"


typedef enum {
    Off=0,
    On,
    blink,
    pwm
} led_st_t;

typedef struct led{
    led_st_t     ST;
    struct {
        uint32_t     MaxPeriod;
        uint32_t     last_ms;
    } blink;
    struct {
        uint32_t     On;
        uint32_t     OnPeriod;
        uint32_t     OffPeriod;
        uint32_t     last_ms;
    } pwm;
    struct {
        uint16_t      GPIO_PIN_x;
        GPIO_TypeDef* GPIOx;
    } dev;
}led_t;


extern led_t    led_R, led_G, led_B;   // led 实例

/**
 * @brief 设置 LED 的状态
 * @param led_inst 指向 led_t 结构体的指针，代表要设置状态的 LED 实例
 * @param ST 要设置的 LED 状态，类型为 led_st_t
 * @param Parameter 如果状态为闪烁，指定闪烁的周期（毫秒）。
 *                  如果状态为 PWM，指定 PWM 的周期和占空比。
 * @details 该函数用于设置 LED 的状态，如开启、关闭或闪烁。
 *          如果状态设置为闪烁，会同时设置闪烁的周期。
 *          如果状态设置为 PWM，会同时设置 PWM 的周期和占空比。
 */
void  Set_LED_State(led_t* led_inst, led_st_t ST, uint32_t Parameter);

void  LED_Init(void);
void  LED_Loop(led_t* led_inst);

#endif
