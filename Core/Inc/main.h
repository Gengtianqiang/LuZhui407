/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "cmsis_os.h"
#include "System_Init.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "Vofa.h"
#include "icm20948_app.h"
#include "motor/motor.h"
#include "battery/battery.h"
#include "sleep/sleep.h"
#include "led/led.h"
#include "jdy28m/jdy28m.h"
#include "buzzer/buzzer.h"


#include "control.h"
#include "PID.h"
#include "balance.h"
#include "Encoder.h"
#include "Motor_GPIO.h"
#include "Vofa+.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;



extern float Angle_init;
extern bool Start_init;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_IO_Pin GPIO_PIN_2
#define DEBUG_IO_GPIO_Port GPIOE
#define Enable_PIN_Pin GPIO_PIN_4
#define Enable_PIN_GPIO_Port GPIOE
#define Motor_C0_Pin GPIO_PIN_0
#define Motor_C0_GPIO_Port GPIOC
#define Battery_Ch_Pin GPIO_PIN_0
#define Battery_Ch_GPIO_Port GPIOB
#define Motor_B12_Pin GPIO_PIN_12
#define Motor_B12_GPIO_Port GPIOB
#define Motor_B13_Pin GPIO_PIN_13
#define Motor_B13_GPIO_Port GPIOB
#define Motor_B14_Pin GPIO_PIN_14
#define Motor_B14_GPIO_Port GPIOB
#define Motor_D10_Pin GPIO_PIN_10
#define Motor_D10_GPIO_Port GPIOD
#define Motor_D12_Pin GPIO_PIN_12
#define Motor_D12_GPIO_Port GPIOD
#define LED_G_PIN_Pin GPIO_PIN_13
#define LED_G_PIN_GPIO_Port GPIOD
#define LED_R_PIN_Pin GPIO_PIN_14
#define LED_R_PIN_GPIO_Port GPIOD
#define LED_B_PIN_Pin GPIO_PIN_15
#define LED_B_PIN_GPIO_Port GPIOD
#define Motor_A8_Pin GPIO_PIN_8
#define Motor_A8_GPIO_Port GPIOA
#define Motor_D3_Pin GPIO_PIN_3
#define Motor_D3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

extern Vofa_HandleTypedef     vofa_inst_binding_uart3;
#define VOFA3                 vofa_inst_binding_uart3
// #define VOFA3_HUART           huart3
// #define VOFA3_HUART_DEFINE    USART3
#define VOFA3_HUART           huart1
#define VOFA3_HUART_DEFINE    USART1

#define DEBUG_HUART           huart3
#define DEBUG_HUART_DEFINE    USART3
#define DEBUG_HUART_MAX_DELAY 300U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
