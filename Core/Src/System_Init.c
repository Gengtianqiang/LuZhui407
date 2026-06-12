#include "System_Init.h"
#include "twr_control.h"
#include "freertos.h"
#include "queue.h"
#include "twr_control.h"
#include "jdy_driver.h"
#include "mesh_mode.h"

uwb_set uwb_set_t = {
	.pdoa_time = 0,
	.twr_time = 0,
	.pdoa_state = false,
		.twr_state = false,
};



QueueHandle_t xUART4_Queue = NULL;

void systemInit(void) 
{
  Car_init();
  //motor
  Motor_Init();
  // led
  LED_Init();
  Set_LED_State(&led_R, Off, 500);
  Set_LED_State(&led_G, blink, 500);
  Set_LED_State(&led_B, Off, 500);
	UART_Init();
  // buzzer
  Buzzer_Start_Duration(100, 150, 400);
 Vofa_Printf(&VOFA3, "Vofa+ Uart3 Init Success!\r\n");

 // battery
 Vofa_Printf(&VOFA3, "Battery Voltage: %.03fV\r\n", Get_battery_volt());

  // jdy28m
  HAL_Delay(300);
  Encoder_Init();



  bool is_icm_success = false;

 ICM_INIT_HOP:

  is_icm_success = ICM20948_APP_Init();

  int i = 1;
  while(is_icm_success == false){
    HAL_Delay(10);
    is_icm_success = ICM20948_APP_Init();
    i++;
    if(i > 2){
      if(is_icm_success == false) {

        my_4g_dtu.imu_error_flag = 1;
      }else{

        my_4g_dtu.imu_error_flag = 0;
      }

      i = 1;
      break;
    }
  }
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
	StateMachine_Init();
	
}

bool uwb_set_state() {
			if(!uwb_set_t.pdoa_state&&!uwb_set_t.twr_state) {
			uwb_set_t.pdoa_time++;
			uwb_set_t.twr_time++;
			if(uwb_set_t.pdoa_time>100) {
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
				uwb_set_t.pdoa_state = true;
				uwb_set_t.twr_state = true;
			}
		}
			if(uwb_set_t.pdoa_state&&uwb_set_t.pdoa_state)	return true;
			return false;
			
}

void RTOS_Init(void) {
  xUART4_Queue = xQueueCreate(10, sizeof(UART4_Queue_Data_t));
}
