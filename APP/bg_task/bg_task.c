#include "bg_task.h"
#include "main.h"
#include "bsp_4g.h"
#include "NRF24L01.h"
#include "uart_callback.h"


uint8_t FLAG;
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  
  for(;;)
  {
    osDelay(100);
		LED_Loop(&led_R);
    LED_Loop(&led_G);
    LED_Loop(&led_B);
		Buzzer_LOOP();

    Volt = Get_battery_volt();
    Set_battery_led();
		

		if(my_4g_dtu.buzzer_flag == 1||my_4g_dtu.imu_error_flag == 1 || FLAG) {


      if(1==my_4g_dtu.buzzer_flag)
            Buzzer_Start_Circle(200, 200); 
      else if(1==my_4g_dtu.imu_error_flag){

          Buzzer_Start_Circle(300, 100); 

      }else {
          Buzzer_Start_Circle(100, 300); 


      }
      FLAG = 0;
    } else {
        Buzzer_Stop();
      
    }
  }
  /* USER CODE END StartLedTask */
}
