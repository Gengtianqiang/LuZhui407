#include "bg_task.h"
#include "main.h"



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
		
		
  }
  /* USER CODE END StartLedTask */
}
