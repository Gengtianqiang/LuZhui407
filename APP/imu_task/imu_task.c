#include "imu_task.h"
#include "main.h"



void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
		bool is_imu_updata = ICM20948_APP_LOOP();

    if(!is_imu_updata) {

      // Vofa_Printf(&vofa_inst_binding_uart3,  "IMU data update failed!\r\n");

    }
  }
  /* USER CODE END StartIMUTask */
}
