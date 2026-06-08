
#ifndef _ICM20948_APP_H_
#define _ICM20948_APP_H_

#include "stm32f4xx_hal.h"
#include "buzzer/buzzer.h"
#include "stdbool.h"
#include "imu_output.h"


struct icm20948_app_data
{
    float acc_x_offset;                  //x轴（前进方向）零漂偏移值
    bool   acc_x_offset_isCalibrated;     // 标记是否已经校准过零漂偏移值
    //x轴（前进方向）当前值 (零漂校准过)
    float acc_x_current;

    euler_t euler;
    float  euler_yaw_offset;
    bool    euler_yaw_offset_isCalibrated;
    float  euler_yaw_current;
};
extern struct icm20948_app_data     app_data;



// icm20948_app.c
#define FILTER_SIZE 3   // 滤波窗口大小
typedef struct {        // 结构体封装滤波器状态
    int16_t buffer[FILTER_SIZE];  // 环形缓冲区
    uint8_t index;                // 当前索引
    int32_t sum;                   // 滤波窗口内数据的累加和
} MovingAvgFilter;
extern MovingAvgFilter aaa1, aaa2, aaa3;
bool ICM20948_APP_Init(void);
bool ICM20948_APP_LOOP(void);
float get_acc_x_offset(void);
float get_acc_x_current(void);
euler_t get_euler(void);
float get_euler_yaw_Cali(void);
void icm_start_cali(bool  Start_or_Stop);



// icm20948_basics.c
// icm20948_advanced.c
void ICM20948_STARTUP(void);

// icm20948_DMP_Quat6_EulerAngles.c
void ICM20948_QUAT6(void *args);

// icm20948_DMP_Quat9_Orientation.c
void ICM20948_QUAT9(void *args);

#endif
