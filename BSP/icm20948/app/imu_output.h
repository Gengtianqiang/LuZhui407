#ifndef IMU_OUTPUT_H
#define IMU_OUTPUT_H
#include "stm32f4xx_hal.h"


// from app.c
// extern double q1;
// extern double q2;
// extern double q3;
// extern double q0;
// extern double quat[4];
// extern int16_t acc_x;
// extern int16_t acc_y;
// extern int16_t acc_z;


#ifndef PI
#define PI                 3.1415926f
#endif

typedef struct {
    float      x;
} linear_acc_t;


typedef struct {
    float      roll;        //左右不平
    float      pitch;       //前后不平
    float      yaw;         //朝向
} euler_t;


typedef struct {
    linear_acc_t    linear_acc;
    euler_t         euler;
    float           euler_yaw_Cali;  //朝向 相对于静止时刻的偏置
} imu_t;




extern imu_t       myimu;

void   imu_updata(imu_t *myimu);



#endif
