/* * Copyright (c) 2021 BlackWalnut Labs., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma diag_suppress 111
#include "icm20948_app.h"
#include "imu_output.h"
#include "main.h"
#define I2C_Handler hi2c1
#define printf(x, ...) Vofa_Printf(&VOFA3, x, ##__VA_ARGS__)


// #include <stdio.h>
#include <math.h>
#include <string.h>
// #include "ohos_init.h"
// #include "cmsis_os2.h"
// #include "iot_watchdog.h"
// #include "iot_i2c.h"
// #include "iot_io.h"
// #include "iot_errno.h"
#include "icm20948.h"


ICM_20948 icm_20948 = {0};
ICM_20948_Device_t device = {0};
ICM_20948_Serif_t serif = {0};


icm_20948_DMP_data_t data = {0};

float q1;
float q2;
float q3;
float q0;
int16_t acc_x;
int16_t acc_y;
int16_t acc_z;

float quat[4] = {0};
float acc[3] = {0};
// float acc_x_smooth = 0.0;

struct icm20948_app_data app_data = {0};    //更新校准后的值存在这



// 滤波
MovingAvgFilter aaa1 = {0} , aaa2 = {0}, aaa3 = {0};



ICM_20948_Status_e serif_i2c_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    uint8_t addr = ((ICM_20948 *)user)->_addr;

    HAL_StatusTypeDef retval = HAL_I2C_Mem_Write(&I2C_Handler, (addr << 1), regaddr, I2C_MEMADD_SIZE_8BIT, pdata, len, 1000U);

    return retval == HAL_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
}

ICM_20948_Status_e serif_i2c_read(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    uint8_t addr = ((ICM_20948 *)user)->_addr;
    
    HAL_StatusTypeDef retval = HAL_I2C_Mem_Read(&I2C_Handler, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buff, len, 1000U);

    return retval == HAL_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
}

float div_quat_data(int32_t data)
{
    // The quaternion data is scaled by 2^30.
    return (float)(((double)data) / 1073741824.0f);
}

float div_compass_Calibr_data(int32_t data)
{
    // The unit is uT scaled by 2^16
    return (float)(((double)data) / 65536.0f);
}

float div_gyro_Calibr_data(int16_t data)
{
    // The unit is degree per second scaled by 2^15
    return (float)(((double)data) / 32768.0f);
}

// Higher Level 获取量程
ICM_20948_Status_e getFss(ICM_20948 *icm_20948)
{
    if (icm_20948 == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    ICM_20948_Device_t *pdev = &icm_20948->_device;
    ICM_20948_AGMT_t *pagmt = &icm_20948->agmt;

	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	// Get settings to be able to compute scaled values
	retval |= ICM_20948_set_bank(pdev, 2);
	ICM_20948_ACCEL_CONFIG_t acfg;
	retval |= ICM_20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t *)&acfg, 1 * sizeof(acfg));
	pagmt->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range
									  // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
	retval |= ICM_20948_set_bank(pdev, 2);
	ICM_20948_GYRO_CONFIG_1_t gcfg1;
	retval |= ICM_20948_execute_r(pdev, (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&gcfg1, 1 * sizeof(gcfg1));
	pagmt->fss.g = gcfg1.GYRO_FS_SEL;
	ICM_20948_ACCEL_CONFIG_2_t acfg2;
	retval |= ICM_20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t *)&acfg2, 1 * sizeof(acfg2));

	return retval;
}

// 用四元数 修正线性加速度
void correct_acceleration(float accel[3], float quat[4]) {
    float gx = 2.0f * (quat[1] * quat[3] - quat[0] * quat[2]);
    float gy = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
    float gz = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);

    accel[0] -= (gx * 1000.0f);
    accel[1] -= (gy * 1000.0f);
    accel[2] -= (gz * 1000.0f);
}

// 滑动窗口均值滤波
// 只处理 acc[0] 专用
float smooth_acc0(float new_accel) {
    static float buffer[5] = {0};
    static int index = 0;
    buffer[index] = new_accel;
    index = (index + 1) % 5;
    float sum = 0;
    for (int i = 0; i < 5; i++) 
        sum += buffer[i];
    return (float)(sum / 5.0f);
}


// 滑动窗口均值滤波
int16_t MovingAvgFilter_Update(MovingAvgFilter *filter, int16_t new_value) {
    // 移除即将被覆盖的旧值
    filter->sum -= filter->buffer[filter->index];
    // 存储新值，并更新索引
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    // 更新索引（环形缓冲区）
    filter->index = (filter->index + 1) % FILTER_SIZE;
    // 返回平均值
    return (int16_t)(filter->sum / FILTER_SIZE);
}

void quat_to_euler(float q[4], euler_t *euler) {
    euler->roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 180.0f / PI;

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinp > 1.0f) sinp = 1.0f;    // 限制范围 [-1,1]
    if (sinp < -1.0f) sinp = -1.0f;  

    euler->pitch = asinf(sinp) * 180.0f / PI;  // 这里不需要额外乘以 180.0f / PI

    euler->yaw  = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                                1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 180.0f / PI;
}



#define CALIBRATION_SAMPLES 56  // 采样 56 次求均值 (静止状态下进行零漂校准) 56Hz执行一次，56刚好是1秒
// 这里可以实现加速度的校准逻辑
// "静止状态下"的采样数据计算零漂偏移值
void cali_offset_accel_x(const float newdata) {
    static float sum = 0.0f;    //采样次数太大可能会导致sum溢出
    static int i = 0;
    // 支持中途取消
    if(app_data.acc_x_offset_isCalibrated == true){
        i=0;
        sum=0.0f;
        return;
    }
    // 过程中
    if(i < CALIBRATION_SAMPLES) {
        // 累加
        sum += newdata;
        i++;
    }
    // 结束
    else if (i == CALIBRATION_SAMPLES) {
        float offset = (float)(sum / ((float)CALIBRATION_SAMPLES));  // 计算平均值作为零漂偏移值
        app_data.acc_x_offset = offset;     // 和上一次计算的偏移值累加
        app_data.acc_x_offset_isCalibrated = true;  // 标记为已校准
        //回归原值
        i=0;
        sum=0.0f;
    }
}

#define CALIBRATION_SAMPLES_YAW 14  // 采样 14 次求均值 (静止状态下进行零漂校准) 56Hz执行一次，14刚好是1/4秒
void cali_offset_euler_yaw(const float newdata){
    static float sum = 0.0f;        //采样次数太大可能会导致sum溢出
    static int i = 0;   
    // 支持中途取消
    if(app_data.euler_yaw_offset_isCalibrated == true){
        i=0;
        sum=0.0f;
        return;
    }
    // 过程中
    if(i < CALIBRATION_SAMPLES_YAW) {
        // 累加
        sum += newdata;
        i++;
    }
    // 结束
    else if (i == CALIBRATION_SAMPLES_YAW) {
        float offset = (float)(sum / ((float)CALIBRATION_SAMPLES_YAW));  // 计算平均值作为零漂偏移值
        app_data.euler_yaw_offset = offset;     // 和上一次计算的偏移值累加
        app_data.euler_yaw_offset_isCalibrated = true;  // 标记为已校准
        //回归原值
        i=0;
        sum=0.0f;
    }
}

// 更新 修正零漂之后的值
float updata_acc_x_current(float current_input) {
    app_data.acc_x_current = current_input - app_data.acc_x_offset;
    return app_data.acc_x_current;
}

// 更新 修正零漂之后的值
float updata_euler_yaw_current(float current_input) {
    app_data.euler_yaw_current = current_input - app_data.euler_yaw_offset;
    return app_data.euler_yaw_current;
}


bool ICM20948_APP_Init(void)
{
    // (void)args;

    serif.write = serif_i2c_write;
    serif.read = serif_i2c_read;
    serif.user = &icm_20948;
    device._serif = &serif;
    icm_20948._addr = ICM_20948_I2C_ADDR_AD0;
    icm_20948._device = device;

    icm20948_begin(&icm_20948);

    ICM_20948_Status_e icm_20948_status = ICM_20948_Stat_Ok; // Use icm_20948_status to show if the DMP configuration was ok

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    icm_20948_status = initializeDMP(&icm_20948);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Initialize DMP failed: %d!\r\n", icm_20948_status);
        return false;
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d246bw_n265bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d151bw8_n187bw6; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    icm_20948_status = setDLPFcfg(&icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("SetDLPF failed: %d!\r\n", icm_20948_status); 
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    icm_20948_status = enableDLPF(&icm_20948, ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("EnDLPF failed: %d!\r\n", icm_20948_status); 
    }

    icm_20948_status = getFss(&icm_20948);
    if(icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("GetFss failed: %d!\r\n", icm_20948_status);
    }


    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    icm_20948_status = enableDMPSensor(&icm_20948, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable DMP Sensor failed: %d!\r\n", icm_20948_status);
        return false;
    }
    // Enable any additional sensors / features
    //icm_20948_status |= enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) ;
    //icm_20948_status |= enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) ;
    //icm_20948_status |= enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) ;

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    icm_20948_status = setDMPODRrate(&icm_20948, DMP_ODR_Reg_Quat6, 0); // Set to the maximum
    // icm_20948_status = setDMPODRrate(&icm_20948, DMP_ODR_Reg_Quat9, 1); // Set to the maximum
    // icm_20948_status = setDMPODRrate(&icm_20948, DMP_ODR_Reg_Geomag, 1); // Set to the maximum
    icm_20948_status |= setDMPODRrate(&icm_20948, DMP_ODR_Reg_Accel, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(&icm_20948, DMP_ODR_Reg_Gyro, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(&icm_20948, DMP_ODR_Reg_Gyro_Calibr, 0) ; // Set to the maximum
    // icm_20948_status |= setDMPODRrate(&icm_20948, DMP_ODR_Reg_Cpass, 0) ; // Set to the maximum
    // icm_20948_status |= setDMPODRrate(&icm_20948, DMP_ODR_Reg_Cpass_Calibr, 0) ; // Set to the maximum
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Set DMP rate failed: %d!\r\n", icm_20948_status);
        return false;
    }

    // Enable the FIFO
    icm_20948_status = enableFIFO(&icm_20948, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable FIFO failed: %d!\r\n", icm_20948_status);
        return false;
    }

    // Enable the DMP
    icm_20948_status = enableDMP(&icm_20948, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable DMP failed: %d!\r\n", icm_20948_status);
        return false;
    }

    // Reset DMP
    icm_20948_status = resetDMP(&icm_20948);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Reset DMP failed: %d!\r\n", icm_20948_status);
        return false;
    }

    // Reset FIFO
    icm_20948_status = resetFIFO(&icm_20948);
    // Check icm_20948_status
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Reset FIFO failed: %d!\r\n", icm_20948_status);
        printf("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\r\n");
        return false;
    }
		
    // success
    return true;
}

// while中 时时刻刻 执行，这样不用担心数据同步问题，加了多重缓冲，输出看 imu_output.h
bool   ICM20948_APP_LOOP(void)
{

    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    ICM_HOP:
    readDMPdataFromFIFO(&icm_20948, &data);
    
    if(icm_20948.status == ICM_20948_Stat_FIFOMoreDataAvail) // Did we read more data than we should have?
    {
        goto ICM_HOP;  // read the  newest fifo data
    }

    if ((icm_20948.status == ICM_20948_Stat_Ok) || (icm_20948.status == ICM_20948_Stat_FIFOMoreDataAvail) ) // Was valid data available?
    {
        if (((data.header & DMP_header_bitmap_Quat6) > 0) && ((data.header & DMP_header_bitmap_Accel) > 0)) // We have asked for orientation data so we should receive Quat9
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.
            // Scale to +/- 1
            q1 = div_quat_data(data.Quat6.Data.Q1); // Convert to float. Divide by 2^30
            q2 = div_quat_data(data.Quat6.Data.Q2); // Convert to float. Divide by 2^30
            q3 = div_quat_data(data.Quat6.Data.Q3); // Convert to float. Divide by 2^30
            q0 = sqrt(1.0f - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            // uint16_t accuracy = data.Quat6.Data.Accuracy;
            
            acc_x = data.Raw_Accel.Data.X;
            acc_y = data.Raw_Accel.Data.Y;
            acc_z = data.Raw_Accel.Data.Z;

            // 滑动窗口滤波 (有问题，不用了)
            // acc_x = MovingAvgFilter_Update(&aaa1,  data.Raw_Accel.Data.X);
            // acc_y = MovingAvgFilter_Update(&aaa2,  data.Raw_Accel.Data.Y);
            // acc_z = MovingAvgFilter_Update(&aaa3,  data.Raw_Accel.Data.Z);
            
            // output the Quaternion data in the format for VOFA+
            // printf("Quat6+Acc:%.05f,%.05f,%.05f,%.05f,  %d,%d,%d\n", q0, q1, q2, q3, acc_x, acc_y, acc_z);
            
            // float quat[4];
            quat[0] = q0;
            quat[1] = q1;
            quat[2] = q2;
            quat[3] = q3;

            // float acc[3];
            acc[0] = (float) acc_x;
            acc[1] = (float) acc_y;
            acc[2] = (float) acc_z;

            // 修正线性加速度
            correct_acceleration(acc, quat);
            // printf("Acc:%.05f,%.05f,%.05f\n", acc[0], acc[1], acc[2]);

            // 滑动窗口滤波 (有问题，不用了)
            // acc_x_smooth = smooth_acc0(acc[0]);
            // printf("Quat6+Acc_x:%.05f,%.05f,%.05f,%.05f,  %.05f\n", q0, q1, q2, q3, app_data.acc_x_current);
            
            // 如何未进行零位校准，那么进行零位校准 (可被取消)
            cali_offset_accel_x(acc[0]);
            
            // 更新校准后的值  
            updata_acc_x_current(acc[0]);
            // printf("Quat6+Acc_x:%.05f,%.05f,%.05f,%.05f,  %.05f,%.05f,  %d,%d,%d,\n", 
            //     q0, q1, q2, q3,  
            //     app_data.acc_x_current, app_data.acc_x_offset,
            //     acc_x, acc_y, acc_z);
            
            // 欧拉角
            quat_to_euler(quat, &app_data.euler);
            // printf("euler:%.05f, %.05f, %.05f\n", app_data.euler.roll, app_data.euler.pitch, app_data.euler.yaw);

            // 如何未进行零位校准，那么进行零位校准 (可被取消)
            cali_offset_euler_yaw(app_data.euler.yaw);
            // printf("euler:%.05f, %.05f\n", app_data.euler.yaw, app_data.euler_yaw_offset);

            // 更新校准后的值
            updata_euler_yaw_current(app_data.euler.yaw);
            // roll: 左右不平   pitch: 前后不平  yaw: 朝向   yaw_curr: 朝向偏差
            // printf("euler:%.05f, %.05f, %.05f,   %.05f\n", app_data.euler.roll, app_data.euler.pitch, app_data.euler.yaw, app_data.euler_yaw_current);

            // 并输出数据到 imu_output.h
            imu_updata(&myimu);

            // ret
            return true;

        }
        // if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0)
        // {
        //     // Scale to +/- 1
        //     float q1 = div_compass_data(data.Compass_Calibr.Data.X); 
        //     float q2 = div_compass_data(data.Compass_Calibr.Data.Y); 
        //     float q3 = div_compass_data(data.Compass_Calibr.Data.Z); 
        //     float q0 = 0.5;

        //     // output the Quaternion data in the format for VOFA+
        //     printf("Compass_Calibr:%.03lf,%.03lf,%.03lf,%.03lf\n", q0, q1, q2, q3);
        // }
        return  false;
    }
    else{
        return false;
    }

    // if (icm_20948.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    // {
        // HAL_Delay(10U);
    // }
}








float get_acc_x_offset(void) {
    return app_data.acc_x_offset;
}

float get_acc_x_current(void) {
    return app_data.acc_x_current;
}

euler_t get_euler(void) {
    return app_data.euler;
}

float get_euler_yaw_Cali(void) {
    return app_data.euler_yaw_current;
}

/**
 * @brief 开始校准，之后程序会自动校准
 * @param Start_or_Stop 1:Start   0:Stop
 */
void icm_start_cali(bool  Start_or_Stop){
    app_data.acc_x_offset_isCalibrated = !Start_or_Stop;
    app_data.euler_yaw_offset_isCalibrated =!Start_or_Stop;
}

//  加速度 数据转换为mg单位
//  角速度 数据转换为dps单位
// 读取FIFO得到的已经是 根据量程 被转换过单位的



