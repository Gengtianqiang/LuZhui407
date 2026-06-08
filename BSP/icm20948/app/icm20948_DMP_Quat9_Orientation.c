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
#include "icm20948_app.h"
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
usart2_rx_DMA_buffer 
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

double div_quat_data(int32_t data)
{
    return (double)data / 1073741824.0;
}

void ICM20948_QUAT9(void *args)
{
    (void)args;

    ICM_20948 icm_20948;
    ICM_20948_Device_t device = {0};
    ICM_20948_Serif_t serif = {0};
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
        while (1)
            ; // Do nothing more
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
    icm_20948_status = enableDMPSensor(&icm_20948, INV_ICM20948_SENSOR_ORIENTATION, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable DMP Sensor failed: %d!\r\n", icm_20948_status);
        while (1)
            ; // Do nothing more
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
    icm_20948_status = setDMPODRrate(&icm_20948, DMP_ODR_Reg_Quat9, 1); // Set to the maximum
    //icm_20948_status |= setDMPODRrate(DMP_ODR_Reg_Accel, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(DMP_ODR_Reg_Gyro, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(DMP_ODR_Reg_Cpass, 0) ; // Set to the maximum
    //icm_20948_status |= setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) ; // Set to the maximum
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Set DMP rate failed: %d!\r\n", icm_20948_status);
        while (1)
            ; // Do nothing more
    }

    // Enable the FIFO
    icm_20948_status = enableFIFO(&icm_20948, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable FIFO failed: %d!\r\n", icm_20948_status);
        while (1)
            ; // Do nothing more
    }

    // Enable the DMP
    icm_20948_status = enableDMP(&icm_20948, true);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Enable DMP failed: %d!\r\n", icm_20948_status);
        while (1)
            ; // Do nothing more
    }

    // Reset DMP
    icm_20948_status = resetDMP(&icm_20948);
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Reset DMP failed: %d!\r\n", icm_20948_status);
        while (1)
            ; // Do nothing more
    }

    // Reset FIFO
    icm_20948_status = resetFIFO(&icm_20948);
    // Check icm_20948_status
    if (icm_20948_status != ICM_20948_Stat_Ok)
    {
        printf("Reset FIFO failed: %d!\r\n", icm_20948_status);
        printf("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\r\n");
        while (1)
            ; // Do nothing more
    }

    icm_20948_DMP_data_t data;
    while (true)
    {
        // Read any DMP data waiting in the FIFO
        // Note:
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
        //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
        //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
        //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
        readDMPdataFromFIFO(&icm_20948, &data);

        if ((icm_20948.status == ICM_20948_Stat_Ok) || (icm_20948.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
        {
            if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
            {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.

                // printf("Quat9 data is: Q1:%d Q2:%d Q3:%d Accuracy:%d\r\n",
                //        data.Quat9.Data.Q1,
                //        data.Quat9.Data.Q2,
                //        data.Quat9.Data.Q3,
                //        data.Quat9.Data.Accuracy);

                // Scale to +/- 1
                double q1 = div_quat_data(data.Quat9.Data.Q1); // Convert to double. Divide by 2^30
                double q2 = div_quat_data(data.Quat9.Data.Q2); // Convert to double. Divide by 2^30
                double q3 = div_quat_data(data.Quat9.Data.Q3); // Convert to double. Divide by 2^30
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                // printf("[q1: %.2lf", q1);
                // printf(" q2: %.2lf", q2);
                // printf(" q3: %.2lf", q3);
                // printf(" q0: %.2lf", q0);
                // printf("]\r\n");

                // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
                // printf("{ \"quat_w\":%lf", q0);
                // printf(", \"quat_x\":%lf", q1);
                // printf(", \"quat_y\":%lf", q2);
                // printf(", \"quat_z\":%lf", q3);
                // printf(" }\r\n");

                // output the Quaternion data in the format for VOFA+
                printf("Quat9:%.03lf,%.03lf,%.03lf,%.03lf\n", q0, q1, q2, q3);
            }
        }

        if (icm_20948.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
        {
            HAL_Delay(10U);
        }
    }
}

