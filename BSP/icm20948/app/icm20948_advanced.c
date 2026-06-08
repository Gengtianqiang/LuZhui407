
#include "icm20948_app.h"
#include "main.h"
#define I2C_Handler hi2c1
#define printf(x, ...) Vofa_Printf(&VOFA3, x, ##__VA_ARGS__)

#include "icm20948.h"


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

void printScaledAGMT(ICM_20948 *sensor)
{
    printf("Scaled. Acc (mg) [ ");
    printf("%f", accX(sensor));
    printf(", ");
    printf("%f", accY(sensor));
    printf(", ");
    printf("%f", accZ(sensor));
    printf(" ], Gyr (DPS) [ ");
    printf("%f", gyrX(sensor));
    printf(", ");
    printf("%f", gyrY(sensor));
    printf(", ");
    printf("%f", gyrZ(sensor));
    printf(" ], Mag (uT) [ ");
    printf("%f", magX(sensor));
    printf(", ");
    printf("%f", magY(sensor));
    printf(", ");
    printf("%f", magZ(sensor));
    printf(" ], Tmp (C) [ ");
    printf("%f", temperature(sensor));
    printf(" ]");
    printf("\r\n");
}

void ICM20948_STARTUP(void)
{
    ICM_20948 icm_20948;
    ICM_20948_Device_t device = {0};
    ICM_20948_Serif_t serif = {0};
    serif.write = serif_i2c_write;
    serif.read = serif_i2c_read;
    serif.user = &icm_20948;
    device._serif = &serif;
    icm_20948._addr = ICM_20948_I2C_ADDR_AD0;
    icm_20948._device = device;

    ICM_20948_Status_e retval = icm20948_begin(&icm_20948);
    printf("icm20948_begin returned: ");
    printf(statusString(&icm_20948, retval));
    printf("\r\n");

    // Here we are doing a SW reset to make sure the device starts in a known state
    swReset(&icm_20948);

    if (icm_20948.status != ICM_20948_Stat_Ok)
    {
        printf("Software Reset returned: ");
        printf(statusString(&icm_20948, ICM_20948_Stat_NUM));
        printf("\r\n");
    }
    HAL_Delay(250);

    // Now wake the sensor up
    deviceSleep(&icm_20948, false);
    lowPower(&icm_20948, false);

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    setSampleMode(&icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (icm_20948.status != ICM_20948_Stat_Ok)
    {
        printf("setSampleMode returned: ");
        printf(statusString(&icm_20948, ICM_20948_Stat_NUM));
        printf("\r\n");
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                      // dps250
                      // dps500
                      // dps1000
                      // dps2000

    setFullScale(&icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (icm_20948.status != ICM_20948_Stat_Ok)
    {
        printf("setFullScale returned: ");
        printf(statusString(&icm_20948, ICM_20948_Stat_NUM));
        printf("\r\n");
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    setDLPFcfg(&icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (icm_20948.status != ICM_20948_Stat_Ok)
    {
        printf("setDLPcfg returned: ");
        printf(statusString(&icm_20948, ICM_20948_Stat_NUM));
        printf("\r\n");
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = enableDLPF(&icm_20948, ICM_20948_Internal_Acc, true);
    printf("Enable DLPF for Accelerometer returned: ");
    printf(statusString(&icm_20948, accDLPEnableStat));
    printf("\r\n");

    ICM_20948_Status_e gyrDLPEnableStat = enableDLPF(&icm_20948, ICM_20948_Internal_Gyr, true);
    printf("Enable DLPF for Gyroscope returned: ");
    printf(statusString(&icm_20948, gyrDLPEnableStat));
    printf("\r\n");

    // Choose whether or not to start the magnetometer
    startupMagnetometer(&icm_20948, false);
    if (icm_20948.status != ICM_20948_Stat_Ok)
    {
        printf("startupMagnetometer returned: ");
        printf(statusString(&icm_20948, ICM_20948_Stat_NUM));
        printf("\r\n");
    }

    printf("icm20948 Configuration complete!\r\n");

    while (true)
    {
        if (dataReady(&icm_20948))
        {
            getAGMT(&icm_20948);         // The values are only updated when you call 'getAGMT'
            printScaledAGMT(&icm_20948); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
            HAL_Delay(30);
        }
        else
        {
            printf("Waiting for data\r\n");
            HAL_Delay(500);
        }
    }
}

