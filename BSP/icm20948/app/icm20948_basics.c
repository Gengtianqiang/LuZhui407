
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

void printScaledAGMT_with_VOFA(ICM_20948 *sensor)
{
    double acc_x = accX(sensor);
    double acc_y = accY(sensor);
    double acc_z = accZ(sensor);
    double gyr_x = gyrX(sensor);
    double gyr_y = gyrY(sensor);
    double gyr_z = gyrZ(sensor);
    double mag_x = magX(sensor);
    double mag_y = magY(sensor);
    double mag_z = magZ(sensor);
    double temp = temperature(sensor);
    printf("AGMT:%.03lf,%.03lf,%.03lf,%.03lf,%.03lf,%.03lf,%.03lf,%.03lf,%.03lf,%.03lf\n", acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z, temp);
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

    while (true)
    {
        if (dataReady(&icm_20948))
        {
            getAGMT(&icm_20948);         // The values are only updated when you call 'getAGMT'
            // printScaledAGMT(&icm_20948); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
            printScaledAGMT_with_VOFA(&icm_20948); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
            HAL_Delay(30U);
        }
        else
        {
            printf("Waiting for data\r\n");
            HAL_Delay(500);
        }
    }
}


