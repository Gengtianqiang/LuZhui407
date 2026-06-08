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
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <math.h>
// #include <unistd.h>
#include "icm20948.h"
// #include "ohos_init.h"
// #include "cmsis_os2.h"

#define TRACE_CMH_1 (printf("%s(%d)-<%s>: ", __FILE__, __LINE__, __FUNCTION__), printf)

const uint8_t MAX_MAGNETOMETER_STARTS = 10; // This replaces maxTries

double getTempC(int16_t val);
double getGyrDPS(ICM_20948 *icm_20948, int16_t axis_val);
double getAccMG(ICM_20948 *icm_20948, int16_t axis_val);
double getMagUT(int16_t axis_val);

static void Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

ICM_20948_Status_e icm20948_begin(ICM_20948 *icm_20948)
{
    if (icm_20948->_device._serif == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

#if defined(ICM_20948_USE_DMP)
    icm_20948->_device._dmp_firmware_available = true; // Initialize _dmp_firmware_available
#else
    icm_20948->_device._dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

    icm_20948->_device._firmware_loaded = false; // Initialize _firmware_loaded
    icm_20948->_device._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
    icm_20948->_device._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
    icm_20948->_device._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    icm_20948->_device._gyroSFpll = 0;
    icm_20948->_device._enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
    icm_20948->_device._enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
    icm_20948->_device._enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
    icm_20948->_device._enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-

    // Perform default startup
    // Do a minimal startupDefault if using the DMP. User can always call startupDefault(false) manually if required.
    icm_20948->status = startupDefault(icm_20948, icm_20948->_device._dmp_firmware_available);
    return icm_20948->status;
}

ICM_20948_Status_e startupMagnetometer(ICM_20948 *icm_20948, bool minimal)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    i2cMasterPassthrough(icm_20948, false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
    i2cMasterEnable(icm_20948, true);

    resetMag(icm_20948);

    //After a ICM reset the Mag sensor may stop responding over the I2C master
    //Reset the Master I2C until it responds
    uint8_t tries = 0;
    while (tries < MAX_MAGNETOMETER_STARTS)
    {
        tries++;

        //See if we can read the WhoIAm register correctly
        retval = magWhoIAm(icm_20948);
        if (retval == ICM_20948_Stat_Ok)
            break; //WIA matched!

        i2cMasterReset(icm_20948); //Otherwise, reset the master I2C and try again

        Delay(10);
    }

    if (tries == MAX_MAGNETOMETER_STARTS)
    {
        icm_20948->status = ICM_20948_Stat_WrongID;
        return icm_20948->status;
    }

    //Return now if minimal is true. The mag will be configured manually for the DMP
    if (minimal) // Return now if minimal is true
    {
        return icm_20948->status;
    }

    //Set up magnetometer
    AK09916_CNTL2_Reg_t reg;
    reg.MODE = AK09916_mode_cont_100hz;
    retval = writeMag(icm_20948, AK09916_REG_CNTL2, (uint8_t *)&reg);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = i2cControllerConfigurePeripheral(icm_20948, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, AK09916_mode_power_down);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    return icm_20948->status;
}

ICM_20948_Status_e magWhoIAm(ICM_20948 *icm_20948)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    uint8_t whoiam1, whoiam2;
    whoiam1 = readMag(icm_20948, AK09916_REG_WIA1);
    // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
    // i2cMasterSingleR updates icm_20948->status so it is OK to set retval to icm_20948->status here
    retval = icm_20948->status;
    if (retval != ICM_20948_Stat_Ok)
    {
        return retval;
    }

    whoiam2 = readMag(icm_20948, AK09916_REG_WIA2);
    // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
    // i2cMasterSingleR updates icm_20948->status so it is OK to set retval to icm_20948->status here
    retval = icm_20948->status;
    if (retval != ICM_20948_Stat_Ok)
    {
        return retval;
    }

    if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
    {
        retval = ICM_20948_Stat_Ok;
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = ICM_20948_Stat_WrongID;
    icm_20948->status = retval;
    return icm_20948->status;
}

ICM_20948_AGMT_t getAGMT(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_get_agmt(&icm_20948->_device, &icm_20948->agmt);

    return icm_20948->agmt;
}

double magX(ICM_20948 *icm_20948)
{
    return getMagUT(icm_20948->agmt.mag.axes.x);
}

double magY(ICM_20948 *icm_20948)
{
    return getMagUT(icm_20948->agmt.mag.axes.y);
}

double magZ(ICM_20948 *icm_20948)
{
    return getMagUT(icm_20948->agmt.mag.axes.z);
}

double getMagUT(int16_t axis_val)
{
    return (((double)axis_val) * 0.15);
}

double accX(ICM_20948 *icm_20948)
{
    return getAccMG(icm_20948, icm_20948->agmt.acc.axes.x);
}

double accY(ICM_20948 *icm_20948)
{
    return getAccMG(icm_20948, icm_20948->agmt.acc.axes.y);
}

double accZ(ICM_20948 *icm_20948)
{
    return getAccMG(icm_20948, icm_20948->agmt.acc.axes.z);
}

double getAccMG(ICM_20948 *icm_20948, int16_t axis_val)
{
    switch (icm_20948->agmt.fss.a)
    {
    case 0:
        return (((double)axis_val) / 16.384);//Fss=±2g
        break;
    case 1:
        return (((double)axis_val) / 8.192);//Fss=±4g
        break;
    case 2:
        return (((double)axis_val) / 4.096);//Fss=±8g
        break;
    case 3:
        return (((double)axis_val) / 2.048);//Fss=±16g
        break;
    default:
        return 0;
        break;
    }
}

double gyrX(ICM_20948 *icm_20948)
{
    return getGyrDPS(icm_20948, icm_20948->agmt.gyr.axes.x);
}

double gyrY(ICM_20948 *icm_20948)
{
    return getGyrDPS(icm_20948, icm_20948->agmt.gyr.axes.y);
}

double gyrZ(ICM_20948 *icm_20948)
{
    return getGyrDPS(icm_20948, icm_20948->agmt.gyr.axes.z);
}

double getGyrDPS(ICM_20948 *icm_20948, int16_t axis_val)
{
    switch (icm_20948->agmt.fss.g)
    {
    case 0:
        return (((double)axis_val) / 131.0);//Fss=±250dps
        break;
    case 1:
        return (((double)axis_val) / 65.5);//Fss=±500dps
        break;
    case 2:
        return (((double)axis_val) / 32.8);//Fss=±1000dps
        break;
    case 3:
        return (((double)axis_val) / 16.4);//Fss=±2000dps
        break;
    default:
        return 0;
        break;
    }
}

double temperature(ICM_20948 *icm_20948)
{
    return getTempC(icm_20948->agmt.tmp.val);
}

double getTempC(int16_t val)
{
    // Fix double add bug
    return (((double)(val * 100 + 699027)) / 33387);
    // return (((double)val - 21) / 333.87) + 21;
}

const char *statusString(ICM_20948 *icm_20948, ICM_20948_Status_e stat)
{
    ICM_20948_Status_e val;
    if (stat == ICM_20948_Stat_NUM)
    {
        val = icm_20948->status;
    }
    else
    {
        val = stat;
    }

    switch (val)
    {
    case ICM_20948_Stat_Ok:
        return "All is well.";
        break;
    case ICM_20948_Stat_Err:
        return "General Error";
        break;
    case ICM_20948_Stat_NotImpl:
        return "Not Implemented";
        break;
    case ICM_20948_Stat_ParamErr:
        return "Parameter Error";
        break;
    case ICM_20948_Stat_WrongID:
        return "Wrong ID";
        break;
    case ICM_20948_Stat_InvalSensor:
        return "Invalid Sensor";
        break;
    case ICM_20948_Stat_NoData:
        return "Data Underflow";
        break;
    case ICM_20948_Stat_SensorNotSupported:
        return "Sensor Not Supported";
        break;
    case ICM_20948_Stat_DMPNotSupported:
        return "DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?";
        break;
    case ICM_20948_Stat_DMPVerifyFail:
        return "DMP Firmware Verification Failed";
        break;
    case ICM_20948_Stat_FIFONoDataAvail:
        return "No FIFO Data Available";
        break;
    case ICM_20948_Stat_FIFOIncompleteData:
        return "DMP data in FIFO was incomplete";
        break;
    case ICM_20948_Stat_FIFOMoreDataAvail:
        return "More FIFO Data Available";
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader:
        return "Unrecognised DMP Header";
        break;
    case ICM_20948_Stat_UnrecognisedDMPHeader2:
        return "Unrecognised DMP Header2";
        break;
    case ICM_20948_Stat_InvalDMPRegister:
        return "Invalid DMP Register";
        break;
    default:
        return "Unknown Status";
        break;
    }
    return "None";
}

// Device Level
ICM_20948_Status_e setBank(ICM_20948 *icm_20948, uint8_t bank)
{
    icm_20948->status = ICM_20948_set_bank(&icm_20948->_device, bank);
    return icm_20948->status;
}

ICM_20948_Status_e swReset(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_sw_reset(&icm_20948->_device);
    return icm_20948->status;
}

ICM_20948_Status_e deviceSleep(ICM_20948 *icm_20948, bool on)
{
    icm_20948->status = ICM_20948_sleep(&icm_20948->_device, on);
    return icm_20948->status;
}

ICM_20948_Status_e lowPower(ICM_20948 *icm_20948, bool on)
{
    icm_20948->status = ICM_20948_low_power(&icm_20948->_device, on);
    return icm_20948->status;
}

ICM_20948_Status_e setClockSource(ICM_20948 *icm_20948, ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
    icm_20948->status = ICM_20948_set_clock_source(&icm_20948->_device, source);
    return icm_20948->status;
}

ICM_20948_Status_e checkID(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_check_id(&icm_20948->_device);
    return icm_20948->status;
}

bool dataReady(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_data_ready(&icm_20948->_device);
    if (icm_20948->status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    return false;
}

uint8_t getWhoAmI(ICM_20948 *icm_20948)
{
    uint8_t retval = 0x00;
    icm_20948->status = ICM_20948_get_who_am_i(&icm_20948->_device, &retval);
    return retval;
}

bool isConnected(ICM_20948 *icm_20948)
{
    icm_20948->status = checkID(icm_20948);
    if (icm_20948->status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    return false;
}

// Internal Sensor Options
ICM_20948_Status_e setSampleMode(ICM_20948 *icm_20948, uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
    icm_20948->status = ICM_20948_set_sample_mode(&icm_20948->_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
    return icm_20948->status;
}

ICM_20948_Status_e setFullScale(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
    icm_20948->status = ICM_20948_set_full_scale(&icm_20948->_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
    return icm_20948->status;
}

ICM_20948_Status_e setDLPFcfg(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
    icm_20948->status = ICM_20948_set_dlpf_cfg(&icm_20948->_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
    return icm_20948->status;
}

ICM_20948_Status_e enableDLPF(ICM_20948 *icm_20948, uint8_t sensor_id_bm, bool enable)
{
    icm_20948->status = ICM_20948_enable_dlpf(&icm_20948->_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
    return icm_20948->status;
}

ICM_20948_Status_e setSampleRate(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
    icm_20948->status = ICM_20948_set_sample_rate(&icm_20948->_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
    return icm_20948->status;
}

//      All these individual functions will use a read->set->write method to leave other settings untouched
ICM_20948_Status_e intEnableI2C(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.I2C_MST_INT_EN = enable;                                              // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.I2C_MST_INT_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableDMP(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.DMP_INT1_EN = enable;                                                 // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.DMP_INT1_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnablePLL(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.PLL_RDY_EN = enable;                                                  // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.PLL_RDY_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableWOM(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.WOM_INT_EN = enable;                                                  // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.WOM_INT_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableWOF(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.REG_WOF_EN = enable;                                                  // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.REG_WOF_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableRawDataReady(ICM_20948 *icm_20948, bool enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.RAW_DATA_0_RDY_EN = enable;                                           // change the setting
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (en.RAW_DATA_0_RDY_EN != enable)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableOverflowFIFO(ICM_20948 *icm_20948, uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e intEnableWatermarkFIFO(ICM_20948 *icm_20948, uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                                                // storage
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, NULL, &en); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
    icm_20948->status = ICM_20948_int_enable(&icm_20948->_device, &en, &en); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    return icm_20948->status;
}

ICM_20948_Status_e WOMThreshold(ICM_20948 *icm_20948, uint8_t threshold)
{
    ICM_20948_ACCEL_WOM_THR_t thr;                                                // storage
    icm_20948->status = ICM_20948_wom_threshold(&icm_20948->_device, NULL, &thr); // read phase
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    thr.WOM_THRESHOLD = threshold;                                                // change the setting
    icm_20948->status = ICM_20948_wom_threshold(&icm_20948->_device, &thr, &thr); // write phase w/ readback
    if (icm_20948->status != ICM_20948_Stat_Ok)
    {
        return icm_20948->status;
    }
    if (thr.WOM_THRESHOLD != threshold)
    {
        icm_20948->status = ICM_20948_Stat_Err;
        return icm_20948->status;
    }
    return icm_20948->status;
}

// Interface Options
ICM_20948_Status_e i2cMasterPassthrough(ICM_20948 *icm_20948, bool passthrough)
{
    icm_20948->status = ICM_20948_i2c_master_passthrough(&icm_20948->_device, passthrough);
    return icm_20948->status;
}

ICM_20948_Status_e i2cMasterEnable(ICM_20948 *icm_20948, bool enable)
{
    icm_20948->status = ICM_20948_i2c_master_enable(&icm_20948->_device, enable);
    return icm_20948->status;
}

ICM_20948_Status_e i2cMasterReset(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_i2c_master_reset(&icm_20948->_device);
    return icm_20948->status;
}

ICM_20948_Status_e i2cControllerConfigurePeripheral(ICM_20948 *icm_20948, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t data_out)
{
    icm_20948->status = ICM_20948_i2c_controller_configure_peripheral(&icm_20948->_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, data_out);
    return icm_20948->status;
}

ICM_20948_Status_e i2cControllerPeriph4Transaction(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
    icm_20948->status = ICM_20948_i2c_controller_periph4_txn(&icm_20948->_device, addr, reg, data, len, Rw, send_reg_addr);
    return icm_20948->status;
}

//Used for configuring the Magnetometer
ICM_20948_Status_e i2cMasterSingleW(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg, uint8_t data)
{
    icm_20948->status = ICM_20948_i2c_master_single_w(&icm_20948->_device, addr, reg, &data);
    return icm_20948->status;
}
uint8_t i2cMasterSingleR(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg)
{
    uint8_t data = 0;
    icm_20948->status = ICM_20948_i2c_master_single_r(&icm_20948->_device, addr, reg, &data);
    return data;
}

// Default Setup
ICM_20948_Status_e startupDefault(ICM_20948 *icm_20948, bool minimal)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    retval = checkID(icm_20948);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = swReset(icm_20948);

    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }
    Delay(500);

    retval = deviceSleep(icm_20948, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = lowPower(icm_20948, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = startupMagnetometer(icm_20948, minimal);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    if (minimal) // Return now if minimal is true
    {
        return icm_20948->status;
    }

    retval = setSampleMode(icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled

    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

    ICM_20948_fss_t FSS;
    FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    retval = setFullScale(icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);

    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d473bw_n499bw;
    dlpcfg.g = gyr_d361bw4_n376bw5;
    retval = setDLPFcfg(icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = enableDLPF(icm_20948, ICM_20948_Internal_Acc, false);
    if (retval != ICM_20948_Stat_Ok)
    {

        icm_20948->status = retval;
        return icm_20948->status;
    }

    retval = enableDLPF(icm_20948, ICM_20948_Internal_Gyr, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        icm_20948->status = retval;
        return icm_20948->status;
    }

    return icm_20948->status;
}

// direct read/write
ICM_20948_Status_e directRead(ICM_20948 *icm_20948, uint8_t reg, uint8_t *pdata, uint32_t len)
{
    icm_20948->status = ICM_20948_execute_r(&icm_20948->_device, reg, pdata, len);
    return (icm_20948->status);
}

ICM_20948_Status_e directWrite(ICM_20948 *icm_20948, uint8_t reg, uint8_t *pdata, uint32_t len)
{
    icm_20948->status = ICM_20948_execute_w(&icm_20948->_device, reg, pdata, len);
    return (icm_20948->status);
}

//Mag specific
uint8_t readMag(ICM_20948 *icm_20948, AK09916_Reg_Addr_e reg)
{
    uint8_t data = i2cMasterSingleR(icm_20948, MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates icm_20948->status too
    return data;
}

ICM_20948_Status_e writeMag(ICM_20948 *icm_20948, AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
    icm_20948->status = i2cMasterSingleW(icm_20948, MAG_AK09916_I2C_ADDR, reg, *pdata);
    return icm_20948->status;
}

ICM_20948_Status_e resetMag(ICM_20948 *icm_20948)
{
    uint8_t SRST = 1;
    // SRST: Soft reset
    // “0”: Normal
    // “1”: Reset
    // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
    icm_20948->status = i2cMasterSingleW(icm_20948, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
    return icm_20948->status;
}

// FIFO
ICM_20948_Status_e enableFIFO(ICM_20948 *icm_20948, bool enable)
{
    icm_20948->status = ICM_20948_enable_FIFO(&icm_20948->_device, enable);
    return icm_20948->status;
}

ICM_20948_Status_e resetFIFO(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_reset_FIFO(&icm_20948->_device);
    return icm_20948->status;
}

ICM_20948_Status_e setFIFOmode(ICM_20948 *icm_20948, bool snapshot)
{
    // Default to Stream (non-Snapshot) mode
    icm_20948->status = ICM_20948_set_FIFO_mode(&icm_20948->_device, snapshot);
    return icm_20948->status;
}

ICM_20948_Status_e getFIFOcount(ICM_20948 *icm_20948, uint16_t *count)
{
    icm_20948->status = ICM_20948_get_FIFO_count(&icm_20948->_device, count);
    return icm_20948->status;
}

ICM_20948_Status_e readFIFO(ICM_20948 *icm_20948, uint8_t *data, uint8_t len)
{
    icm_20948->status = ICM_20948_read_FIFO(&icm_20948->_device, data, len);
    return icm_20948->status;
}

// DMP

ICM_20948_Status_e enableDMP(ICM_20948 *icm_20948, bool enable)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to enable the DMP?
    {
        icm_20948->status = ICM_20948_enable_DMP(&icm_20948->_device, enable == true ? 1 : 0);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e resetDMP(ICM_20948 *icm_20948)
{
    icm_20948->status = ICM_20948_reset_DMP(&icm_20948->_device);
    return icm_20948->status;
}

ICM_20948_Status_e loadDMPFirmware(ICM_20948 *icm_20948)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to load the DMP firmware?
    {
        icm_20948->status = ICM_20948_firmware_load(&icm_20948->_device);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e setDMPstartAddress(ICM_20948 *icm_20948, unsigned short address)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to set the start address?
    {
        icm_20948->status = ICM_20948_set_dmp_start_address(&icm_20948->_device, address);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e enableDMPSensor(ICM_20948 *icm_20948, enum inv_icm20948_sensor sensor, bool enable)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to enable the sensor?
    {
        icm_20948->status = inv_icm20948_enable_dmp_sensor(&icm_20948->_device, sensor, enable == true ? 1 : 0);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e enableDMPSensorInt(ICM_20948 *icm_20948, enum inv_icm20948_sensor sensor, bool enable)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to enable the sensor interrupt?
    {
        icm_20948->status = inv_icm20948_enable_dmp_sensor_int(&icm_20948->_device, sensor, enable == true ? 1 : 0);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e writeDMPmems(ICM_20948 *icm_20948, unsigned short reg, unsigned int length, const unsigned char *data)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to write to the DMP?
    {
        icm_20948->status = inv_icm20948_write_mems(&icm_20948->_device, reg, length, data);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e readDMPmems(ICM_20948 *icm_20948, unsigned short reg, unsigned int length, unsigned char *data)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to read from the DMP?
    {
        icm_20948->status = inv_icm20948_read_mems(&icm_20948->_device, reg, length, data);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e setDMPODRrate(ICM_20948 *icm_20948, enum DMP_ODR_Registers odr_reg, int interval)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to set the DMP ODR?
    {
        // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
        // Setting value can be calculated as follows:
        // Value = (DMP running rate (225Hz) / ODR ) - 1
        // E.g. For a 25Hz ODR rate, value= (225/25) - 1 = 8.

        icm_20948->status = inv_icm20948_set_dmp_sensor_period(&icm_20948->_device, odr_reg, interval);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e readDMPdataFromFIFO(ICM_20948 *icm_20948, icm_20948_DMP_data_t *data)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to set the data from the FIFO?
    {
        icm_20948->status = inv_icm20948_read_dmp_data(&icm_20948->_device, data);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e setGyroSF(ICM_20948 *icm_20948, unsigned char div, int gyro_level)
{
    if (icm_20948->_device._dmp_firmware_available == true) // Should we attempt to set the Gyro SF?
    {
        icm_20948->status = inv_icm20948_set_gyro_sf(&icm_20948->_device, div, gyro_level);
        return icm_20948->status;
    }
    return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e initializeDMP(ICM_20948 *icm_20948)
{
    // First, let's check if the DMP is available
    if (icm_20948->_device._dmp_firmware_available != true)
    {
        return ICM_20948_Stat_DMPNotSupported;
    }

    ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

#if defined(ICM_20948_USE_DMP)

    // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
    // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

    ICM_20948_Status_e result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

    // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
    // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
    // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
    // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
    //
    // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
    // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
    // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
    // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
    // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
    //
    // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
    // 0: use I2C_SLV0
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
    // 10: we read 10 bytes each cycle
    // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
    result = i2cControllerConfigurePeripheral(icm_20948, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0);
    if (result > worstResult)
        worstResult = result;

    // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
    // 1: use I2C_SLV1
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_CNTL2: we start writing here (0x31)
    // 1: not sure why, but the write does not happen if this is set to zero
    // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
    // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
    result = i2cControllerConfigurePeripheral(icm_20948, 1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
    if (result > worstResult)
        worstResult = result;

    // Set the I2C Master ODR configuration
    // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
    // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
    //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
    //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
    //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
    // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
    // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
    result = setBank(icm_20948, 3);
    if (result > worstResult)
        worstResult = result; // Select Bank 3

    uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
    result = directWrite(icm_20948, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
    if (result > worstResult)
        worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register

    // Configure clock source through PWR_MGMT_1
    // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    result = setClockSource(icm_20948, ICM_20948_Clock_Auto);
    if (result > worstResult)
        worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

    // Enable accel and gyro sensors through PWR_MGMT_2
    // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
    result = setBank(icm_20948, 0);
    if (result > worstResult)
        worstResult = result; // Select Bank 0

    uint8_t pwrMgmt2 = 0x40; // Set the reserved bit 6 (pressure sensor disable?)
    result = directWrite(icm_20948, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);
    if (result > worstResult)
        worstResult = result; // Write one byte to the PWR_MGMT_2 register

    // Configure I2C_Master/Gyro/Accel in Low Power Mode (cycled) with LP_CONFIG
    result = setSampleMode(icm_20948, (ICM_20948_Internal_Mst | ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
    if (result > worstResult)
        worstResult = result;

    // Disable the FIFO
    result = enableFIFO(icm_20948, false);
    if (result > worstResult)
        worstResult = result;

    // Disable the DMP
    result = enableDMP(icm_20948, false);
    if (result > worstResult)
        worstResult = result;

    // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
    // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                           // gpm2
                           // gpm4
                           // gpm8
                           // gpm16
    myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                           // dps250
                           // dps500
                           // dps1000
                           // dps2000
    result = setFullScale(icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (result > worstResult)
        worstResult = result;

    // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
    // If we see this interrupt, we'll need to reset the FIFO
    //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

    // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
    // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
    result = setBank(icm_20948, 0);
    if (result > worstResult)
        worstResult = result; // Select Bank 0

    uint8_t zero = 0;
    result = directWrite(icm_20948, AGB0_REG_FIFO_EN_1, &zero, 1);
    if (result > worstResult)
        worstResult = result;

    // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
    result = directWrite(icm_20948, AGB0_REG_FIFO_EN_2, &zero, 1);
    if (result > worstResult)
        worstResult = result;

    // Turn off data ready interrupt through INT_ENABLE_1
    result = intEnableRawDataReady(icm_20948, false);
    if (result > worstResult)
        worstResult = result;

    // Reset FIFO through FIFO_RST
    result = resetFIFO(icm_20948);
    if (result > worstResult)
        worstResult = result;

    // Set gyro sample rate divider with GYRO_SMPLRT_DIV
    // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
    ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
    mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
    // ** Note: comment the next line to leave the sample rates at the maximum **
    result = setSampleRate(icm_20948, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
    if (result > worstResult)
        worstResult = result;

    // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress(icm_20948, DMP_START_ADDRESS);
    if (result > worstResult)
        worstResult = result; // Defaults to DMP_START_ADDRESS

    // Now load the DMP firmware
    result = loadDMPFirmware(icm_20948);
    if (result > worstResult)
        worstResult = result;

    // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress(icm_20948, DMP_START_ADDRESS);
    if (result > worstResult)
        worstResult = result; // Defaults to DMP_START_ADDRESS

    // Set the Hardware Fix Disable register to 0x48
    result = setBank(icm_20948, 0);
    if (result > worstResult)
        worstResult = result; // Select Bank 0

    uint8_t fix = 0x48;
    result = directWrite(icm_20948, AGB0_REG_HW_FIX_DISABLE, &fix, 1);
    if (result > worstResult)
        worstResult = result;

    // Set the Single FIFO Priority Select register to 0xE4
    result = setBank(icm_20948, 0);
    if (result > worstResult)
        worstResult = result; // Select Bank 0

    uint8_t fifoPrio = 0xE4;
    result = directWrite(icm_20948, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
    if (result > worstResult)
        worstResult = result;

    // Configure Accel scaling to DMP
    // The DMP scales accel raw data internally to align 1g as 2^25
    // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
    const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
    result = writeDMPmems(icm_20948, ACC_SCALE, 4, &accScale[0]);
    if (result > worstResult)
        worstResult = result; // Write accScale to ACC_SCALE DMP register

    // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    result = writeDMPmems(icm_20948, ACC_SCALE2, 4, &accScale2[0]);
    if (result > worstResult)
        worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

    // Configure Compass mount matrix and scale to DMP
    // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
    // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
    // Each compass axis will be converted as below:
    // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
    // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
    // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
    // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
    // 2^30 / 6.66666 = 161061273 = 0x9999999
    const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
    const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(icm_20948, CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
    if (result > worstResult)
        worstResult = result;

    result = writeDMPmems(icm_20948, CPASS_MTX_01, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_02, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_10, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_12, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_20, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_21, 4, &mountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the B2S Mounting Matrix
    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(icm_20948, B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult)
        worstResult = result;
    result = writeDMPmems(icm_20948, B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the DMP Gyro Scaling Factor
    // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
    //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
    //            10=102.2727Hz sample rate, ... etc.
    // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
    result = setGyroSF(icm_20948, 19, 3);
    if (result > worstResult)
        worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

    // Configure the Gyro full scale
    // 2000dps : 2^28
    // 1000dps : 2^27
    //  500dps : 2^26
    //  250dps : 2^25
    const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
    result = writeDMPmems(icm_20948, GYRO_FULLSCALE, 4, &gyroFullScale[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
    const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
    //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // InvenSense Nucleo example uses 225Hz
    result = writeDMPmems(icm_20948, ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
    const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
    //const unsigned char accelAlphaVar[4] = {0x06, 0x66, 0x66, 0x66}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(icm_20948, ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
    const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
    //const unsigned char accelAVar[4] = {0x39, 0x99, 0x99, 0x9A}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(icm_20948, ACCEL_A_VAR, 4, &accelAVar[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the Accel Cal Rate
    const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(icm_20948, ACCEL_CAL_RATE, 2, &accelCalRate[0]);
    if (result > worstResult)
        worstResult = result;

    // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
    // Let's set the Compass Time Buffer to 69 (Hz).
    const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
    result = writeDMPmems(icm_20948, CPASS_TIME_BUFFER, 2, &compassRate[0]);
    if (result > worstResult)
        worstResult = result;

        // Enable DMP interrupt
        // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
        //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

#endif

    return worstResult;
}
