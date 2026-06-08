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


#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "ICM_20948_CONTROL.h" // The C backbone. ICM_20948_USE_DMP is defined in here.
#include "AK09916_REGISTERS.h"

typedef struct _ICM_20948
{
    ICM_20948_Device_t _device;
    ICM_20948_AGMT_t agmt;     // Acceleometer, Gyroscope, Magenetometer, and Temperature data
    ICM_20948_Status_e status; // Status from latest operation
    uint8_t _addr;
} ICM_20948;

ICM_20948_Status_e icm20948_begin(ICM_20948 *);

ICM_20948_AGMT_t getAGMT(ICM_20948 *icm_20948); // Updates the agmt field in the object and also returns a copy directly

double magX(ICM_20948 *icm_20948); // micro teslas
double magY(ICM_20948 *icm_20948); // micro teslas
double magZ(ICM_20948 *icm_20948); // micro teslas

double accX(ICM_20948 *icm_20948); // milli g's
double accY(ICM_20948 *icm_20948); // milli g's
double accZ(ICM_20948 *icm_20948); // milli g's

double gyrX(ICM_20948 *icm_20948); // degrees per second
double gyrY(ICM_20948 *icm_20948); // degrees per second
double gyrZ(ICM_20948 *icm_20948); // degrees per second

double temperature(ICM_20948 *icm_20948); // degrees celsius

const char *statusString(ICM_20948 *icm_20948, ICM_20948_Status_e stat); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

// Device Level
ICM_20948_Status_e setBank(ICM_20948 *icm_20948, uint8_t bank);                                // Sets the bank
ICM_20948_Status_e swReset(ICM_20948 *icm_20948);                                              // Performs a SW reset
ICM_20948_Status_e deviceSleep(ICM_20948 *icm_20948, bool on);                                 // Set sleep mode for the chip
ICM_20948_Status_e lowPower(ICM_20948 *icm_20948, bool on);                                    // Set low power mode for the chip
ICM_20948_Status_e setClockSource(ICM_20948 *icm_20948, ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
ICM_20948_Status_e checkID(ICM_20948 *icm_20948);                                              // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

bool dataReady(ICM_20948 *icm_20948);    // Returns 'true' if data is ready
uint8_t getWhoAmI(ICM_20948 *icm_20948); // Return whoami in out prarmeter
bool isConnected(ICM_20948 *icm_20948);  // Returns true if communications with the device are sucessful

// Internal Sensor Options
ICM_20948_Status_e setSampleMode(ICM_20948 *icm_20948, uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
ICM_20948_Status_e setFullScale(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_fss_t fss);
ICM_20948_Status_e setDLPFcfg(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
ICM_20948_Status_e enableDLPF(ICM_20948 *icm_20948, uint8_t sensor_id_bm, bool enable);
ICM_20948_Status_e setSampleRate(ICM_20948 *icm_20948, uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

// Interrupts on INT and FSYNC Pins
ICM_20948_Status_e clearInterrupts(ICM_20948 *icm_20948);

ICM_20948_Status_e cfgIntActiveLow(ICM_20948 *icm_20948, bool active_low);
ICM_20948_Status_e cfgIntOpenDrain(ICM_20948 *icm_20948, bool open_drain);
ICM_20948_Status_e cfgIntLatch(ICM_20948 *icm_20948, bool latching);         // If not latching then the interrupt is a 50 us pulse
ICM_20948_Status_e cfgIntAnyReadToClear(ICM_20948 *icm_20948, bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
ICM_20948_Status_e cfgFsyncActiveLow(ICM_20948 *icm_20948, bool active_low);
ICM_20948_Status_e cfgFsyncIntMode(ICM_20948 *icm_20948, bool interrupt_mode); // Can use FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

ICM_20948_Status_e intEnableI2C(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnableDMP(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnablePLL(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnableWOM(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnableWOF(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnableRawDataReady(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e intEnableOverflowFIFO(ICM_20948 *icm_20948, uint8_t bm_enable);
ICM_20948_Status_e intEnableWatermarkFIFO(ICM_20948 *icm_20948, uint8_t bm_enable);

ICM_20948_Status_e WOMThreshold(ICM_20948 *icm_20948, uint8_t threshold);

// Interface Options
ICM_20948_Status_e i2cMasterPassthrough(ICM_20948 *icm_20948, bool passthrough);
ICM_20948_Status_e i2cMasterEnable(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e i2cMasterReset(ICM_20948 *icm_20948);

//Used for configuring peripherals 0-3
ICM_20948_Status_e i2cControllerConfigurePeripheral(ICM_20948 *icm_20948, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t data_out);
ICM_20948_Status_e i2cControllerPeriph4Transaction(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);

//Used for configuring the Magnetometer
ICM_20948_Status_e i2cMasterSingleW(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg, uint8_t data);
uint8_t i2cMasterSingleR(ICM_20948 *icm_20948, uint8_t addr, uint8_t reg);

// Default Setup
ICM_20948_Status_e startupDefault(ICM_20948 *icm_20948, bool minimal); // If minimal is true, several startup steps are skipped. If ICM_20948_USE_DMP is defined, .begin will call startupDefault with minimal set to true.

// direct read/write
ICM_20948_Status_e directRead(ICM_20948 *icm_20948, uint8_t reg, uint8_t *pdata, uint32_t len);
ICM_20948_Status_e directWrite(ICM_20948 *icm_20948, uint8_t reg, uint8_t *pdata, uint32_t len);

//Mag specific
ICM_20948_Status_e startupMagnetometer(ICM_20948 *icm_20948, bool minimal);
ICM_20948_Status_e magWhoIAm(ICM_20948 *icm_20948);
uint8_t readMag(ICM_20948 *icm_20948, AK09916_Reg_Addr_e reg);
ICM_20948_Status_e writeMag(ICM_20948 *icm_20948, AK09916_Reg_Addr_e reg, uint8_t *pdata);
ICM_20948_Status_e resetMag(ICM_20948 *icm_20948);

//FIFO
ICM_20948_Status_e enableFIFO(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e resetFIFO(ICM_20948 *icm_20948);
ICM_20948_Status_e setFIFOmode(ICM_20948 *icm_20948, bool snapshot); // Default to Stream (ICM_20948 *icm_20948, non-Snapshot) mode
ICM_20948_Status_e getFIFOcount(ICM_20948 *icm_20948, uint16_t *count);
ICM_20948_Status_e readFIFO(ICM_20948 *icm_20948, uint8_t *data, uint8_t len);

//DMP

// Done:
//  Configure DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
//  Load Firmware
//  Configure Accel scaling to DMP
//  Configure Compass mount matrix and scale to DMP
//  Reset FIFO
//  Reset DMP
//  Enable DMP interrupt
//  Configuring DMP to output data to FIFO: set DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_INTR_CTL and MOTION_EVENT_CTL
//  Configuring DMP to output data at multiple ODRs
//  Configure DATA_RDY_STATUS
//  Configuring Accel calibration
//  Configuring Compass calibration
//  Configuring Gyro gain
//  Configuring Accel gain

// To Do:
//  Additional FIFO output control: FIFO_WATERMARK, BM_BATCH_MASK, BM_BATCH_CNTR, BM_BATCH_THLD
//  Configuring DMP features: PED_STD_STEPCTR, PED_STD_TIMECTR
//  Enabling Activity Recognition (BAC) feature
//  Enabling Significant Motion Detect (SMD) feature
//  Enabling Tilt Detector feature
//  Enabling Pick Up Gesture feature
//  Enabling Fsync detection feature
//  Biases?

ICM_20948_Status_e enableDMP(ICM_20948 *icm_20948, bool enable);
ICM_20948_Status_e resetDMP(ICM_20948 *icm_20948);
ICM_20948_Status_e loadDMPFirmware(ICM_20948 *icm_20948);
ICM_20948_Status_e setDMPstartAddress(ICM_20948 *icm_20948, unsigned short address);
ICM_20948_Status_e enableDMPSensor(ICM_20948 *icm_20948, enum inv_icm20948_sensor sensor, bool enable);
ICM_20948_Status_e enableDMPSensorInt(ICM_20948 *icm_20948, enum inv_icm20948_sensor sensor, bool enable);
ICM_20948_Status_e writeDMPmems(ICM_20948 *icm_20948, unsigned short reg, unsigned int length, const unsigned char *data);
ICM_20948_Status_e readDMPmems(ICM_20948 *icm_20948, unsigned short reg, unsigned int length, unsigned char *data);
ICM_20948_Status_e setDMPODRrate(ICM_20948 *icm_20948, enum DMP_ODR_Registers odr_reg, int interval);
ICM_20948_Status_e readDMPdataFromFIFO(ICM_20948 *icm_20948, icm_20948_DMP_data_t *data);
ICM_20948_Status_e setGyroSF(ICM_20948 *icm_20948, unsigned char div, int gyro_level);
ICM_20948_Status_e initializeDMP(ICM_20948 *icm_20948);

#endif /* _ICM_20948_H_ */
