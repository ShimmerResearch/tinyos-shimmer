/*
 * Copyright (c) 2010, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Shimmer Research, Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date   November, 2010
 */

#ifndef SHIMMER_H
#define SHIMMER_H

// Maximum number of channels
enum {
   MAX_NUM_2_BYTE_CHANNELS = 11,
   MAX_NUM_1_BYTE_CHANNELS = 1,
   MAX_NUM_CHANNELS = MAX_NUM_2_BYTE_CHANNELS + MAX_NUM_1_BYTE_CHANNELS
};

// Channel contents
enum {
   X_ACCEL     = 0x00,
   Y_ACCEL     = 0x01,
   Z_ACCEL     = 0x02,
   X_GYRO      = 0x03,
   Y_GYRO      = 0x04,
   Z_GYRO      = 0x05,
   X_MAG       = 0x06,
   Y_MAG       = 0x07,
   Z_MAG       = 0x08,
   ECG_RA_LL   = 0x09,
   ECG_LA_LL   = 0x0A,
   GSR_RAW     = 0x0B,
   GSR_RES     = 0x0C,     // GSR resistance (not used in this app)
   EMG         = 0x0D,
   ANEX_A0     = 0x0E,
   ANEX_A7     = 0x0F,
   STRAIN_HIGH = 0x10,
   STRAIN_LOW  = 0x11, 
   HEART_RATE  = 0x12
};

// SD write size
enum{
    NV_SD_WRITE_SIZE = 512
};

// Config header contents;
enum {
   NV_NUM_CONFIG_BYTES = 136
};

enum {
   NV_SENSORS0      = 0,
   NV_SENSORS1      = 1,
   NV_SENSORS2      = 2,
   NV_SENSORS3      = 3,
   NV_SENSORS4      = 4,
   NV_SENSORS5      = 5,
   NV_SENSORS6      = 6,
   NV_SENSORS7      = 7,
   NV_SENSORS8      = 8,
   NV_SENSORS9      = 9,
   NV_CONFIG_SETUP0 = 10,
   NV_CONFIG_SETUP1 = 11,
   NV_CONFIG_SETUP2 = 12,
   NV_CONFIG_SETUP3 = 13,
   NV_CONFIG_SETUP4 = 14,
   NV_CONFIG_SETUP5 = 15,
   NV_CONFIG_SETUP6 = 16,
   NV_CONFIG_SETUP7 = 17,
   NV_CONFIG_SETUP8 = 18,
   NV_CONFIG_SETUP9 = 19,
   NV_SENSORCONFIG0 = 20,
   NV_SENSORCONFIG1 = 21,
   NV_SENSORCONFIG2 = 22,
   NV_SENSORCONFIG3 = 23,
   NV_SENSORCONFIG4 = 24,
   NV_SENSORCONFIG5 = 25,
   NV_SENSORCONFIG6 = 26,
   NV_SENSORCONFIG7 = 27,
   NV_SENSORCONFIG8 = 28,
   NV_SENSORCONFIG9 = 29,
   NV_TRIAL_SETUP0  = 30,
   NV_TRIAL_SETUP1  = 31,
   NV_TRIAL_SETUP2  = 32,
   NV_TRIAL_SETUP3  = 33,
   NV_TRIAL_SETUP4  = 34,
   NV_TRIAL_SETUP5  = 35,
   NV_TRIAL_SETUP6  = 36,
   NV_TRIAL_SETUP7  = 37,
   NV_TRIAL_SETUP8  = 38,
   NV_TRIAL_SETUP9  = 39,
   NV_HOST_TIME_0   = 40,
   NV_HOST_TIME_1   = 41,
   NV_HOST_TIME_2   = 42,
   NV_HOST_TIME_3   = 43,
   NV_HOST_TIME_4   = 44,
   NV_HOST_TIME_5   = 45,
   NV_HOST_TIME_6   = 46,
   NV_HOST_TIME_7   = 47,
   NV_MY_TIME_0   = 48,
   NV_MY_TIME_1   = 49,
   NV_MY_TIME_2   = 50,
   NV_MY_TIME_3   = 51,
   NV_CONFIG_TIME = 52,
   NV_ACCEL_CALIBRATION = 56,
   NV_GYRO_CALIBRATION = 77,
   NV_MAG_CALIBRATION = 98,
   NV_ECG_CALIBRATION = 119,
   NV_EMG_CALIBRATION = 127
};

enum{
   NV_SYNC_INTERVAL      = NV_CONFIG_SETUP5,
   NV_SAMPLING_RATE      = NV_SENSORCONFIG0,
   NV_ACCEL_RANGE        = NV_SENSORCONFIG2,
   NV_GSR_RANGE          = NV_SENSORCONFIG3,
   NV_MAG_RANGE          = NV_SENSORCONFIG4,
   NV_MAG_RATE           = NV_SENSORCONFIG5,
   NV_FW_VER_BYTE0       = NV_TRIAL_SETUP4,
   NV_SHIMMER_VER_BYTE0  = NV_TRIAL_SETUP0,
   NV_MYTRIALID          = NV_TRIAL_SETUP2,
   NV_NUMSHIMMERS        = NV_TRIAL_SETUP3
};

enum{
    S_ACCEL = 0,
    S_GYRO = 1,
    S_MAG = 2,
    S_ECG = 3,
    S_EMG = 4
};

//Sensor bitmap
//SENSORS0
enum {
   SENSOR_ACCEL   = 0x80,
   SENSOR_GYRO    = 0x40,
   SENSOR_MAG     = 0x20,
   SENSOR_ECG     = 0x10,
   SENSOR_EMG     = 0x08,
   SENSOR_GSR     = 0x04,
   SENSOR_ANEX_A7 = 0x02,
   SENSOR_ANEX_A0 = 0x01
};
//SENSORS1
enum {
   SENSOR_STRAIN  = 0x80,
   SENSOR_HEART   = 0x40
};

// Config Byte0 bitmap
enum {
   CONFIG_5V_REG        = 0x80,
   CONFIG_PMUX          = 0x40,
};

// BoilerPlate specific extension to range values
enum {
   GSR_AUTORANGE  = 0x04,
   GSR_X4         = 0x05
};

// Magnetometer specific range and rate values
enum {
   RANGE_0_7GA    = 0,
   RANGE_1_0GA    = 1,
   RANGE_1_5GA    = 2,
   RANGE_2_0GA    = 3,
   RANGE_3_2GA    = 4,
   RANGE_3_8GA    = 5,
   RANGE_4_5GA    = 6
};

enum {
   RATE_0_5Hz    = 0,
   RATE_1Hz      = 1,
   RATE_2Hz      = 2,
   RATE_5Hz      = 3,
   RATE_10Hz     = 4,
   RATE_20Hz     = 5,
   RATE_50Hz     = 6
};

#endif // SHIMMER_H
