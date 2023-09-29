/*
* ________________________________________________________________________________________________________
* Copyright © 2014 InvenSense Inc.  All rights reserved.
*
* This software and/or documentation  (collectively “Software”) is subject to InvenSense intellectual property rights
* under U.S. and international copyright and other intellectual property rights laws.
*
* The Software contained herein is PROPRIETARY and CONFIDENTIAL to InvenSense and is provided
* solely under the terms and conditions of a form of InvenSense software license agreement between
* InvenSense and you and any use, modification, reproduction or disclosure of the Software without
* such agreement or the express written consent of InvenSense is strictly prohibited.
*
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
* PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
* TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
* INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
* DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
* NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
* OF THE SOFTWARE.
* ________________________________________________________________________________________________________
*/

#ifndef _DMP_3_DRIVER_H__
#define _DMP_3_DRIVER_H__

#include "int_types.h"
//#include "../mltypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* enum for sensor
   The sequence is important.
   It represents the order of apperance from DMP */
enum INV_SENSORS
{
    INV_SENSOR_ACCEL = 0,
    INV_SENSOR_GYRO,
    INV_SENSOR_LPQ,             // 20610:  we'll find out if it breaks 20628 being inserted here....
    INV_SENSOR_COMPASS,
    INV_SENSOR_ALS,
    INV_SENSOR_SIXQ,
    INV_SENSOR_NINEQ,
    INV_SENSOR_GEOMAG,
    INV_SENSOR_PEDQ,
    INV_SENSOR_PRESSURE,
    INV_SENSOR_CALIB_GYRO,
    INV_SENSOR_CALIB_COMPASS,
    INV_SENSOR_STEP_COUNTER,
    INV_SENSOR_ACTIVITY_CLASSIFIER,
    INV_SENSOR_FLIP_PICKUP,
    INV_SENSOR_BRING_TO_SEE,

    INV_SENSOR_SIXQ_accel,
    INV_SENSOR_NINEQ_accel,
    INV_SENSOR_GEOMAG_cpass,
    INV_SENSOR_NINEQ_cpass,

    INV_SENSOR_WAKEUP_ACCEL,
    INV_SENSOR_WAKEUP_GYRO,
//INV_SENSOR_WAKEUP_LPQ,
    INV_SENSOR_WAKEUP_COMPASS,
    INV_SENSOR_WAKEUP_ALS,
    INV_SENSOR_WAKEUP_SIXQ,
    INV_SENSOR_WAKEUP_NINEQ,
    INV_SENSOR_WAKEUP_GEOMAG,
    INV_SENSOR_WAKEUP_PEDQ,
    INV_SENSOR_WAKEUP_PRESSURE,
    INV_SENSOR_WAKEUP_CALIB_GYRO,
    INV_SENSOR_WAKEUP_CALIB_COMPASS,
    INV_SENSOR_WAKEUP_STEP_COUNTER,
    INV_SENSOR_WAKEUP_TILT_DETECTOR,
//INV_SENSOR_WAKEUP_ACTIVITY_CLASSIFIER,

    INV_SENSOR_WAKEUP_SIXQ_accel,
    INV_SENSOR_WAKEUP_NINEQ_accel,
    INV_SENSOR_WAKEUP_GEOMAG_cpass,
    INV_SENSOR_WAKEUP_NINEQ_cpass,

    INV_SENSOR_NUM_MAX,
    INV_SENSOR_INVALID,
};


enum accel_cal_params
{
    ACCEL_CAL_ALPHA_VAR = 0,
    ACCEL_CAL_A_VAR,
    ACCEL_CAL_DIV,
    NUM_ACCEL_CAL_PARAMS
};

enum compass_cal_params
{
    CPASS_CAL_TIME_BUFFER = 0,
    CPASS_CAL_RADIUS_3D_THRESH_ANOMALY,
    NUM_CPASS_CAL_PARAMS
};
extern int inv_dmpdriver_write_mems(unsigned short reg, unsigned int length, const unsigned char *data);
extern int inv_dmpdriver_read_mems(unsigned short reg, unsigned int length, unsigned char *data);
extern int inv_dmpdriver_mems_firmware_load(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr);
extern unsigned char *inv_dmpdriver_int16_to_big8(short x, unsigned char *big8);
extern unsigned char *inv_dmpdriver_int32_to_big8(long x, unsigned char *big8);
extern long inv_dmpdriver_big8_to_int32(const unsigned char *big8);

#ifdef __cplusplus
}
#endif

// _DMP_3_DRIVER_H__
#endif
