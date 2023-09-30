/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#include "inv_mems_augmented_sensors.h"

#ifndef MEMS_20609
    #include "inv_mems_base_control.h"
#else
    #include "inv_mems_base_control_20609.h"
#endif

#include "mlmath.h"
#include "ml_math_func.h"

// ODR expected for gravity-based sensors
static unsigned short sGravityOdrMs = 0xFFFF;
static unsigned short sGrvOdrMs = 0xFFFF;
static unsigned short sLinAccOdrMs = 0xFFFF;
static unsigned short sGravityWuOdrMs = 0xFFFF;
static unsigned short sGrvWuOdrMs = 0xFFFF;
static unsigned short sLinAccWuOdrMs = 0xFFFF;

// ODR expected for rotation vector-based sensors
static unsigned short sRvOdrMs = 0xFFFF;
static unsigned short sOriOdrMs = 0xFFFF;
static unsigned short sRvWuOdrMs = 0xFFFF;
static unsigned short sOriWuOdrMs = 0xFFFF;

// Determine the fastest ODR for all gravity-based sensors
#define AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(newOdr) \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_GRAVITY)) \
        newOdr = MIN(sGravityOdrMs,newOdr); \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_GAME_ROTATION_VECTOR)) \
        newOdr = MIN(sGrvOdrMs,newOdr);  \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_LINEAR_ACCELERATION)) \
        newOdr = MIN(sLinAccOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(newOdr) \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_WAKEUP_GRAVITY)) \
        newOdr = MIN(sGravityWuOdrMs,newOdr); \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR)) \
        newOdr = MIN(sGrvWuOdrMs,newOdr);  \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION)) \
        newOdr = MIN(sLinAccWuOdrMs,newOdr);

// Determine the fastest ODR for all rotation vector-based sensors
#define AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(newOdr) \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_ORIENTATION)) \
        newOdr = MIN(sOriOdrMs,newOdr); \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_ROTATION_VECTOR)) \
        newOdr = MIN(sRvOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(newOdr) \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_WAKEUP_ORIENTATION)) \
        newOdr = MIN(sOriWuOdrMs,newOdr); \
    if	(inv_androidSensor_enabled	(ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)) \
        newOdr = MIN(sRvWuOdrMs,newOdr);

inv_error_t inv_mems_augmented_sensors_get_gravity(long gravity[3], const long quat6axis_3e[3])
{
    long lQuat6axis4e[4];
    unsigned long lConvertgToMS2 = 642689UL; //9.80665f in Q16

    if(!gravity) return -1;

    if(!quat6axis_3e) return -1;

    // compute w element
    inv_compute_scalar_part(quat6axis_3e, lQuat6axis4e);

    gravity[0] = (2 * inv_qfix_mult(lQuat6axis4e[1], lQuat6axis4e[3], 30) - 2 * inv_qfix_mult(lQuat6axis4e[0], lQuat6axis4e[2], 30)) >> (30 - 16);
    gravity[1] = (2 * inv_qfix_mult(lQuat6axis4e[2], lQuat6axis4e[3], 30) + 2 * inv_qfix_mult(lQuat6axis4e[0], lQuat6axis4e[1], 30)) >> (30 - 16);
    gravity[2] = ((1 << 30) - 2 * inv_qfix_mult(lQuat6axis4e[1], lQuat6axis4e[1], 30) - 2 * inv_qfix_mult(lQuat6axis4e[2], lQuat6axis4e[2], 30)) >> (30 - 16);

    // convert to m/s2
    gravity[0] = inv_qfix_mult(gravity[0], lConvertgToMS2, 16);
    gravity[1] = inv_qfix_mult(gravity[1], lConvertgToMS2, 16);
    gravity[2] = inv_qfix_mult(gravity[2], lConvertgToMS2, 16);

    return MPU_SUCCESS;
}

inv_error_t inv_mems_augmented_sensors_get_linearacceleration(long linacc[3], const long gravity[3], const long accel[3])
{
    if(!linacc) return -1;

    if(!gravity) return -1;

    if(!accel) return -1;

    linacc[0] = accel[0] - gravity[0];
    linacc[1] = accel[1] - gravity[1];
    linacc[2] = accel[2] - gravity[2];

    return MPU_SUCCESS;
}


inv_error_t inv_mems_augmented_sensors_get_orientation(long orientation[3], const long quat9axis_3e[4])
{
    long lQuat9axis4e[4];
    long lMatrixQ30[9];
    long lMatrixQ30Square;
    long lRad2degQ16 = 0x394BB8; // (float)(180.0 / 3.14159265358979) in Q16

    if(!orientation) return -1;

    if(!quat9axis_3e) return -1;

    // compute w element
    inv_compute_scalar_part(quat9axis_3e, lQuat9axis4e);

    // quaternion to a rotation matrix, q30 to q30
    inv_transpose_quaternion_to_rotation((const long *)lQuat9axis4e, (long *)lMatrixQ30);

    // compute orientation in q16
    // orientationFlt[0] = atan2f(-matrixFlt[1][0], matrixFlt[0][0]) * rad2deg;
    orientation[0] = atan2_q15(-lMatrixQ30[3] >> 15, lMatrixQ30[0] >> 15) << 1;
    orientation[0] = inv_qfix_mult(orientation[0], lRad2degQ16, 16);

    // orientationFlt[1] = atan2f(-matrixFlt[2][1], matrixFlt[2][2]) * rad2deg;
    orientation[1] = atan2_q15(-lMatrixQ30[7] >> 15, lMatrixQ30[8] >> 15) << 1;
    orientation[1] = inv_qfix_mult(orientation[1], lRad2degQ16, 16);

    // orientationFlt[2] = asinf ( matrixFlt[2][0]) * rad2deg;
    // asin(x) = atan (x/sqrt(1-x²))
    // atan2(y,x) = atan(y/x)
    // asin(x) = atan2(x, sqrt(1-x²))
    lMatrixQ30Square = inv_qfix_mult(lMatrixQ30[6], lMatrixQ30[6], 30); // x²
    lMatrixQ30Square = (1UL << 30) - lMatrixQ30Square; // 1-x²
    lMatrixQ30Square = inv_fast_sqrt(lMatrixQ30Square); // sqrt(1-x²)
    orientation[2] = atan2_q15(lMatrixQ30[6] >> 15,  lMatrixQ30Square >> 15) << 1; // atan2(x, sqrt(1-x²))
    orientation[2] = inv_qfix_mult(orientation[2], lRad2degQ16, 16); // * rad2deg

    if (orientation[0] < 0)
        orientation[0] += 360UL << 16;

    return MPU_SUCCESS;
}

unsigned short inv_mems_augmented_sensors_set_odr(unsigned char androidSensor, unsigned short delayInMs)
{
    switch(androidSensor)
    {
        case ANDROID_SENSOR_GRAVITY:
            sGravityOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
            sGrvOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_LINEAR_ACCELERATION:
            sLinAccOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_ORIENTATION:
            sOriOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_ROTATION_VECTOR:
            sRvOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_WAKEUP_GRAVITY:
            sGravityWuOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
            sGrvWuOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
            sLinAccWuOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_WAKEUP_ORIENTATION:
            sOriWuOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(delayInMs);
            break;

        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
            sRvWuOdrMs = delayInMs;
            AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(delayInMs);
            break;

        default :
            break;
    }

    return delayInMs;
}


void inv_mems_augmented_sensors_update_odr(unsigned char androidSensor, unsigned short * updatedDelayPtr)
{
    unsigned short lDelayInMs = 0xFFFF; // max value of uint16_t, so that we can get min value of all enabled sensors

    switch(androidSensor)
    {
        case ANDROID_SENSOR_GRAVITY:
        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
            AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(lDelayInMs);
            *updatedDelayPtr = lDelayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_GRAVITY:
        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
            AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(lDelayInMs);
            *updatedDelayPtr = lDelayInMs;
            break;

        case ANDROID_SENSOR_ORIENTATION:
        case ANDROID_SENSOR_ROTATION_VECTOR:
            AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(lDelayInMs);
            *updatedDelayPtr = lDelayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_ORIENTATION:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
            AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(lDelayInMs);
            *updatedDelayPtr = lDelayInMs;
            break;

        default :
            break;
    }

}
