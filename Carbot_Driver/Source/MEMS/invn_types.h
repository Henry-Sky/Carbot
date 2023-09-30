//----------------------------------------------------------------------------- 
/*
    Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively “Software”) is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
    and other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license agreement
    from InvenSense is strictly prohibited.
*/
//-----------------------------------------------------------------------------

#ifndef INVN_COMMON_INVN_TYPES_H_
#define INVN_COMMON_INVN_TYPES_H_

#include "int_types.h"

/**
 *  @defgroup invn_types
 *  @brief  Motion Library - Type definitions.
 *          Definition of codes and error codes used within the MPL and
 *          returned to the user.
 *          Every function tries to return a meaningful error code basing
 *          on the occuring error condition. The error code is numeric.
 *
 *          The available error codes and their associated values are:
 *          - (0)               INV_SUCCESS
 *          - (32)              INV_ERROR
 *          - (22 / EINVAL)     INV_ERROR_INVALID_PARAMETER
 *          - (1  / EPERM)      INV_ERROR_FEATURE_NOT_ENABLED
 *          - (36)              INV_ERROR_FEATURE_NOT_IMPLEMENTED
 *          - (64)              INV_ERROR_FIFO_READ_COUNT
 *
 *  @{
 *      @file invn_types.h
 *  @}
 */


#ifndef REMOVE_INV_ERROR_T
//typedef int inv_error_t;
#endif

//typedef int mpu_error_t;
typedef long long mpu_time_t;

// Typically I2C addresses are 8-bit, but some specifications allow for a 10-bit address
// This definition allows the length to be optimally defined for the platform
typedef unsigned char inv_i2c_addr_t;

#ifdef __IAR_SYSTEMS_ICC__
// These are defined in standard C errno.h
#define EINVAL                                  (22)
#define EPERM                                   (1)
#define ENOMEM                                  (12)
#else
#include "errno.h"
#endif

#define INV_SUCCESS								(0)
#define INV_ERROR_BASE							(0x20)
#define INV_ERROR								(INV_ERROR_BASE) 
#define INV_ERROR_FEATURE_NOT_ENABLED           (EPERM)
#define INV_ERROR_FEATURE_NOT_IMPLEMENTED       (INV_ERROR_BASE + 4)
#define INV_ERROR_INVALID_PARAMETER             (EINVAL)
#define INV_ERROR_FILE_OPEN                     (INV_ERROR_BASE + 14)
#define INV_ERROR_FILE_READ                     (INV_ERROR_BASE + 15)
#define INV_ERROR_FILE_WRITE                    (INV_ERROR_BASE + 16)
#define INV_ERROR_INVALID_CONFIGURATION         (INV_ERROR_BASE + 17)
/* Serial Communication */
#define INV_ERROR_SERIAL_OPEN_ERROR             (INV_ERROR_BASE + 21)
#define INV_ERROR_SERIAL_READ                   (INV_ERROR_BASE + 22)
#define INV_ERROR_SERIAL_WRITE                  (INV_ERROR_BASE + 23)
/* Fifo */
#define INV_ERROR_FIFO_OVERFLOW                 (INV_ERROR_BASE + 30)
#define INV_ERROR_FIFO_FOOTER                   (INV_ERROR_BASE + 31)
#define INV_ERROR_FIFO_READ_COUNT               (INV_ERROR_BASE + 32)
#define INV_ERROR_FIFO_READ_DATA                (INV_ERROR_BASE + 33)

/* OS interface errors */
#define INV_ERROR_OS_BAD_HANDLE                 (INV_ERROR_BASE + 61)
#define INV_ERROR_OS_CREATE_FAILED              (INV_ERROR_BASE + 62)
#define INV_ERROR_OS_LOCK_FAILED                (INV_ERROR_BASE + 63)


#define INV_WARNING_SEMAPHORE_TIMEOUT           (INV_ERROR_BASE + 86)

#endif // INVN_COMMON_INVN_TYPES_H_
