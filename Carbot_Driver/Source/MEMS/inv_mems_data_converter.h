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
/** @defgroup mems_data_converter data_converter
	@ingroup  Mems_driver
	@{
*/
#ifndef INV_MEMS_DATA_CONVERTER_H__
#define INV_MEMS_DATA_CONVERTER_H__


#ifdef __cplusplus
extern "C"
{
#endif

#include "ml_math_func.h"

/** @brief Sets the transformation used for chip to body frame
* @param[in] quat 	the quaternion used for the transformation
*/
void inv_set_chip_to_body(long *quat);

/** @brief Converts fixed point DMP rotation vector to floating point android notation
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion in Android format
*/
void inv_convert_rotation_vector(const long *quat, float *values);

/** @brief Converts 3 element fixed point DMP rotation vector to 4 element rotation vector in world frame
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] quat4_world 4 element quaternion
*/

void inv_convert_rotation_vector_1(const long *quat, long *values);

void inv_convert_rotation_vector_2(const long *quat, long *quat4_world);

/** @brief Converts 4 element rotation vector in world frame to floating point android notation
* @param[in] quat4_world 4 element rotation vector in World frame
* @param[out] values in Android format
*/
void inv_convert_rotation_vector_3(const long *quat4_world, float *values);

/** @brief Converts the data in android values
* @param[in] vec3 vector of the DMP
* @param[in] scale scale calculated
* @param[out] values in Android format
*/
void inv_convert_dmp3_to_body(const long *vec3, float scale, float *values);

/** @brief Converts the data in android quaternion values
* @param[in] accel_gyro_matrix 	vector of the DMP
* @param[out] angle 			angle calculated
*/
void inv_set_chip_to_body_axis_quaternion(signed char *accel_gyro_matrix, float angle);

/** @brief Converts a 32-bit long to a little endian byte stream
* @param[in] x 				the long to be converted
* @param[in] little8 		little endian byte converted
* @return 					0 on success, negative value on error.
*/
unsigned char *inv_int32_to_little8(long x, unsigned char *little8);
#ifdef __cplusplus
}
#endif
#endif	/* INV_MEMS_DATA_CONVERTER_H__ */

/** @} */
