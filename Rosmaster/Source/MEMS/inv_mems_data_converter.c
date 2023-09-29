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

#include "inv_mems_data_converter.h"

#include <string.h>

#include "mlmath.h"

#define INV_TWO_POWER_NEG_30 9.313225746154785e-010f

static long s_quat_chip_to_body[4] = {(1L << 30), 0, 0, 0};

/** Performs a fixed point quaternion multiply with inverse on second element q1*q2'.
* @param[in] q1 First Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30
* @param[in] q2 Second Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30. Inverse will be take before multiply
* @param[out] qProd Product after quaternion multiply q1*q2'. Length 4.
*             1.0 scaled to 2^30.
*/
static void inv_q_mult_q_qi(const long *q1, const long *q2, long *qProd)
{
    qProd[0] = inv_q30_mult(q1[0], q2[0]) + inv_q30_mult(q1[1], q2[1]) +
               inv_q30_mult(q1[2], q2[2]) + inv_q30_mult(q1[3], q2[3]);

    qProd[1] = -inv_q30_mult(q1[0], q2[1]) + inv_q30_mult(q1[1], q2[0]) -
               inv_q30_mult(q1[2], q2[3]) + inv_q30_mult(q1[3], q2[2]);

    qProd[2] = -inv_q30_mult(q1[0], q2[2]) + inv_q30_mult(q1[1], q2[3]) +
               inv_q30_mult(q1[2], q2[0]) - inv_q30_mult(q1[3], q2[1]);

    qProd[3] = -inv_q30_mult(q1[0], q2[3]) - inv_q30_mult(q1[1], q2[2]) +
               inv_q30_mult(q1[2], q2[1]) + inv_q30_mult(q1[3], q2[0]);
}

/** Set the transformation used for chip to body frame
*/
void inv_set_chip_to_body(long *quat)
{
    memcpy(s_quat_chip_to_body, quat, sizeof(s_quat_chip_to_body));
}

/** Convert fixed point DMP rotation vector to floating point android notation
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion in Android format
*/
void inv_convert_rotation_vector(const long *quat, float *values)
{
    long quat4[4];
    long quat_body_to_world[4];

    inv_compute_scalar_part(quat, quat4);
    inv_q_mult_q_qi(quat4, s_quat_chip_to_body, quat_body_to_world);

    if (quat_body_to_world[0] >= 0)
    {
        values[0] = quat_body_to_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat_body_to_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat_body_to_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat_body_to_world[0] * INV_TWO_POWER_NEG_30;
    }
    else
    {
        values[0] = -quat_body_to_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat_body_to_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat_body_to_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat_body_to_world[0] * INV_TWO_POWER_NEG_30;
    }
}

/** Convert fixed point DMP rotation vector to fixed point android notation
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion in Android format
*/
void inv_convert_rotation_vector_1(const long *quat, long *values)
{
    long quat4[4];
    long quat_body_to_world[4];

    inv_compute_scalar_part(quat, quat4);
    inv_q_mult_q_qi(quat4, s_quat_chip_to_body, quat_body_to_world);

    if (quat_body_to_world[0] >= 0)
    {
        values[0] = quat_body_to_world[1];
        values[1] = quat_body_to_world[2];
        values[2] = quat_body_to_world[3];
        values[3] = quat_body_to_world[0];
    }
    else
    {
        values[0] = -quat_body_to_world[1];
        values[1] = -quat_body_to_world[2];
        values[2] = -quat_body_to_world[3];
        values[3] = -quat_body_to_world[0];
    }
}

/** Convert 3 element fixed point DMP rotation vector to 4 element rotation vector in world frame
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion
*/
void inv_convert_rotation_vector_2(const long *quat, long *quat4_world)
{
    long quat4[4];
    long quat_body_to_world[4];

    inv_compute_scalar_part(quat, quat4);
    inv_q_mult_q_qi(quat4, s_quat_chip_to_body, quat_body_to_world);
    memcpy(quat4_world, quat_body_to_world, 4 * sizeof(long));
}

/** Convert 4 element rotation vector in world frame to floating point android notation
* @param[in] quat 4 element rotation vector in World frame
* @param[out] values in Android format
*/
void inv_convert_rotation_vector_3(const long *quat4_world, float *values)
{
    if (quat4_world[0] >= 0)
    {
        values[0] = quat4_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat4_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat4_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat4_world[0] * INV_TWO_POWER_NEG_30;
    }
    else
    {
        values[0] = -quat4_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat4_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat4_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat4_world[0] * INV_TWO_POWER_NEG_30;
    }
}

void inv_set_chip_to_body_axis_quaternion(signed char *accel_gyro_matrix, float angle)
{
    int i;
    float rot[9];
    long qcb[4], q_all[4];
    long q_adjust[4];

    for (i = 0; i < 9; i++)
    {
        rot[i] = (float)accel_gyro_matrix[i];
    }

    // Convert Chip to Body transformation matrix to quaternion
    inv_rotation_to_quaternion(rot, qcb);
    // The quaterion generated is the inverse, take the inverse again.
    qcb[1] = -qcb[1];
    qcb[2] = -qcb[2];
    qcb[3] = -qcb[3];

    // Now rotate by angle, negate angle to rotate other way
    q_adjust[0] = (long)((1L << 30) * cosf(-angle * (float)M_PI / 180.f / 2.f));
    q_adjust[1] = 0;
    q_adjust[2] = (long)((1L << 30) * sinf(-angle * (float)M_PI / 180.f / 2.f));
    q_adjust[3] = 0;
    inv_q_mult(q_adjust, qcb, q_all);
    inv_set_chip_to_body(q_all);
}

void inv_convert_dmp3_to_body(const long *vec3, float scale, float *values)
{
    long out[3];
    inv_q_rotate(s_quat_chip_to_body, vec3, out);
    values[0] = out[0] * scale;
    values[1] = out[1] * scale;
    values[2] = out[2] * scale;
}

/** Converts a 32-bit long to a little endian byte stream */
unsigned char *inv_int32_to_little8(long x, unsigned char *little8)
{
    little8[3] = (unsigned char)((x >> 24) & 0xff);
    little8[2] = (unsigned char)((x >> 16) & 0xff);
    little8[1] = (unsigned char)((x >> 8) & 0xff);
    little8[0] = (unsigned char)(x & 0xff);
    return little8;
}

