/*
$License:
Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
See included License.txt for License information.
$
*/

/*******************************************************************************
*
* $Id:$
*
******************************************************************************/

/**
*   @defgroup  ML_MATH_FUNC ml_math_func
*   @brief     Motion Library - Math Functions
*              Common math functions the Motion Library
*
*   @{
*       @file ml_math_func.c
*       @brief Math Functions.
*/

#include "mlmath.h"
#include "ml_math_func.h"
#include <string.h>

/** @internal
* Does the cross product of compass by gravity, then converts that
* to the world frame using the quaternion, then computes the angle that
* is made.
*
* @param[in] compass Compass Vector (Body Frame), length 3
* @param[in] grav Gravity Vector (Body Frame), length 3
* @param[in] quat Quaternion, Length 4
* @return Angle Cross Product makes after quaternion rotation.
*/
float inv_compass_angle(const long *compass, const long *grav, const float *quat)
{
    float cgcross[4], q1[4], q2[4], qi[4];
    float angW;

    // Compass cross Gravity
    cgcross[0] = 0.f;
    cgcross[1] = (float)compass[1] * grav[2] - (float)compass[2] * grav[1];
    cgcross[2] = (float)compass[2] * grav[0] - (float)compass[0] * grav[2];
    cgcross[3] = (float)compass[0] * grav[1] - (float)compass[1] * grav[0];

    // Now convert cross product into world frame
    inv_q_multf(quat, cgcross, q1);
    inv_q_invertf(quat, qi);
    inv_q_multf(q1, qi, q2);

    // Protect against atan2 of 0,0
    if ((q2[2] == 0.f) && (q2[1] == 0.f))
        return 0.f;

    // This is the unfiltered heading correction
    angW = -atan2f(q2[2], q2[1]);
    return angW;
}

/**
*  @brief  The gyro data magnitude squared :
*          (1 degree per second)^2 = 2^6 = 2^GYRO_MAG_SQR_SHIFT.
* @param[in] gyro Gyro data scaled with 1 dps = 2^16
*  @return the computed magnitude squared output of the gyroscope.
*/
unsigned long inv_get_gyro_sum_of_sqr(const long *gyro)
{
    unsigned long gmag = 0;
    long temp;
    int kk;

    for (kk = 0; kk < 3; ++kk)
    {
        temp = gyro[kk] >> (16 - (GYRO_MAG_SQR_SHIFT / 2));
        gmag += temp * temp;
    }

    return gmag;
}

/** Performs a multiply and shift by qfix. These are good functions to write in assembly on
* with devices with small memory where you want to get rid of the long long which some
* assemblers don't handle well
* @param[in] a
* @param[in] b
* @param[in] qfix, should be in 1,31 since a and b are on 32 bits
* @return ((long long)a*b)>>qfix
*/
long inv_qfix_mult(long a, long b, unsigned char qfix)
{
    #ifdef UMPL_ELIMINATE_64BIT
    long result;
    result = (long)((float)a * b / (1L << qfix));
    return result;
    #else
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> qfix);
    return result;
    #endif
}

/** Performs a multiply and shift by 29. These are good functions to write in assembly on
* with devices with small memory where you want to get rid of the long long which some
* assemblers don't handle well
* @param[in] a
* @param[in] b
* @return ((long long)a*b)>>29
*/
long inv_q29_mult(long a, long b)
{
    #ifdef UMPL_ELIMINATE_64BIT
    long result;
    result = (long)((float)a * b / (1L << 29));
    return result;
    #else
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> 29);
    return result;
    #endif
}

/** Performs a multiply and shift by 30. These are good functions to write in assembly on
* with devices with small memory where you want to get rid of the long long which some
* assemblers don't handle well
* @param[in] a
* @param[in] b
* @return ((long long)a*b)>>30
*/
long inv_q30_mult(long a, long b)
{
    #ifdef UMPL_ELIMINATE_64BIT
    long result;
    result = (long)((float)a * b / (1L << 30));
    return result;
    #else
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> 30);
    return result;
    #endif
}

void inv_q30_matrix_mult(const long left_matrix[9], const long right_matrix[9], long matrix_product[9])
{
    matrix_product[0] = inv_q30_mult(left_matrix[0], right_matrix[0]) + inv_q30_mult(left_matrix[1], right_matrix[3]) + inv_q30_mult(left_matrix[2], right_matrix[6]);
    matrix_product[1] = inv_q30_mult(left_matrix[0], right_matrix[1]) + inv_q30_mult(left_matrix[1], right_matrix[4]) + inv_q30_mult(left_matrix[2], right_matrix[7]);
    matrix_product[2] = inv_q30_mult(left_matrix[0], right_matrix[2]) + inv_q30_mult(left_matrix[1], right_matrix[5]) + inv_q30_mult(left_matrix[2], right_matrix[8]);

    matrix_product[3] = inv_q30_mult(left_matrix[3], right_matrix[0]) + inv_q30_mult(left_matrix[4], right_matrix[3]) + inv_q30_mult(left_matrix[5], right_matrix[6]);
    matrix_product[4] = inv_q30_mult(left_matrix[3], right_matrix[1]) + inv_q30_mult(left_matrix[4], right_matrix[4]) + inv_q30_mult(left_matrix[5], right_matrix[7]);
    matrix_product[5] = inv_q30_mult(left_matrix[3], right_matrix[2]) + inv_q30_mult(left_matrix[4], right_matrix[5]) + inv_q30_mult(left_matrix[5], right_matrix[8]);

    matrix_product[6] = inv_q30_mult(left_matrix[6], right_matrix[0]) + inv_q30_mult(left_matrix[7], right_matrix[3]) + inv_q30_mult(left_matrix[8], right_matrix[6]);
    matrix_product[7] = inv_q30_mult(left_matrix[6], right_matrix[1]) + inv_q30_mult(left_matrix[7], right_matrix[4]) + inv_q30_mult(left_matrix[8], right_matrix[7]);
    matrix_product[8] = inv_q30_mult(left_matrix[6], right_matrix[2]) + inv_q30_mult(left_matrix[7], right_matrix[5]) + inv_q30_mult(left_matrix[8], right_matrix[8]);
}

#ifndef UMPL_ELIMINATE_64BIT
long inv_q30_div(long a, long b)
{
    long long temp;
    long result;
    temp = (((long long)a) << 30) / b;
    result = (long)temp;
    return result;
}
#endif

/** Performs a multiply and shift by shift. These are good functions to write
* in assembly on with devices with small memory where you want to get rid of
* the long long which some assemblers don't handle well
* @param[in] a First multicand
* @param[in] b Second multicand
* @param[in] shift Shift amount after multiplying
* @return ((long long)a*b)<<shift
*/
#ifndef UMPL_ELIMINATE_64BIT
long inv_q_shift_mult(long a, long b, int shift)
{
    long result;
    result = (long)(((long long)a * b) >> shift);
    return result;
}
#endif

/** Performs a fixed point quaternion multiply.
* @param[in] q1 First Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30
* @param[in] q2 Second Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30
* @param[out] qProd Product after quaternion multiply. Length 4.
*             1.0 scaled to 2^30.
*/
void inv_q_mult(const long *q1, const long *q2, long *qProd)
{
    qProd[0] = inv_q30_mult(q1[0], q2[0]) - inv_q30_mult(q1[1], q2[1]) -
               inv_q30_mult(q1[2], q2[2]) - inv_q30_mult(q1[3], q2[3]);

    qProd[1] = inv_q30_mult(q1[0], q2[1]) + inv_q30_mult(q1[1], q2[0]) +
               inv_q30_mult(q1[2], q2[3]) - inv_q30_mult(q1[3], q2[2]);

    qProd[2] = inv_q30_mult(q1[0], q2[2]) - inv_q30_mult(q1[1], q2[3]) +
               inv_q30_mult(q1[2], q2[0]) + inv_q30_mult(q1[3], q2[1]);

    qProd[3] = inv_q30_mult(q1[0], q2[3]) + inv_q30_mult(q1[1], q2[2]) -
               inv_q30_mult(q1[2], q2[1]) + inv_q30_mult(q1[3], q2[0]);
}

/** Performs a fixed point quaternion addition.
* @param[in] q1 First Quaternion term, length 4. 1.0 scaled
*            to 2^30
* @param[in] q2 Second Quaternion term, length 4. 1.0 scaled
*            to 2^30
* @param[out] qSum Sum after quaternion summation. Length 4.
*             1.0 scaled to 2^30.
*/
void inv_q_add(long *q1, long *q2, long *qSum)
{
    qSum[0] = q1[0] + q2[0];
    qSum[1] = q1[1] + q2[1];
    qSum[2] = q1[2] + q2[2];
    qSum[3] = q1[3] + q2[3];
}

void inv_vector_normalize(long *vec, int length)
{
    double normSF = 0;
    int ii;

    for (ii = 0; ii < length; ii++)
    {
        normSF +=
            inv_q30_to_double(vec[ii]) * inv_q30_to_double(vec[ii]);
    }

    if (normSF > 0)
    {
        normSF = 1 / sqrt(normSF);

        for (ii = 0; ii < length; ii++)
        {
            vec[ii] = (int)((double)vec[ii] * normSF);
        }
    }
    else
    {
        vec[0] = 1073741824L;

        for (ii = 1; ii < length; ii++)
        {
            vec[ii] = 0;
        }
    }
}

void inv_q_normalize(long *q)
{
    inv_vector_normalize(q, 4);
}

void inv_q_invert(const long *q, long *qInverted)
{
    qInverted[0] = q[0];
    qInverted[1] = -q[1];
    qInverted[2] = -q[2];
    qInverted[3] = -q[3];
}

double quaternion_to_rotation_angle(const long *quat)
{
    double quat0 = (double)quat[0] / 1073741824;

    if (quat0 > 1.0f)
    {
        quat0 = 1.0;
    }
    else if (quat0 < -1.0f)
    {
        quat0 = -1.0;
    }

    return acos(quat0) * 2 * 180 / M_PI;
}

/** Rotates a 3-element vector by Rotation defined by Q
*/
void inv_q_rotate(const long *q, const long *in, long *out)
{
    long q_temp1[4], q_temp2[4];
    long in4[4], out4[4];

    // Fixme optimize
    in4[0] = 0;
    memcpy(&in4[1], in, 3 * sizeof(long));
    inv_q_mult(q, in4, q_temp1);
    inv_q_invert(q, q_temp2);
    inv_q_mult(q_temp1, q_temp2, out4);
    memcpy(out, &out4[1], 3 * sizeof(long));
}

/** Rotate a 3-element vector from body to inertial via quaternion in the DMP convention (it takes
*  inertial vector to body via B = q^-1 I q, which is same q as in I = q B q^-1 )
*  This is same convention as Matlab: http://www.mathworks.com/help/aerotbx/ug/quat2dcm.html
*  and to see these same examples ran through this function, see test/testU_ml_math_func.c
* @param[in] q_ItoB - quaternion using the DMP convention (it takes inertial vector to body via B = q^-1 I q, which is same q as in I = q B q^-1 )
* @param[in] B - vector in body frame
* @param[out] I - vector in inertial frame
*/
void inv_q_rotate_BtoI(const long *q_ItoB, const long *B, long *I)
{
    long q_ItoB_times_B[4], q_BtoI[4];
    long B4[4], I4[4];

    // put vector into last 3 slots in in4 and make 1st slot zero
    B4[0] = 0;
    memcpy(&B4[1], B, 3 * sizeof(long));

    //compute q*B in (I = q B q^-1)
    inv_q_mult(q_ItoB, B4, q_ItoB_times_B);

    //compute q^-1 in (I = q B q^-1)
    inv_q_invert(q_ItoB, q_BtoI);

    //compute I  in (I = q B q^-1)
    inv_q_mult(q_ItoB_times_B, q_BtoI, I4);

    //select just last 3 elements (could check if I4[0]==0?)
    memcpy(I, &I4[1], 3 * sizeof(long));
}

/** Rotate a 3-element vector from inertial to body via quaternion in the DMP convention (it takes
*  inertial vector to body via B = q^-1 I q, which is same q as in I = q B q^-1 )
*  This is same convention as Matlab: http://www.mathworks.com/help/aerotbx/ug/quat2dcm.html
*  and to see these same examples ran through this function, see test/testU_ml_math_func.c
* @param[in] q_ItoB - quaternion which using the formula (B = q^-1 I q) takes a vector from inertial to body
* @param[in] I - vector in inertial frame
* @param[out] B - vector in body frame
*/
void inv_q_rotate_ItoB(const long *q_ItoB, const long *I, long *B)
{
    long q_BtoI[4], q_BtoI_times_I[4];
    long I4[4], B4[4];

    // put vector into last 3 slots in I4 and make 1st slot zero
    I4[0] = 0;
    memcpy(&I4[1], I, 3 * sizeof(long));

    //compute q^-1 in (B = q^-1 I q)
    inv_q_invert(q_ItoB, q_BtoI);

    //compute q^-1*I in (B = q^-1 I q)
    inv_q_mult(q_BtoI, I4, q_BtoI_times_I);

    //compute (q^-1*I)*q in (B = q^-1 I q)
    inv_q_mult(q_BtoI_times_I, q_ItoB, B4);

    //select just last 3 elements in B4 (could check if B4[0]==0?)
    memcpy(B, &B4[1], 3 * sizeof(long));
}

void inv_q_multf(const float *q1, const float *q2, float *qProd)
{
    qProd[0] =
        (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    qProd[1] =
        (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    qProd[2] =
        (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    qProd[3] =
        (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
}

void inv_q_addf(const float *q1, const float *q2, float *qSum)
{
    qSum[0] = q1[0] + q2[0];
    qSum[1] = q1[1] + q2[1];
    qSum[2] = q1[2] + q2[2];
    qSum[3] = q1[3] + q2[3];
}

void inv_q_normalizef(float *q)
{
    float normSF = 0;
    float xHalf = 0;
    normSF = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (normSF < 2)
    {
        xHalf = 0.5f * normSF;
        normSF = normSF * (1.5f - xHalf * normSF * normSF);
        normSF = normSF * (1.5f - xHalf * normSF * normSF);
        normSF = normSF * (1.5f - xHalf * normSF * normSF);
        normSF = normSF * (1.5f - xHalf * normSF * normSF);
        q[0] *= normSF;
        q[1] *= normSF;
        q[2] *= normSF;
        q[3] *= normSF;
    }
    else
    {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    }

    normSF = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/** Performs a length 4 vector normalization with a square root.
* @param[in,out] q vector to normalize. Returns [1,0,0,0] is magnitude is zero.
*/
void inv_q_norm4(float *q)
{
    float mag;
    mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (mag)
    {
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    }
    else
    {
        q[0] = 1.f;
        q[1] = 0.f;
        q[2] = 0.f;
        q[3] = 0.f;
    }
}

void inv_q_invertf(const float *q, float *qInverted)
{
    qInverted[0] = q[0];
    qInverted[1] = -q[1];
    qInverted[2] = -q[2];
    qInverted[3] = -q[3];
}

/**
* Does triad in ENU (Z-up Y-north) convention
* one-step triad - get quaternion from accel and compass at one instant.
* based on a modified version of triad.m in users\algorithms folder (no transpose on "R = [X, Y, Z]';" line).
* should return q=[334637952 -284493440 703934784 681526400] when
* when accel=[-24998052 22384162 0], compass=[1754490 -1443843 1307299], and accel_fs=11.
*
* @param[in] long accel_body[3] - accel in rawscaled units
*            long compass_body[3] - compass in rawscaled units (1uT=2^16lsb)
*            long accel_fs - how is accel scaled (11 = 2g, 12=4g, etc)
* @param[out] long Qbi_fp[4] - quaternion in fixed point that takes you from Inertial to Body frame (DSB notation). 1.0 is 2^30lsb.
*/
void inv_triad(long *accel_body, long *compass_body, long accel_fs, long *Qbi_fp)
{
    float acc[3], an, mag[3], mn, X[3], Xn, Y[3], Z[3], O_BI[9];

    acc[0] = (float)accel_body[0] / ((float)(1 << (14 + accel_fs))); //get accel into float g's, ie divide RawScaled by 2^(14+11) in 2g setting or 26 in 4g, etc. 14 comes from basic raw definition: 2g=2^15lsb
    acc[1] = (float)accel_body[1] / ((float)(1 << (14 + accel_fs)));
    acc[2] = (float)accel_body[2] / ((float)(1 << (14 + accel_fs)));

    mag[0] = (float)compass_body[0] / ((float)(1 << 16)); //get compass into float uT's, ie divide RawScaled by 2^16
    mag[1] = (float)compass_body[1] / ((float)(1 << 16));
    mag[2] = (float)compass_body[2] / ((float)(1 << 16));

    //Z = accel/norm(accel);
    an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]); //acc norm
    Z[0] = acc[0] / an;
    Z[1] = acc[1] / an;
    Z[2] = acc[2] / an;

    //compass = compass/norm(compass);
    mn = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]); //mag norm
    mag[0] = mag[0] / mn;
    mag[1] = mag[1] / mn;
    mag[2] = mag[2] / mn;

    //X = cross(compass, Z);
    X[0] = -mag[2] * Z[1] + mag[1] * Z[2];
    X[1] = mag[2] * Z[0] - mag[0] * Z[2];
    X[2] = -mag[1] * Z[0] + mag[0] * Z[1];

    //X = X/norm(X);
    Xn = sqrtf(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]); //X norm
    X[0] = X[0] / Xn;
    X[1] = X[1] / Xn;
    X[2] = X[2] / Xn;

    //Y = cross(Z, X);
    Y[0] = -Z[2] * X[1] + Z[1] * X[2];
    Y[1] = Z[2] * X[0] - Z[0] * X[2];
    Y[2] = -Z[1] * X[0] + Z[0] * X[1];

    //O = [X, Y, Z];
    O_BI[0] = X[0];
    O_BI[1] = Y[0];
    O_BI[2] = Z[0];
    O_BI[3] = X[1];
    O_BI[4] = Y[1];
    O_BI[5] = Z[1];
    O_BI[6] = X[2];
    O_BI[7] = Y[2];
    O_BI[8] = Z[2];

    //q = O2q(O);
    inv_rotation_to_quaternion(O_BI, Qbi_fp);
}

void inv_triad_fxp_xnorth(const long *accel, const long *compass, long *quat)
{
    //if (norm(accel)==0 || norm(compass)==0 )%|| norm(accel)> 1.1)
    //    q=qCurrent;
    //    return
    //end
    //
    //Z = accel/norm(accel);
    //compass = compass/norm(compass);
    //
    //X = cross(compass, Z);
    //X = X/norm(X);
    //Y = cross(Z, X);
    //
    //R = [X, Y, Z]';
    //q = dcmToQuat(R);

    long tmp;
    long R[9], rot[9];
    long *X, *Y, *Z;
    int shift, pow2;

    X = R;
    Y = (long*)(R + 3);
    Z = (long*)(R + 6);

    // Normalize
    tmp = (long)(((long long)accel[0] * accel[0] +
                  (long long)accel[1] * accel[1] +
                  (long long)accel[2] * accel[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        Z[0] = (long)(((long long)accel[0] * tmp) >> shift);
        Z[1] = (long)(((long long)accel[1] * tmp) >> shift);
        Z[2] = (long)(((long long)accel[2] * tmp) >> shift);
    }
    else
        return;


    // Cross product Z x compass and get Y
    // Cross Vert x compass --> North direction
    Y[0] = (long)(((long long)Z[1] * compass[2] - (long long)Z[2] * compass[1]) >> 30);
    Y[1] = (long)(((long long)Z[2] * compass[0] - (long long)Z[0] * compass[2]) >> 30);
    Y[2] = (long)(((long long)Z[0] * compass[1] - (long long)Z[1] * compass[0]) >> 30);

    tmp = (long)(((long long)Y[0] * Y[0] +
                  (long long)Y[1] * Y[1] +
                  (long long)Y[2] * Y[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        Y[0] = (long)(((long long)Y[0] * tmp) >> shift);
        Y[1] = (long)(((long long)Y[1] * tmp) >> shift);
        Y[2] = (long)(((long long)Y[2] * tmp) >> shift);
    }
    else
        return;

    // Cross product: YxZ and get X
    // Cross Vert x East -- > North direction
    // Result is unit norm since both vectors are normalized.
    X[0] = (long)(((long long)Y[1] * Z[2] - (long long)Y[2] * Z[1]) >> 30);
    X[1] = (long)(((long long)Y[2] * Z[0] - (long long)Y[0] * Z[2]) >> 30);
    X[2] = (long)(((long long)Y[0] * Z[1] - (long long)Y[1] * Z[0]) >> 30);

    rot[0] = R[0];
    rot[3] = R[1];
    rot[6] = R[2];

    rot[1] = R[3];
    rot[4] = R[4];
    rot[7] = R[5];

    rot[2] = R[6];
    rot[5] = R[7];
    rot[8] = R[8];

    inv_rotation_to_quaternion_fxp(rot, quat);


    return;
}


void inv_triad_fxp_ynorth(const long *accel, const long *compass, long *quat)
{
    //if (norm(accel)==0 || norm(compass)==0 )%|| norm(accel)> 1.1)
    //    q=qCurrent;
    //    return
    //end
    //
    //Z = accel/norm(accel);
    //compass = compass/norm(compass);
    //
    //X = cross(compass, Z);
    //X = X/norm(X);
    //Y = cross(Z, X);
    //
    //R = [X, Y, Z]';
    //q = dcmToQuat(R);

    long tmp;
    long R[9], rot[9];
    long *X, *Y, *Z;
    int shift, pow2;

    X = R;
    Y = (long*)(R + 3);
    Z = (long*)(R + 6);

    // Normalize
    tmp = (long)(((long long)accel[0] * accel[0] +
                  (long long)accel[1] * accel[1] +
                  (long long)accel[2] * accel[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        Z[0] = (long)(((long long)accel[0] * tmp) >> shift);
        Z[1] = (long)(((long long)accel[1] * tmp) >> shift);
        Z[2] = (long)(((long long)accel[2] * tmp) >> shift);
    }
    else
        return;

    /*
    X[0] = - mag[2] * Z[1] + mag[1] * Z[2];
    X[1] =   mag[2] * Z[0] - mag[0] * Z[2];
    X[2] = - mag[1] * Z[0] + mag[0] * Z[1];
    */

    // Cross product compass x Z and get X
    X[0] = (long)((-(long long)Z[1] * compass[2] + (long long)Z[2] * compass[1]) >> 30);
    X[1] = (long)((-(long long)Z[2] * compass[0] + (long long)Z[0] * compass[2]) >> 30);
    X[2] = (long)((-(long long)Z[0] * compass[1] + (long long)Z[1] * compass[0]) >> 30);

    tmp = (long)(((long long)X[0] * X[0] +
                  (long long)X[1] * X[1] +
                  (long long)X[2] * X[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        X[0] = (long)(((long long)X[0] * tmp) >> shift);
        X[1] = (long)(((long long)X[1] * tmp) >> shift);
        X[2] = (long)(((long long)X[2] * tmp) >> shift);
    }
    else
        return;

    /*
    Y[0] = - Z[2] * X[1] + Z[1] * X[2];
    Y[1] =   Z[2] * X[0] - Z[0] * X[2];
    Y[2] = - Z[1] * X[0] + Z[0] * X[1];
    */

    // Cross product: ZxX and get Y
    Y[0] = (long)((-(long long)X[1] * Z[2] + (long long)X[2] * Z[1]) >> 30);
    Y[1] = (long)((-(long long)X[2] * Z[0] + (long long)X[0] * Z[2]) >> 30);
    Y[2] = (long)((-(long long)X[0] * Z[1] + (long long)X[1] * Z[0]) >> 30);


    rot[0] = R[0];
    rot[3] = R[1];
    rot[6] = R[2];

    rot[1] = R[3];
    rot[4] = R[4];
    rot[7] = R[5];

    rot[2] = R[6];
    rot[5] = R[7];
    rot[8] = R[8];

    inv_rotation_to_quaternion_fxp(rot, quat);

    return;
}

/**
* Converts a rotation matrix to a quaternion.
* @param[in] Rcb Rotation matrix in floating point. The
*             First 3 elements of the rotation matrix, represent
*             the first row of the matrix.
* @param[out] Qcb_fp 4-element quaternion in fixed point. One is 2^30.
*/
void inv_rotation_to_quaternion(float *Rcb, long *Qcb_fp)
{
    float r11, r12, r13, r21, r22, r23, r31, r32, r33;
    float Qcb[4];

    r11 = Rcb[0]; //assume matrix is stored row wise first, that is rot[1] is row 1, col 2
    r12 = Rcb[1];
    r13 = Rcb[2];

    r21 = Rcb[3];
    r22 = Rcb[4];
    r23 = Rcb[5];

    r31 = Rcb[6];
    r32 = Rcb[7];
    r33 = Rcb[8];

    Qcb[0] = (1.f + r11 + r22 + r33) / 4.f;
    Qcb[1] = (1.f + r11 - r22 - r33) / 4.f;
    Qcb[2] = (1.f - r11 + r22 - r33) / 4.f;
    Qcb[3] = (1.f - r11 - r22 + r33) / 4.f;

    if (Qcb[0] < 0.0f) Qcb[0] = 0.0f;

    if (Qcb[1] < 0.0f) Qcb[1] = 0.0f;

    if (Qcb[2] < 0.0f) Qcb[2] = 0.0f;

    if (Qcb[3] < 0.0f) Qcb[3] = 0.0f;

    Qcb[0] = sqrtf(Qcb[0]);
    Qcb[1] = sqrtf(Qcb[1]);
    Qcb[2] = sqrtf(Qcb[2]);
    Qcb[3] = sqrtf(Qcb[3]);

    if (Qcb[0] >= Qcb[1] && Qcb[0] >= Qcb[2] && Qcb[0] >= Qcb[3]) //Qcb[0] is max
    {
        Qcb[1] = (r23 - r32) / (4.f * Qcb[0]);
        Qcb[2] = (r31 - r13) / (4.f * Qcb[0]);
        Qcb[3] = (r12 - r21) / (4.f * Qcb[0]);
    }
    else if (Qcb[1] >= Qcb[0] && Qcb[1] >= Qcb[2] && Qcb[1] >= Qcb[3]) //Qcb[1] is max
    {
        Qcb[0] = (r23 - r32) / (4.f * Qcb[1]);
        Qcb[2] = (r12 + r21) / (4.f * Qcb[1]);
        Qcb[3] = (r31 + r13) / (4.f * Qcb[1]);
    }
    else if (Qcb[2] >= Qcb[0] && Qcb[2] >= Qcb[1] && Qcb[2] >= Qcb[3]) //Qcb[2] is max
    {
        Qcb[0] = (r31 - r13) / (4.f * Qcb[2]);
        Qcb[1] = (r12 + r21) / (4.f * Qcb[2]);
        Qcb[3] = (r23 + r32) / (4.f * Qcb[2]);
    }
    else if (Qcb[3] >= Qcb[0] && Qcb[3] >= Qcb[1] && Qcb[3] >= Qcb[2]) //Qcb[3] is max
    {
        Qcb[0] = (r12 - r21) / (4.f * Qcb[3]);
        Qcb[1] = (r31 + r13) / (4.f * Qcb[3]);
        Qcb[2] = (r23 + r32) / (4.f * Qcb[3]);
    }
    else
    {
        //printf('coding error\n'); //error
    }

    Qcb_fp[0] = (long)(Qcb[0] * 1073741824.0f);
    Qcb_fp[1] = (long)(Qcb[1] * 1073741824.0f);
    Qcb_fp[2] = (long)(Qcb[2] * 1073741824.0f);
    Qcb_fp[3] = (long)(Qcb[3] * 1073741824.0f);
}

/**
* Converts a quaternion to a rotation matrix.
* @param[in] quat 4-element quaternion in fixed point. One is 2^30.
* @param[out] rot Rotation matrix in fixed point. One is 2^30. The
*             First 3 elements of the rotation matrix, represent
*             the first row of the matrix. Rotation matrix multiplied
*             by a 3 element column vector transform a vector from Body
*             to World.
*/
// MATRIX STORAGE IS COLUMN-WISE!
void inv_transpose_quaternion_to_rotation(const long *quat, long *rot)
{
    rot[0] =
        inv_q29_mult(quat[1], quat[1]) + inv_q29_mult(quat[0],
                quat[0]) -
        1073741824L;
    rot[1] =
        inv_q29_mult(quat[1], quat[2]) - inv_q29_mult(quat[3], quat[0]);
    rot[2] =
        inv_q29_mult(quat[1], quat[3]) + inv_q29_mult(quat[2], quat[0]);
    rot[3] =
        inv_q29_mult(quat[1], quat[2]) + inv_q29_mult(quat[3], quat[0]);
    rot[4] =
        inv_q29_mult(quat[2], quat[2]) + inv_q29_mult(quat[0],
                quat[0]) -
        1073741824L;
    rot[5] =
        inv_q29_mult(quat[2], quat[3]) - inv_q29_mult(quat[1], quat[0]);
    rot[6] =
        inv_q29_mult(quat[1], quat[3]) - inv_q29_mult(quat[2], quat[0]);
    rot[7] =
        inv_q29_mult(quat[2], quat[3]) + inv_q29_mult(quat[1], quat[0]);
    rot[8] =
        inv_q29_mult(quat[3], quat[3]) + inv_q29_mult(quat[0],
                quat[0]) -
        1073741824L;
}

/**
* Converts a quaternion to a rotation matrix.
* @param[in] q_ItoB - quaternion in the DMP convention (it takes
*  inertial vector to body via B = q^-1 I q, which is same q as in I = q B q^-1 )
*  This is same convention as Matlab: http://www.mathworks.com/help/aerotbx/ug/quat2dcm.html
*  and to see these same examples ran through this function, see test/testU_ml_math_func.c
* @param[out] R_ItoB - Rotation matrix in DMP convention (it takes
*  inertial vector to body via B = R I)
*             First 3 elements of the rotation matrix, represent
*             the first row of the matrix. Rotation matrix multiplied
*             by a 3 element column vector transform a vector from Inertial
*             to Body.
*/
// OUTPUT matrix rotates INERTIAL vector into BODY frame
void inv_quaternion_to_rotation(const long *q_ItoB, long *R_ItoB)
{
    R_ItoB[0] =
        inv_q29_mult(q_ItoB[1], q_ItoB[1]) + inv_q29_mult(q_ItoB[0], q_ItoB[0]) - 1073741824L;
    R_ItoB[1] =
        inv_q29_mult(q_ItoB[1], q_ItoB[2]) + inv_q29_mult(q_ItoB[3], q_ItoB[0]);
    R_ItoB[2] =
        inv_q29_mult(q_ItoB[1], q_ItoB[3]) - inv_q29_mult(q_ItoB[2], q_ItoB[0]);
    R_ItoB[3] =
        inv_q29_mult(q_ItoB[1], q_ItoB[2]) - inv_q29_mult(q_ItoB[3], q_ItoB[0]);
    R_ItoB[4] =
        inv_q29_mult(q_ItoB[2], q_ItoB[2]) + inv_q29_mult(q_ItoB[0], q_ItoB[0]) - 1073741824L;
    R_ItoB[5] =
        inv_q29_mult(q_ItoB[2], q_ItoB[3]) + inv_q29_mult(q_ItoB[1], q_ItoB[0]);
    R_ItoB[6] =
        inv_q29_mult(q_ItoB[1], q_ItoB[3]) + inv_q29_mult(q_ItoB[2], q_ItoB[0]);
    R_ItoB[7] =
        inv_q29_mult(q_ItoB[2], q_ItoB[3]) - inv_q29_mult(q_ItoB[1], q_ItoB[0]);
    R_ItoB[8] =
        inv_q29_mult(q_ItoB[3], q_ItoB[3]) + inv_q29_mult(q_ItoB[0], q_ItoB[0]) - 1073741824L;
}

/**
* Converts a quaternion to a rotation vector. A rotation vector is
* a method to represent a 4-element quaternion vector in 3-elements.
* To get the quaternion from the 3-elements, The last 3-elements of
* the quaternion will be the given rotation vector. The first element
* of the quaternion will be the positive value that will be required
* to make the magnitude of the quaternion 1.0 or 2^30 in fixed point units.
* @param[in] quat 4-element quaternion in fixed point. One is 2^30.
* @param[out] rot Rotation vector in fixed point. One is 2^30.
*/
void inv_quaternion_to_rotation_vector(const long *quat, long *rot)
{
    rot[0] = quat[1];
    rot[1] = quat[2];
    rot[2] = quat[3];

    if (quat[0] < 0.0)
    {
        rot[0] = -rot[0];
        rot[1] = -rot[1];
        rot[2] = -rot[2];
    }
}

/** Converts a 32-bit long to a big endian byte stream */
unsigned char *inv_int32_to_big8(long x, unsigned char *big8)
{
    big8[0] = (unsigned char)((x >> 24) & 0xff);
    big8[1] = (unsigned char)((x >> 16) & 0xff);
    big8[2] = (unsigned char)((x >> 8) & 0xff);
    big8[3] = (unsigned char)(x & 0xff);
    return big8;
}

/** Converts a big endian byte stream into a 32-bit long */
long inv_big8_to_int32(const unsigned char *big8)
{
    long x;
    x = ((long)big8[0] << 24) | ((long)big8[1] << 16) | ((long)big8[2] << 8)
        | ((long)big8[3]);
    return x;
}

/** Converts a big endian byte stream into a 16-bit integer (short) */
short inv_big8_to_int16(const unsigned char *big8)
{
    short x;
    x = ((short)big8[0] << 8) | ((short)big8[1]);
    return x;
}

/** Converts a little endian byte stream into a 16-bit integer (short) */
short inv_little8_to_int16(const unsigned char *little8)
{
    short x;
    x = ((short)little8[1] << 8) | ((short)little8[0]);
    return x;
}

/** Converts a 16-bit short to a big endian byte stream */
unsigned char *inv_int16_to_big8(short x, unsigned char *big8)
{
    big8[0] = (unsigned char)((x >> 8) & 0xff);
    big8[1] = (unsigned char)(x & 0xff);
    return big8;
}

void inv_matrix_det_inc(float *a, float *b, int *n, int x, int y)
{
    int k, l, i, j;

    for (i = 0, k = 0; i < *n; i++, k++)
    {
        for (j = 0, l = 0; j < *n; j++, l++)
        {
            if (i == x)
                i++;

            if (j == y)
                j++;

            *(b + 6 * k + l) = *(a + 6 * i + j);
        }
    }

    *n = *n - 1;
}

void inv_matrix_det_incd(double *a, double *b, int *n, int x, int y)
{
    int k, l, i, j;

    for (i = 0, k = 0; i < *n; i++, k++)
    {
        for (j = 0, l = 0; j < *n; j++, l++)
        {
            if (i == x)
                i++;

            if (j == y)
                j++;

            *(b + 6 * k + l) = *(a + 6 * i + j);
        }
    }

    *n = *n - 1;
}

float inv_matrix_det(float *p, int *n)
{
    float d[6][6], sum = 0;
    int i, j, m;
    m = *n;

    if (*n == 2)
        return (*p ** (p + 7) - * (p + 1) ** (p + 6));

    for (i = 0, j = 0; j < m; j++)
    {
        *n = m;
        inv_matrix_det_inc(p, &d[0][0], n, i, j);
        sum =
            sum + *(p + 6 * i + j) * SIGNM(i +
                                           j) *
            inv_matrix_det(&d[0][0], n);
    }

    return (sum);
}

double inv_matrix_detd(double *p, int *n)
{
    double d[6][6], sum = 0;
    int i, j, m;
    m = *n;

    if (*n == 2)
        return (*p ** (p + 7) - * (p + 1) ** (p + 6));

    for (i = 0, j = 0; j < m; j++)
    {
        *n = m;
        inv_matrix_det_incd(p, &d[0][0], n, i, j);
        sum =
            sum + *(p + 6 * i + j) * SIGNM(i +
                                           j) *
            inv_matrix_detd(&d[0][0], n);
    }

    return (sum);
}

/** Wraps angle from (-M_PI,M_PI]
* @param[in] ang Angle in radians to wrap
* @return Wrapped angle from (-M_PI,M_PI]
*/
float inv_wrap_angle(float ang)
{
    if (ang > M_PI)
        return ang - 2 * (float)M_PI;
    else if (ang <= -(float)M_PI)
        return ang + 2 * (float)M_PI;
    else
        return ang;
}

/** Finds the minimum angle difference ang1-ang2 such that difference
* is between [-M_PI,M_PI]
* @param[in] ang1
* @param[in] ang2
* @return angle difference ang1-ang2
*/
float inv_angle_diff(float ang1, float ang2)
{
    float d;
    ang1 = inv_wrap_angle(ang1);
    ang2 = inv_wrap_angle(ang2);
    d = ang1 - ang2;

    if (d > M_PI)
        d -= 2 * (float)M_PI;
    else if (d < -(float)M_PI)
        d += 2 * (float)M_PI;

    return d;
}

/** bernstein hash, derived from public domain source */
uint32_t inv_checksum(const unsigned char *str, int len)
{
    uint32_t hash = 5381;
    int i, c;

    for (i = 0; i < len; i++)
    {
        c = *(str + i);
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }

    return hash;
}

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error

    return b;
}


/** Converts an orientation matrix made up of 0,+1,and -1 to a scalar representation.
* @param[in] mtx Orientation matrix to convert to a scalar.
* @return Description of orientation matrix. The lowest 2 bits (0 and 1) represent the column the one is on for the
* first row, with the bit number 2 being the sign. The next 2 bits (3 and 4) represent
* the column the one is on for the second row with bit number 5 being the sign.
* The next 2 bits (6 and 7) represent the column the one is on for the third row with
* bit number 8 being the sign. In binary the identity matrix would therefor be:
* 010_001_000 or 0x88 in hex.
*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
    XYZ  010_001_000 Identity Matrix
    XZY  001_010_000
    YXZ  010_000_001
    YZX  000_010_001
    ZXY  001_000_010
    ZYX  000_001_010
    */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/** Uses the scalar orientation value to convert from body frame to chip frame
* see comment above for inv_orientation_matrix_to_scalar to see how "orient" is encoded. This is just reverse of inv_orientation_matrix_to_scalar
* @param[in] orient A scalar that represent how to go from chip to body frame
* @param[out] mtx Output matrix, length 9
* @param[in] sensitivity A q30 representation of sensitivity, eg for compass 0.15uT/1bit is encoded as int32(0.15*2^30) = 161061274
*/
void inv_convert_orient_scalar_to_dmp_compass_mtx(short orient, long *mtx, long sensitivity)
{
    long value;

    memset(mtx, 0, 9 * sizeof(long));

    if (orient & 4)
        value = -1073741824L;
    else
        value = 1073741824L;

    mtx[(orient & 3)] = (long)(((long long)value * sensitivity) >> 30);

    if (orient & 0x20)
        value = -1073741824L;
    else
        value = 1073741824L;

    mtx[((orient >> 3) & 3) + 3] = (long)(((long long)value * sensitivity) >> 30);

    if (orient & 0x100)
        value = -1073741824L;
    else
        value = 1073741824L;

    mtx[((orient >> 6) & 3) + 6] = (long)(((long long)value * sensitivity) >> 30);
}

/** Uses the scalar orientation value to convert from chip frame to body frame
* @param[in] orientation A scalar that represent how to go from chip to body frame
* @param[in] input Input vector, length 3
* @param[out] output Output vector, length 3
*/
void inv_convert_to_body(unsigned short orientation, const long *input, long *output)
{
    output[0] = input[orientation & 0x03] * SIGNSET(orientation & 0x004);
    output[1] = input[(orientation >> 3) & 0x03] * SIGNSET(orientation & 0x020);
    output[2] = input[(orientation >> 6) & 0x03] * SIGNSET(orientation & 0x100);
}

/** Uses the scalar orientation value to convert from body frame to chip frame
* @param[in] orientation A scalar that represent how to go from chip to body frame
* @param[in] input Input vector, length 3
* @param[out] output Output vector, length 3
*/
void inv_convert_to_chip(unsigned short orientation, const long *input, long *output)
{
    output[orientation & 0x03] = input[0] * SIGNSET(orientation & 0x004);
    output[(orientation >> 3) & 0x03] = input[1] * SIGNSET(orientation & 0x020);
    output[(orientation >> 6) & 0x03] = input[2] * SIGNSET(orientation & 0x100);
}

/** Uses the scalar orientation value to convert from body frame to chip frame
* @param[in] orientation A scalar that represent how to go from chip to body frame
* @param[out] output Output matrix, length 9
*/
void inv_orientation_scalar_to_matrix(unsigned short orientation, long *output)
{
    output[0] = 0;
    output[1] = 0;
    output[2] = 0;
    output[3] = 0;
    output[4] = 0;
    output[5] = 0;
    output[6] = 0;
    output[7] = 0;
    output[8] = 0;


    output[orientation & 0x03] = SIGNSET(orientation & 0x004);
    output[((orientation >> 3) & 0x03) + 3] = SIGNSET(orientation & 0x020);
    output[((orientation >> 6) & 0x03) + 6] = SIGNSET(orientation & 0x100);
}


/** Uses the scalar orientation value to convert from chip frame to body frame and
* apply appropriate scaling.
* @param[in] orientation A scalar that represent how to go from chip to body frame
* @param[in] sensitivity Sensitivity scale
* @param[in] input Input vector, length 3
* @param[out] output Output vector, length 3
*/
void inv_convert_to_body_with_scale(unsigned short orientation,
                                    long sensitivity,
                                    const long *input, long *output)
{
    output[0] = inv_q30_mult(input[orientation & 0x03] *
                             SIGNSET(orientation & 0x004), sensitivity);
    output[1] = inv_q30_mult(input[(orientation >> 3) & 0x03] *
                             SIGNSET(orientation & 0x020), sensitivity);
    output[2] = inv_q30_mult(input[(orientation >> 6) & 0x03] *
                             SIGNSET(orientation & 0x100), sensitivity);
}

/* Converts Quaternion from chip to body
*  Requires real part
*  (1) q_rotation([1 0 0 0]) (2) q_rotation([sqrt(.5) 0 0 sqrt(.5)])
*  (3) q_rotation([sqrt(.5) 0 0 -sqrt(.5)]) (4) q_rotation([0 1 0 0])
*  (5) q_rotation([0 0 1 0]) (6) q_rotation([0 -sqrt(.5) sqrt(.5) 0])
*  (7) q_rotation([0 sqrt(.5) sqrt(.5) 0]) (8) q_rotation([0 0 0 1])
*/
//#define nexus5
void inv_convert_quaternion_to_body(long *mcb,
                                    const long *Qc, long *Qb)
{
    long qcb[4];
    float Rcb[9];
    int i;
    #ifdef nexus5
    qcb[0] = 0;
    qcb[1] = 759250125; //0.7071
    qcb[2] = 759250125;
    qcb[3] = 0;
    #else

    for (i = 0; i < 9; i++)
        Rcb[i] = (float)mcb[i];

    inv_rotation_to_quaternion(Rcb, qcb);
    #endif
    inv_q_mult(Qc, qcb, Qb);
    return;
}
/** find a norm for a vector
* @param[in] a vector [3x1]
* @param[out] output the norm of the input vector
*/
double inv_vector_norm(const float *x)
{
    return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}


void inv_get_cross_product_vec(float *cgcross, float compass[3], float grav[3])
{

    cgcross[0] = (float)compass[1] * grav[2] - (float)compass[2] * grav[1];
    cgcross[1] = (float)compass[2] * grav[0] - (float)compass[0] * grav[2];
    cgcross[2] = (float)compass[0] * grav[1] - (float)compass[1] * grav[0];
}

void mlMatrixVectorMult(long matrix[9], const long vecIn[3], long *vecOut)
{
    // matrix format
    //  [ 0  3  6;
    //    1  4  7;
    //    2  5  8]

    // vector format:  [0  1  2]^T;
    int i, j;
    long temp;

    for (i = 0; i < 3; i++)
    {
        temp = 0;

        for (j = 0; j < 3; j++)
        {
            temp += inv_q30_mult(matrix[i + j * 3], vecIn[j]);
        }

        vecOut[i] = temp;
    }
}

//============== 1/sqrt(x), 1/x, sqrt(x) Functions ================================

/** Calculates 1/square-root of a fixed-point number (30 bit mantissa, positive): Q1.30
* Input must be a positive scaled (2^30) integer
* The number is scaled to lie between a range in which a Newton-Raphson
* iteration works best. Corresponding square root of the power of two is returned.
*  Caller must scale final result by 2^rempow (while avoiding overflow).
* @param[in] x0, length 1
* @param[out] rempow, length 1
* @return scaledSquareRoot on success or zero.
*/
long inv_inverse_sqrt(long x0, int*rempow)
{
    //% Inverse sqrt NR in the neighborhood of 1.3>x>=0.65
    //% x(k+1) = x(k)*(3 - x0*x(k)^2)

    //% Seed equals 1. Works best in this region.
    //xx0 = int32(1*2^30);

    long oneoversqrt2, oneandhalf, x0_2;
    unsigned long xx;
    int pow2, sq2scale, nr_iters;
    //long upscale, sqrt_upscale, upsclimit;
    //long downscale, sqrt_downscale, downsclimit;

    // Precompute some constants for efficiency
    //% int32(2^30*1/sqrt(2))
    oneoversqrt2 = 759250125L;
    //% int32(1.5*2^30);
    oneandhalf = 1610612736L;

    //// Further scaling into optimal region saves one or more NR iterations. Maps into region (.9, 1.1)
    //// int32(0.9/log(2)*2^30)
    //upscale = 1394173804L;
    //// int32(sqrt(0.9/log(2))*2^30)
    //sqrt_upscale = 1223512453L;
    // // int32(1.1*log(2)/.9*2^30)
    //upsclimit = 909652478L;
    //// int32(1.1/log(4)*2^30)
    //downscale = 851995103L;
    //// int32(sqrt(1.1/log(4))*2^30)
    //sqrt_downscale = 956463682L;
    // // int32(0.9*log(4)/1.1*2^30)
    //downsclimit = 1217881829L;

    nr_iters = test_limits_and_scale(&x0, &pow2);

    sq2scale = pow2 % 2;  // Find remainder. Is it even or odd?
    // Further scaling to decrease NR iterations
    // With the mapping below, 89% of calculations will require 2 NR iterations or less.
    // TBD


    x0_2 = x0 >> 1; // This scaling incorporates factor of 2 in NR iteration below.
    // Initial condition starts at 1: xx=(1L<<30);
    // First iteration is simple: Instead of initializing xx=1, assign to result of first iteration:
    // xx= (3/2-x0/2);
    //% NR formula: xx=xx*(3/2-x0*xx*xx/2); = xx*(1.5 - (x0/2)*xx*xx)
    // Initialize NR (first iteration). Note we are starting with xx=1, so the first iteration becomes an initialization.
    xx = oneandhalf - x0_2;

    if (nr_iters >= 2)
    {
        // Second NR iteration
        xx = inv_q30_mult(xx, (oneandhalf - inv_q30_mult(x0_2, inv_q30_mult(xx, xx))));

        if (nr_iters == 3)
        {
            // Third NR iteration.
            xx = inv_q30_mult(xx, (oneandhalf - inv_q30_mult(x0_2, inv_q30_mult(xx, xx))));
            // Fourth NR iteration: Not needed due to single precision.
        }
    }

    if (sq2scale)
    {
        *rempow = (pow2 >> 1) + 1; // Account for sqrt(2) in denominator, note we multiply if s2scale is true
        return (inv_q30_mult(xx, oneoversqrt2));
    }
    else
    {
        *rempow = pow2 >> 1;
        return xx;
    }
}


/** Calculates square-root of a fixed-point number (30 bit mantissa, positive)
* Input must be a positive scaled (2^30) integer
* The number is scaled to lie between a range in which a Newton-Raphson
* iteration works best.
* @param[in] x0, length 1
* @return scaledSquareRoot on success or zero. **/
long inv_fast_sqrt(long x0)
{

    //% Square-Root with NR in the neighborhood of 1.3>x>=0.65 (log(2) <= x <= log(4) )
    // Two-variable NR iteration:
    // Initialize: a=x; c=x-1;
    // 1st Newton Step:  a=a-a*c/2; ( or: a = x - x*(x-1)/2  )
    // Iterate: c = c*c*(c-3)/4
    //          a = a - a*c/2    --> reevaluating c at this step gives error of approximation

    //% Seed equals 1. Works best in this region.
    //xx0 = int32(1*2^30);

    long sqrt2, oneoversqrt2, one_pt5;
    long xx, cc;
    int pow2, sq2scale, nr_iters;

    // Return if input is zero. Negative should really error out.
    if (x0 <= 0L)
    {
        return 0L;
    }

    sqrt2 = 1518500250L;
    oneoversqrt2 = 759250125L;
    one_pt5 = 1610612736L;

    nr_iters = test_limits_and_scale(&x0, &pow2);

    sq2scale = 0;

    if (pow2 > 0)
        sq2scale = pow2 % 2;  // Find remainder. Is it even or odd?

    pow2 = pow2 - sq2scale; // Now pow2 is even. Note we are adding because result is scaled with sqrt(2)

    // Sqrt 1st NR iteration
    cc = x0 - (1L << 30);
    xx = x0 - (inv_q30_mult(x0, cc) >> 1);

    if (nr_iters >= 2)
    {
        // Sqrt second NR iteration
        // cc = cc*cc*(cc-3)/4; = cc*cc*(cc/2 - 3/2)/2;
        // cc = ( cc*cc*((cc>>1) - onePt5) ) >> 1
        cc = inv_q30_mult(cc, inv_q30_mult(cc, (cc >> 1) - one_pt5)) >> 1;
        xx = xx - (inv_q30_mult(xx, cc) >> 1);

        if (nr_iters == 3)
        {
            // Sqrt third NR iteration
            cc = inv_q30_mult(cc, inv_q30_mult(cc, (cc >> 1) - one_pt5)) >> 1;
            xx = xx - (inv_q30_mult(xx, cc) >> 1);
        }
    }

    if (sq2scale)
        xx = inv_q30_mult(xx, oneoversqrt2);

    // Scale the number with the half of the power of 2 scaling
    if (pow2 > 0)
        xx = (xx >> (pow2 >> 1));
    else if (pow2 == -1)
        xx = inv_q30_mult(xx, sqrt2);

    return xx;
}

/** Calculates 1/x of a fixed-point number (30 bit mantissa)
* Input must be a scaled (2^30) integer (+/-)
* The number is scaled to lie between a range in which a Newton-Raphson
* iteration works best. Corresponding multiplier power of two is returned.
*  Caller must scale final result by 2^pow (while avoiding overflow).
* @param[in] x, length 1
* @param[out] pow, length 1
* @return scaledOneOverX on success or zero.
*/
long inv_one_over_x(long x0, int*pow)
{
    //% NR for 1/x in the neighborhood of log(2) => x => log(4)
    //%    y(k+1)=y(k)*(2 – x0*y(k))
    //% with y(0) = 1 as the starting value for NR

    long two, xx;
    int numberwasnegative, nr_iters, did_upscale, did_downscale;

    long upscale, downscale, upsclimit, downsclimit;

    *pow = 0;

    // Return if input is zero.
    if (x0 == 0L)
    {
        return 0L;
    }

    // This is really (2^31-1), i.e. 1.99999... .
    // Approximation error is 1e-9. Note 2^31 will overflow to sign bit, so it can't be used here.
    two = 2147483647L;

    // int32(0.92/log(2)*2^30)
    upscale = 1425155444L;
    // int32(1.08/upscale*2^30)
    upsclimit = 873697834L;

    // int32(1.08/log(4)*2^30)
    downscale = 836504283L;
    // int32(0.92/downscale*2^30)
    downsclimit = 1268000423L;

    // Algorithm is intended to work with positive numbers only. Change sign:
    numberwasnegative = 0;

    if (x0 < 0L)
    {
        numberwasnegative = 1;
        x0 = -x0;
    }

    nr_iters = test_limits_and_scale(&x0, pow);

    did_upscale = 0;
    did_downscale = 0;

    // Pre-scaling to reduce NR iterations and improve accuracy:
    if (x0 <= upsclimit)
    {
        x0 = inv_q30_mult(x0, upscale);
        did_upscale = 1;
        // The scaling ALWAYS leaves the number in the 2-NR iterations region:
        nr_iters = 2;

        // Is x0 in the single NR iteration region (0.994, 1.006) ?
        if (x0 > 1067299373 && x0 < 1080184275)
            nr_iters = 1;
    }
    else if (x0 >= downsclimit)
    {
        x0 = inv_q30_mult(x0, downscale);
        did_downscale = 1;
        // The scaling ALWAYS leaves the number in the 2-NR iterations region:
        nr_iters = 2;

        // Is x0 in the single NR iteration region (0.994, 1.006) ?
        if (x0 > 1067299373 && x0 < 1080184275)
            nr_iters = 1;
    }

    xx = (two - x0) + 1; // Note 2 will overflow so the computation (2-x) is done with "two" == (2^30-1)
    // First NR iteration
    xx = inv_q30_mult(xx, (two - inv_q30_mult(x0, xx)) + 1);

    if (nr_iters >= 2)
    {
        // Second NR iteration
        xx = inv_q30_mult(xx, (two - inv_q30_mult(x0, xx)) + 1);

        if (nr_iters == 3)
        {
            // THird NR iteration.
            xx = inv_q30_mult(xx, (two - inv_q30_mult(x0, xx)) + 1);
            // Fourth NR iteration: Not needed due to single precision.
        }
    }

    // Post-scaling
    if (did_upscale)
        xx = inv_q30_mult(xx, upscale);
    else if (did_downscale)
        xx = inv_q30_mult(xx, downscale);

    if (numberwasnegative)
        xx = -xx;

    return xx;
}

/** Auxiliary function used by inv_OneOverX(), inv_fastSquareRoot(), inv_inverseSqrt().
* Finds the range of the argument, determines the optimal number of Newton-Raphson
* iterations and . Corresponding square root of the power of two is returned.
* Restrictions: Number is represented as Q1.30.
*               Number is betweeen the range 2<x<=0
* @param[in] x, length 1
* @param[out] pow, length 1
* @return # of NR iterations, x0 scaled between log(2) and log(4) and 2^N scaling (N=pow)
*/
int test_limits_and_scale(long *x0, int *pow)
{
    long lowerlimit, upperlimit, oneiterlothr, oneiterhithr, zeroiterlothr, zeroiterhithr;

    // Lower Limit: ll = int32(log(2)*2^30);
    lowerlimit = 744261118L;
    //Upper Limit ul = int32(log(4)*2^30);
    upperlimit = 1488522236L;
    //  int32(0.9*2^30)
    oneiterlothr = 966367642L;
    // int32(1.1*2^30)
    oneiterhithr = 1181116006L;
    // int32(0.99*2^30)
    zeroiterlothr = 1063004406L;
    //int32(1.01*2^30)
    zeroiterhithr = 1084479242L;

    // Scale number such that Newton Raphson iteration works best:
    // Find the power of two scaling that leaves the number in the optimal range,
    // ll <= number <= ul. Note odd powers have special scaling further below
    if (*x0 > upperlimit)
    {
        // Halving the number will push it in the optimal range since largest value is 2
        *x0 = *x0 >> 1;
        *pow = -1;
    }
    else if (*x0 < lowerlimit)
    {
        // Find position of highest bit, counting from left, and scale number
        *pow = get_highest_bit_position((unsigned long*)x0);

        if (*x0 >= upperlimit)
        {
            // Halving the number will push it in the optimal range
            *x0 = *x0 >> 1;
            *pow = *pow - 1;
        }
        else if (*x0 < lowerlimit)
        {
            // Doubling the number will push it in the optimal range
            *x0 = *x0 << 1;
            *pow = *pow + 1;
        }
    }
    else
    {
        *pow = 0;
    }

    if (*x0 < oneiterlothr || *x0 > oneiterhithr)
        return 3; // 3 NR iterations

    if (*x0 < zeroiterlothr || *x0 > zeroiterhithr)
        return 2; // 2 NR iteration

    return 1; // 1 NR iteration
}

/** Auxiliary function used by testLimitsAndScale()
* Find the highest nonzero bit in an unsigned 32 bit integer:
* @param[in] value, length 1.
* @return highes bit position.
**/int get_highest_bit_position(unsigned long *value)
{
    int position;
    position = 0;

    if (*value == 0) return 0;

    if ((*value & 0xFFFF0000) == 0)
    {
        position += 16;
        *value = *value << 16;
    }

    if ((*value & 0xFF000000) == 0)
    {
        position += 8;
        *value = *value << 8;
    }

    if ((*value & 0xF0000000) == 0)
    {
        position += 4;
        *value = *value << 4;
    }

    if ((*value & 0xC0000000) == 0)
    {
        position += 2;
        *value = *value << 2;
    }

    // If we got too far into sign bit, shift back. Note we are using an
    // unsigned long here, so right shift is going to shift all the bits.
    if ((*value & 0x80000000))
    {
        position -= 1;
        *value = *value >> 1;
    }

    return position;
}


long inv_fastsine29(long x)
{
    // http://devmaster.net/forums/topic/4648-fast-and-accurate-sinecosine/
    // Method is based on a parabola fit for the sine function. Q, P are chosen to minimize error.
    // Input argument, constants and calculations are Q29. Result is promoted to Q30 before return.
    // Argument (scaled Q29) must reside between (-pi, pi).
    long two_over_pi = 341782638L; // int32(2/pi*2^29)
    long Q = 416074957L; // = int32(0.775*2^29)
    long P = 120795955L; // = int32(0.225*2^29)
    long small_angle = 100448548L; // = int32(0.1871*2^29) Angle threshold for better accuracy, about 10.7 deg.
    long y, absx;

    // Early cop-out for smaller angles:
    if (ABS(x) < small_angle)
        return x << 1; // Promote to 30 bits. Note this is a signed-int shift.

    // First iteration
    // y = x*(B  - C * abs(x))  Scale. As a result B=2, C=1.
    // y = x*(2 - abs(x)) after scaling.
    x = inv_q29_mult(x, two_over_pi); //
    absx = ABS(x);
    y = inv_q29_mult(x, (2L << 29) - absx);
    // Second iteration
    //  %y = Q * y + P * y * abs(y) = y *(Q + P*abs(y))
    y = inv_q29_mult(y, Q + inv_q29_mult(P, ABS(y)));

    return y << 1;// Promote to 30 bits. Note this is a signed-int shift.
}

long inv_fastcosine29(long x)
{
    // Input argument, constants and calculations are Q29. Result is promoted to Q30 before return
    long pi_over_2 = 843314857L;

    if (x < 0)
        x = x + pi_over_2;
    else
        x = pi_over_2 - x;

    return inv_fastsine29(x);
}

/* compute real part of quaternion, element[0]
@param[in]  inQuat, 3 elements gyro quaternion
@param[out] outquat, 4 elements gyro quaternion
*/
int inv_compute_scalar_part(const long * inQuat, long* outQuat)
{
    #if 1
    long scalarPart = 0;

    scalarPart = inv_fast_sqrt((1L << 30) - inv_q30_mult(inQuat[0], inQuat[0])
                               - inv_q30_mult(inQuat[1], inQuat[1])
                               - inv_q30_mult(inQuat[2], inQuat[2]));
    outQuat[0] = scalarPart;
    outQuat[1] = inQuat[0];
    outQuat[2] = inQuat[1];
    outQuat[3] = inQuat[2];

    #else

    double scalarPart = 0.0;
    double t0, t1, t2;
    double norm = 0.0;
    double dQuat[4];

    t0 = (double)inQuat[0] / (double)(1L << 30);
    t1 = (double)inQuat[1] / (double)(1L << 30);
    t2 = (double)inQuat[2] / (double)(1L << 30);

    scalarPart = sqrt(1.0 - (t0 * t0) - (t1 * t1) - (t2 * t2));

    norm = sqrtf(scalarPart * scalarPart + t0 * t0 + t1 * t1 + t2 * t2);
    dQuat[0] = scalarPart / norm;
    dQuat[1] = t0 / norm;
    dQuat[2] = t1 / norm;
    dQuat[3] = t2 / norm;

    outQuat[0] = (long)(dQuat[0] * (1L << 30));
    outQuat[1] = (long)(dQuat[1] * (1L << 30));
    outQuat[2] = (long)(dQuat[2] * (1L << 30));
    outQuat[3] = (long)(dQuat[3] * (1L << 30));

    #endif

    return 0;
}

/**
* Computes where a quaternion should be given only accel data.
* Minimum rotation about Z is done
* @param[in] accel Accel in Body Frame
* @param[out] quat quaternion
*/
void inv_compute_quat_from_accel(const long *accel, long *quat)
{
    float mg;
    float cs, sn, csd;

    /* We are trying to find the minimal rotation to force the accel Z axes to
    be aligned with gravity, which would define our body to world quaternion rotation.
    The unit vector we should rotate about is computed by taking the cross product
    of accel with the Z-body axis [0,0,1], which gives us [ay,-ax,0]. The angle to
    rotate about is atan of the magnitude of the x,y components with z
    * This gives:
    mg = sqrtf((float)accel[0]*accel[0]+(float)accel[1]*accel[1]);
    ang1 = atan2f(mg,accel[2]);
    cs = cosf(ang1/2.f);
    sn = sinf(ang1/2.f);

    Next we use the half angle formulas to avoid trig.
    cosine(ang1) = accel[2] / magnitude(accel);
    To compute cs and sn above.
    */
    mg = sqrtf((float)accel[0] * accel[0] +
               (float)accel[1] * accel[1] + (float)accel[2] * accel[2]);

    if (mg <= 1)
    {
        quat[0] = 1L << 30;
        quat[1] = 0;
        quat[2] = 0;
        quat[3] = 0;
    }
    else
    {

        csd = accel[2] / mg;
        sn = sqrtf(MAX((1.f - csd), 0) / 2.f);
        cs = sqrtf(MAX((1.f + csd), 0) / 2.f);
        mg = sqrtf((float)accel[0] * accel[0] + (float)accel[1] * accel[1]);

        if (mg <= 1)
        {
            quat[0] = 1L << 30;
            quat[1] = 0;
            quat[2] = 0;
            quat[3] = 0;
        }
        else
        {
            quat[0] = (long)(cs * (1L << 30));
            quat[1] = (long)(sn * (accel[1] / mg) * (1L << 30));
            quat[2] = (long)(-sn * (accel[0] / mg) * (1L << 30));
            quat[3] = 0;
        }
    }
}

long inv_q30_scalar_product(const long x[3], const long y[3])
{
    return inv_q30_mult(x[0], y[0]) + inv_q30_mult(x[1], y[1]) + inv_q30_mult(x[2], y[2]);
}

//=====================================================================//
// FIXED POINT MATH FUNCTIONS IN Q15 AND Q30
//
// Q15 MULTIPLY FUNCTION
/** Performs a multiply and shift by 15.
* This function is to be moved to ml_math_func.c
* @param[in] a
* @param[in] b
* @return ((long long)a*b)>>15
*/
long inv_q15_mult(long a, long b)
{
    long out = (long)(((long long)a * (long long)b) >> 15);
    return out;
}

//% 1/sqrt(x)
//% x(k+1) = x(k)*(3 - x0*x(k)^2)
// THIS VERSION IMPLEMENTED FOR Q15 Fixed-point
//% Code scales number and performs NR in the neighborhood of 1.3>x>=0.65
long inverse_sqrt_q15(long x)
{
    long oneoversqrt2 = 23170L; // int32(2^15*1/sqrt(2))
    long oneandhalf = 49152L; // int32(1.5*2^15);
    long upperlimit = 45426; // int32(log(4)*2^15);
    long lowerlimit = 22713; // int32(log(2)*2^15);
    long xx, x0_2, invsqrtx;
    int pow2;

    if (x <= 0)
        return 0L;

    pow2 = 0;
    xx = x;

    if (xx > upperlimit)
    {
downscale:

        if (xx > upperlimit)
        {
            xx = xx / 2;
            pow2 = pow2 - 1;
            goto downscale;
        }

        goto newton_raphson;
    }

    if (xx < lowerlimit)
    {
upscale:

        if (xx < lowerlimit)
        {
            xx = xx * 2;
            pow2 = pow2 + 1;
            goto upscale;
        }

        goto newton_raphson;
    }

newton_raphson:
    // 3 NR iterations. In some cases second and/or third iteration may not be needed, however
    // for code simplicity always iterate three times. Fourth iteration is below bit precision.
    x0_2 = xx >> 1;
    xx = oneandhalf - x0_2;
    xx = inv_q15_mult(xx, (oneandhalf - inv_q15_mult(x0_2, inv_q15_mult(xx, xx))));
    xx = inv_q15_mult(xx, (oneandhalf - inv_q15_mult(x0_2, inv_q15_mult(xx, xx))));

    if (pow2 & 1)   // This checks if the number is even or odd.
    {
        pow2 = (pow2 >> 1) + 1; // Account for sqrt(2) in denominator
        invsqrtx = (inv_q15_mult(xx, oneoversqrt2));
    }
    else
    {
        pow2 = pow2 >> 1;
        invsqrtx = xx;
    }

    if (pow2 < 0)
        invsqrtx = invsqrtx >> ABS(pow2);
    else if (pow2 > 0)
        invsqrtx = invsqrtx << pow2;

    return invsqrtx;
}


//% Square-Root:
// This code calls 1/sqrt(x) and multiplies result with x, i.e. sqrt(x)=x*(1/sqrt(x).
// See the implementation of 1/sqrt(x) in inverse_sqrt_q15().
// THIS VERSION IMPLEMENTED FOR Q15 Fixed-point
long sqrt_fun_q15(long x)
{
    long sqrtx;

    if (x <= 0L)
    {
        sqrtx = 0L;
        return sqrtx;
    }

    sqrtx = inverse_sqrt_q15(x); // invsqrtx

    sqrtx = inv_q15_mult(x, sqrtx);

    return sqrtx;
}


//reciprocal_fun_q15:
// Reciprocal function based on Newton-Raphson 1/sqrt(x) calculation
// THIS VERSION IMPLEMENTED FOR Q15 Fixed-point
long reciprocal_fun_q15(long x)
{
    long y;
    int negx;

    if (x == 0)
    {
        y = 0L;
        return y;
    }

    negx = 0;

    if (x < 0)
    {
        x = -x;
        negx = 1;
    }

    if (x >= 1073741824L)   // 2^15 in Q15; underflow number
    {
        if (negx)
            y = -1L;
        else
            y = 1L;

        return y;
    }

    y = inverse_sqrt_q15(x); // sqrt(y)
    y = inv_q15_mult(y, y);

    if (negx)
        y = -y;

    return y;
}


// ATAN2() Chebychev polynomial approximation.
// For polynomial orders 3, 5, 7, use:
// constA3 = int32(2^15*[0.970562748477141, -0.189514164974601]); % 3rd order
// constA5 = int32(2^15*[0.994949366116654,-0.287060635532652,0.078037176446441]); %5th order
// constA7 = int32(2^15*[0.999133448222780 -0.320533292381664 0.144982490144465,-0.038254464970299]); %7th order
// AND  use corresponding multiplications
// 3rd   Z = q15_mult((constA3(1) + q15_mult(constA3(2), tmp2)), tmp);
// 5th:  Z = q15_mult((constA5(1) + q15_mult((constA5(2) + q15_mult(constA5(3), tmp2)), tmp2)), tmp);
// 7th:  Z = q15_mult((constA7(1) + ...
//           q15_mult((constA7(2) + ...
//           q15_mult((constA7(3) + ...
//           q15_mult(constA7(4), tmp2)), tmp2)), tmp2)), tmp);
// 3rd Order Accuracy is +/-0.3 degrees except near the origin (avoid when both args X and Y less than +/-0.03 simultaneously)
// 7th Order Accuracy is +/-0.02 degrees (worst case) through entire range (accomplished with scaling)
//
// This code depends on:
//    reciprocal_fun_q15()
//       inverse_sqrt_q15()
//    inv_q15_mult()
//
/** Third, fifth or seventh order Chebychev polynomial approximation in Q15 (or 5th order).
* @param[in] Y input of atan2(Y, X) in Q15
* @param[in] X input of atan2(Y, X) in Q15
* @param[out] output angle in radians, Q15
*/
long atan2_q15(long Y, long X)
{
    long absy, absx, maxABS, tmp, tmp2, tmp3, Z, angle;
    //static long constA3[2] = {31803,-6210};      //int32(2^15*[0.970562748477141, -0.189514164974601]); % 3rd order
    //static long constA5[3] = {32603,-9406,2557}; //int32(2^15*[0.994949366116654,-0.287060635532652,0.078037176446441]); %5th order
    static long constA7[4] = { 32740, -10503, 4751, -1254 }; // int32(2^15*[0.999133448222780 -0.320533292381664 0.144982490144465,-0.038254464970299]); %7th order
    static long PI15 = 102944; // int32(2^15*pi): pi in Q15

    absx = ABS(X);
    absy = ABS(Y);
    // 3rd order only:
    //if (absx<10000L && absy<10000L) { // args X and Y less than +/-0.03 simultaneously. Ill conditioned.
    //    return = 0L;
    //}
    // When one argument is zero:
    // These can be eliminated to save code space. Code works correctly
    // under these conditions
    //if (X == 0L) {
    //    if (Y == 0L)
    //        return 0L; // Undefined, but we return zero.
    //    if (Y > 0L)
    //        return PI15/2;
    //    if (Y < 0L)
    //        return -PI15/2;
    //}

    //if (Y == 0L) {
    //    if (X > 0L)
    //        return 0L;
    //    if (X < 0L)
    //        return -PI15;
    //}

    maxABS = MAX(absx, absy);

    // SCALE arguments down to protect from roundoff loss due to 1/x operation.
    //% Threshold for scaling found by numericaly simulating arguments
    //% to yield optimal (minimal) error of less than 0.01 deg through
    //% entire range (for Chebycheff order 7).
    //    while ( maxABS >> 13) {  --> Or it can be done this way if DMP code is more efficient
    while (maxABS > 8192L)
    {
        maxABS = maxABS / 2;
        absx = absx / 2;
        absy = absy / 2;
    }

    {
        if (absx >= absy) // (0, pi/4]: tmp = abs(y)/abs(x);
            tmp = inv_q15_mult(absy, reciprocal_fun_q15(absx));
        else             // (pi/4, pi/2): tmp = abs(x)/abs(y);
            tmp = inv_q15_mult(absx, reciprocal_fun_q15(absy));

        tmp2 = inv_q15_mult(tmp, tmp);
        // 3rd order
        // Z = inv_q15_mult((constA3[0] + inv_q15_mult(constA3[1], tmp2)), tmp);
        // 5th order
        // Z = inv_q15_mult((constA5[0] + inv_q15_mult((constA5[1] + inv_q15_mult(constA5[2], tmp2)), tmp2)), tmp);
        // 7th order
        //Z = inv_q15_mult((constA7[0] +
        //    inv_q15_mult((constA7[1] +
        //    inv_q15_mult((constA7[2] +
        //    inv_q15_mult( constA7[3], tmp2)), tmp2)), tmp2)), tmp);
        // Alternatively:
        tmp3 = inv_q15_mult(constA7[3], tmp2);
        tmp3 = inv_q15_mult(constA7[2] + tmp3, tmp2);
        tmp3 = inv_q15_mult(constA7[1] + tmp3, tmp2);
        Z = inv_q15_mult(constA7[0] + tmp3, tmp);

        if (absx < absy)
            Z = PI15 / 2 - Z;

        if (X < 0)   // second and third quadrant
        {
            if (Y < 0)
                Z = -PI15 + Z;
            else
                Z = PI15 - Z;
        }
        else   // fourth quadrant
        {
            if (Y < 0)
                Z = -Z;
        }

        angle = Z; // Note the result is angle in radians, expressed in Q15.
    }

    return angle;
}


long inv_fastsine_q15(long x)
{
    // http://devmaster.net/forums/topic/4648-fast-and-accurate-sinecosine/
    // Method is based on a parabola fit for the sine function. Q, P are chosen to minimize error.
    // Input argument, constants and calculations are Q15.
    // Argument (scaled Q15) must reside between (-pi, pi).
    // For more accurate results, use the Q29 version.
    long PI15 = 102944L;
    long two_over_pi = 20861L; // int32(2/pi*2^15)
    long Q = 25395L; // = int32(0.775*2^15)
    long P = 7373L; // = int32(0.225*2^15)
    long small_angle = 6131L; // = int32(0.1871*2^15) Angle threshold for better accuracy, about 10.7 deg.
    long y, absx;

    // Early cop-out for smaller angles:
    if (ABS(x) < small_angle)
        return x;

    // Fix for arguments outside (-pi, pi)
    if (ABS(x) > PI15)
        x = x % (2 * PI15);

    if (x > PI15)
        x = x - 2 * PI15;

    if (x < -102944L)
        x = x + 2 * PI15;

    // First iteration
    // y = x*(B  - C * abs(x))  Scale. As a result B=2, C=1.
    // y = x*(2 - abs(x)) after scaling.
    x = inv_q15_mult(x, two_over_pi); //
    absx = ABS(x);
    y = inv_q15_mult(x, (2L << 15) - absx);
    // Second iteration
    //  %y = Q * y + P * y * abs(y) = y *(Q + P*abs(y))
    y = inv_q15_mult(y, Q + inv_q15_mult(P, ABS(y)));

    return y;
}

long inv_fastcosine_q15(long x)
{
    long pi_over_2 = 51472L; // int32(pi/2*2^15)
    long PI15 = 102944L;

    // Fix for arguments larger than (-pi, pi)
    if (ABS(x) > PI15)
        x = x % (2 * PI15);

    if (x > PI15)
        x = x - 2 * PI15;

    if (x < -102944L)
        x = x + 2 * PI15;

    if (x < 0)
        x = x + pi_over_2;
    else
        x = pi_over_2 - x;

    return inv_fastsine_q15(x);
}


//====================================================
// Q30 Functions
//===================================================

long inverse_sqrt_q30(long x, int *pow2)
{
    long oneoversqrt2 = 759250125L; // int32(2^30*1/sqrt(2))
    long oneandhalf = 1610612736L; // int32(1.5*2^30);
    long upperlimit = 1488522236; // int32(log(4)*2^30);
    long lowerlimit = 744261118; // int32(log(2)*2^30);
    long xx, x0_2, invsqrtx;

    *pow2 = 0;

    if (x <= 0)
    {
        return 1L << 30;
    }

    xx = x;

    if (xx > upperlimit)
    {
downscale:

        if (xx > upperlimit)
        {
            xx = xx / 2;
            *pow2 = *pow2 - 1;
            goto downscale;
        }
    }

    if (xx < lowerlimit)
    {
upscale:

        if (xx < lowerlimit)
        {
            xx = xx * 2;
            *pow2 = *pow2 + 1;
            goto upscale;
        }
    }

    // 3 NR iterations. In some cases second and/or third iteration may not be needed, however
    // for code simplicity always iterate three times. Fourth iteration is below bit precision.
    x0_2 = xx >> 1;
    xx = oneandhalf - x0_2;
    xx = inv_q30_mult(xx, (oneandhalf - inv_q30_mult(x0_2, inv_q30_mult(xx, xx))));
    xx = inv_q30_mult(xx, (oneandhalf - inv_q30_mult(x0_2, inv_q30_mult(xx, xx))));

    if (*pow2 & 1)   // This checks if the number is even or odd.
    {
        *pow2 = (*pow2 >> 1) + 1; // Account for sqrt(2) in denominator
        invsqrtx = (inv_q30_mult(xx, oneoversqrt2));
    }
    else
    {
        *pow2 = *pow2 >> 1;
        invsqrtx = xx;
    }

    return invsqrtx;
}


//% Square-Root:
// This code calls 1/sqrt(x) and multiplies result with x, i.e. sqrt(x)=x*(1/sqrt(x).
// See the implementation of 1/sqrt(x) in inverse_sqrt_q15().
// THIS VERSION IMPLEMENTED FOR Q15 Fixed-point
long sqrt_fun_q30(long x)
{
    long sqrtx;
    int pow2;

    if (x <= 0L)
    {
        sqrtx = 0L;
        return sqrtx;
    }

    sqrtx = inverse_sqrt_q30(x, &pow2); // invsqrtx

    sqrtx = inv_q30_mult(x, sqrtx);

power_up:

    if (pow2 > 0)
    {
        sqrtx = 2 * sqrtx;
        pow2 = pow2 - 1;
        goto power_up;
    }

power_down:

    if (pow2 < 0)
    {
        sqrtx = sqrtx / 2;
        pow2 = pow2 + 1;
        goto power_down;
    }

    return sqrtx;
}

/*reciprocal_fun_q30:
Reciprocal function based on Newton-Raphson 1/sqrt(x) calculation
THIS VERSION IMPLEMENTED FOR Q30 Fixed-point
INPUTS
b - the q30 number we are trying to divide by

OUTPUTS
c - the 1/b result in q30 downshifted by pow2:
pow2 - a power of 2 by which 1/b is downshifted to fit in q30.

LIMITS
Note that upshifting c by pow2 right away will overflow q30 if b<0.5 in q30 (=536870912)
so if you are doing some multiplication later on (like a/b), then it might be better
to do q30_mult(a,c) first and then shift it up by pow2: q30_mult(a,c)<<pow2
The result might still overflow in some cases (large a, small b: a=1073741824, b=1
but precise limits of the overflow are tbd).

EXAMPLE
//we want to find a/b, where a and b are very small. Hence 1/b will
//definitely overflow q30, but downshifted 1/b won't. So just remember
//to multiply 1/b by a first and only then upshift by pow2:
long a = 1024; //1e-6 in q30
long b = 2048; //2e-6 in q30
long c; //the scaled result of 1/b in pow2 format
int pow2; //exponent storage
long d; //the result of a/b (in this example should be 0.5 in q30)
c = reciprocal_fun_q30( b, &pow2 ); //c=536870912, pow2=20 (ie .5*2^30*2^20, much greater than q30 limit)
d = inv_q30_mult(a,c) << pow2; //as expected d=536870912 (ie 0.5 in q30)
*/
long reciprocal_fun_q30(long x, int *pow2)
{
    long y;
    int negx;

    if (x == 0)
    {
        y = 0L;
        *pow2 = 0;
        return y;
    }

    negx = 0;

    if (x < 0)
    {
        x = -x;
        negx = 1;
    }

    y = inverse_sqrt_q30(x, pow2); // sqrt(y)
    y = inv_q30_mult(y, y);
    *pow2 = *pow2 * 2;  // Must double exponent due to multiply

    if (negx)
        y = -y;

    return y;
}

// IFR: Q30 sqrt function, Joe's implementation
//============== sqrt(x) Function ================================
/** Calculates square-root of positive integer number (32 bit) using abacus method
* Input must be a positive integer
* Return is sqrt_x * 2^15
*		if x represents an integer	then sqrt_x is a Q15
*		if x represents a Q30		then sqrt_x is a Q30
*		if x represents a Q2*N		then sqrt_x is a Q(N+15)
* Example:	for x = 5 then sqrt_x = 73271.
*			RoundingError = sqrt(5) - (73271./2^15) = 1.4e-5
*/
unsigned long sqrt_abacus_q30_q30(unsigned long x_u32)
{
    unsigned long op;
    unsigned long one;
    unsigned long sqrt_x_q15 = 0U;
    op = x_u32;
    one = 1073741824U;

    while (one > op)
    {
        one >>= 1;
        op <<= 1;
    }

    op >>= 1;
    one >>= 1;

    while ((unsigned long)one != 0)
    {
        if (op >= sqrt_x_q15 + one)
        {
            op = (op - sqrt_x_q15) - one;
            sqrt_x_q15 += one << 1;
        }

        op <<= 1;
        one >>= 1;
    }

    return sqrt_x_q15;
}

// IFR: Q15 inverse sqrt function, Joe's implementation
unsigned long inverse_sqrt_abacus_q30_q15(unsigned long x_u32)
{
    if (x_u32 == 0)
        return 0;
    else
        return (unsigned long)((((long long)1U << 60) / (long long)sqrt_abacus_q30_q30(x_u32)) >> 15);
}

// Division algorithm. Divide num by den.
// Algorithm retains full precision by  intelligent scaling.
// Precision loss for very small arguments is avoided.
// The result is scaled by some power-of-two. Caller needs to handle this.
// Inputs are in the range [-2, 2).
// Output is N*2^e where N is in the range [-2, 2). Number is scaled with exponent e.
long num_over_den_q30(long num, long den, int *pow2)
{
    long oneoverDen;
    int neg;
    long one = 1L << 30; // int32(2^31-1) to avoid overflow
    long onehalf = 536870912L; // int32(2^29)

    *pow2 = 0;

    if (den == 0)    // Return with zero.
    {
        return 0L;
    }

    neg = 0;

    if (num < 0 && den > 0)
    {
        neg = 1;
        num = -num;
    }
    else if (num > 0 && den < 0)
    {
        neg = 1;
        den = -den;
    }

    if (den == one)    // Return with zero.
    {
        return num;
    }

    oneoverDen = reciprocal_fun_q30(den, pow2);

    while (num <= onehalf)
    {
        num = num << 1;
        *pow2 = *pow2 - 1;
    }

    while (num > onehalf)
    {
        num = num >> 1;
        *pow2 = *pow2 + 1;
    }

    num = inv_q30_mult(num, oneoverDen);
    // The above while loops guarantee the result of
    // the division will be always less than One.

    if (neg)
        num = -num;

    return num;
}

/**
* Converts a rotation matrix to a quaternion.
* @param[in] Rcb Rotation matrix q30. The
*             First 3 elements of the rotation matrix, represent
*             the first row of the matrix.
* @param[out] Qcb_fp 4-element quaternion in fixed point. One is 2^30.
*/
void inv_rotation_to_quaternion_fxp(long *Rcb, long *Qcb)
{
    long r11, r12, r13, r21, r22, r23, r31, r32, r33;
    long temp[3];
    long tmp;
    int pow2, shift;

    r11 = Rcb[0] >> 1; //assume matrix is stored row wise first, that is rot[1] is row 1, col 2
    r12 = Rcb[1] >> 1;
    r13 = Rcb[2] >> 1;

    r21 = Rcb[3] >> 1;
    r22 = Rcb[4] >> 1;
    r23 = Rcb[5] >> 1;

    r31 = Rcb[6] >> 1;
    r32 = Rcb[7] >> 1;
    r33 = Rcb[8] >> 1;

    //Qcb[0] = (1.f + r11 + r22 + r33) / 4.f;
    //Qcb[1] = (1.f + r11 - r22 - r33) / 4.f;
    //Qcb[2] = (1.f - r11 + r22 - r33) / 4.f;
    //Qcb[3] = (1.f - r11 - r22 + r33) / 4.f;
    Qcb[0] = (268435456L + (r11 >> 1) + (r22 >> 1) + (r33 >> 1)); // Effectively shifted by 2 bits, one above, one here
    Qcb[1] = (268435456L + (r11 >> 1) - (r22 >> 1) - (r33 >> 1));
    Qcb[2] = (268435456L - (r11 >> 1) + (r22 >> 1) - (r33 >> 1));
    Qcb[3] = (268435456L - (r11 >> 1) - (r22 >> 1) + (r33 >> 1));

    if (Qcb[0] < 0L) Qcb[0] = 0L;

    if (Qcb[1] < 0L) Qcb[1] = 0L;

    if (Qcb[2] < 0L) Qcb[2] = 0L;

    if (Qcb[3] < 0L) Qcb[3] = 0L;

    if (Qcb[0] == 0L && Qcb[1] == 0L && Qcb[2] == 0L && Qcb[3] == 0L)
    {
        Qcb[0] = 1L << 30;
        return;
    }

    //Qcb[0] = sqrt(Qcb[0]);
    //Qcb[1] = sqrt(Qcb[1]);
    //Qcb[2] = sqrt(Qcb[2]);
    //Qcb[3] = sqrt(Qcb[3]);
    Qcb[0] = sqrt_fun_q30(Qcb[0]);
    Qcb[1] = sqrt_fun_q30(Qcb[1]);
    Qcb[2] = sqrt_fun_q30(Qcb[2]);
    Qcb[3] = sqrt_fun_q30(Qcb[3]);

    if (Qcb[0] >= Qcb[1] && Qcb[0] >= Qcb[2] && Qcb[0] >= Qcb[3]) //Qcb[0] is max
    {
        tmp = reciprocal_fun_q30(Qcb[0], &pow2);
        shift = 30 - pow2 + 1;
        Qcb[1] = (long)(((long long)(r23 - r32) * tmp) >> shift);
        Qcb[2] = (long)(((long long)(r31 - r13) * tmp) >> shift);
        Qcb[3] = (long)(((long long)(r12 - r21) * tmp) >> shift);
        //Qcb[1] = (r23 - r32)/(4.f*Qcb[0]);
        //Qcb[2] = (r31 - r13)/(4.f*Qcb[0]);
        //Qcb[3] = (r12 - r21)/(4.f*Qcb[0]);
    }
    else if (Qcb[1] >= Qcb[0] && Qcb[1] >= Qcb[2] && Qcb[1] >= Qcb[3]) //Qcb[1] is max
    {
        tmp = reciprocal_fun_q30(Qcb[1], &pow2);
        shift = 30 - pow2 + 1;
        Qcb[0] = (long)(((long long)(r23 - r32) * tmp) >> shift);
        Qcb[2] = (long)(((long long)(r12 + r21) * tmp) >> shift);
        Qcb[3] = (long)(((long long)(r31 + r13) * tmp) >> shift);
        // Qcb[0] = (r23 - r32)/(4.f*Qcb[1]);
        // Qcb[1] = Qcb[1];
        // Qcb[2] = (r12 + r21)/(4.f*Qcb[1]);
        // Qcb[3] = (r31 + r13)/(4.f*Qcb[1]);
    }
    else if (Qcb[2] >= Qcb[0] && Qcb[2] >= Qcb[1] && Qcb[2] >= Qcb[3]) //Qcb[2] is max
    {
        tmp = reciprocal_fun_q30(Qcb[2], &pow2);
        shift = 30 - pow2 + 1;
        Qcb[0] = (long)(((long long)(r31 - r13) * tmp) >> shift);
        Qcb[1] = (long)(((long long)(r12 + r21) * tmp) >> shift);
        Qcb[3] = (long)(((long long)(r23 + r32) * tmp) >> shift);
        //Qcb[0] = (r31 - r13)/(4.f*Qcb[2]);
        //Qcb[1] = (r12 + r21)/(4.f*Qcb[2]);
        //Qcb[2] = Qcb[2];
        //Qcb[3] = (r23 + r32)/(4.f*Qcb[2]);
    }
    else if (Qcb[3] >= Qcb[0] && Qcb[3] >= Qcb[1] && Qcb[3] >= Qcb[2]) //Qcb[3] is max
    {
        tmp = reciprocal_fun_q30(Qcb[3], &pow2);
        shift = 30 - pow2 + 1;
        Qcb[0] = (long)(((long long)(r12 - r21) * tmp) >> shift);
        Qcb[1] = (long)(((long long)(r31 + r13) * tmp) >> shift);
        Qcb[2] = (long)(((long long)(r23 + r32) * tmp) >> shift);
        //Qcb[0] = (r12 - r21)/(4.f*Qcb[3]);
        //Qcb[1] = (r31 + r13)/(4.f*Qcb[3]);
        //Qcb[2] = (r23 + r32)/(4.f*Qcb[3]);
        //Qcb[3] = Qcb[3];
    }
    else
    {
        // printf('coding error\n'); //error
        Qcb[0] = 1L << 30;
        Qcb[1] = 0L;
        Qcb[2] = 0L;
        Qcb[3] = 0L;
        return;
    }

    // Normalize
    // compute inverse square root, using first order taylor series
    // Here temp aligns with row 8
    temp[1] = (long)(((long long)Qcb[0] * Qcb[0] +
                      (long long)Qcb[1] * Qcb[1] +
                      (long long)Qcb[2] * Qcb[2] +
                      (long long)Qcb[3] * Qcb[3]) >> 30);
    temp[2] = temp[1] >> 1; // Multiply by 2^29
    temp[0] = (1L << 30) + (1L << 29) - temp[2];

    // Normalize
    Qcb[0] = inv_q30_mult(temp[0], Qcb[0]);
    Qcb[1] = inv_q30_mult(temp[0], Qcb[1]);
    Qcb[2] = inv_q30_mult(temp[0], Qcb[2]);
    Qcb[3] = inv_q30_mult(temp[0], Qcb[3]);
}

/**
* Does triad in NED convention
*/
void inv_triad2(const long *accel_body, const long *compass_body, long accel_fs, long *Qbi_fp)
{
    float acc[3], an, mag[3], mn, X[3], Yn, Y[3], Z[3], O_BI[9];

    acc[0] = (float)accel_body[0] / ((float)(1 << (14 + accel_fs))); //get accel into float g's, ie divide RawScaled by 2^(14+11) in 2g setting or 26 in 4g, etc. 14 comes from basic raw definition: 2g=2^15lsb
    acc[1] = (float)accel_body[1] / ((float)(1 << (14 + accel_fs)));
    acc[2] = (float)accel_body[2] / ((float)(1 << (14 + accel_fs)));

    mag[0] = (float)compass_body[0] / ((float)(1 << 16)); //get compass into float uT's, ie divide RawScaled by 2^16
    mag[1] = (float)compass_body[1] / ((float)(1 << 16));
    mag[2] = (float)compass_body[2] / ((float)(1 << 16));

    //Z = accel/norm(accel);
    an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]); //acc norm
    Z[0] = acc[0] / an;
    Z[1] = acc[1] / an;
    Z[2] = acc[2] / an;

    //compass = compass/norm(compass);
    mn = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]); //mag norm
    mag[0] = mag[0] / mn;
    mag[1] = mag[1] / mn;
    mag[2] = mag[2] / mn;

    //Y = cross(Z, compass);
    Y[0] = Z[1] * mag[2] - Z[2] * mag[1];
    Y[1] = Z[2] * mag[0] - Z[0] * mag[2];
    Y[2] = Z[0] * mag[1] - Z[1] * mag[0];

    //Y = Y/norm(Y);
    Yn = sqrtf(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]); //X norm
    Y[0] = Y[0] / Yn;
    Y[1] = Y[1] / Yn;
    Y[2] = Y[2] / Yn;

    //X = cross(Y, Z);
    X[0] = Y[1] * Z[2] - Y[2] * Z[1];
    X[1] = Y[2] * Z[0] - Y[0] * Z[2];
    X[2] = Y[0] * Z[1] - Y[1] * Z[0];

    //O = [X, Y, Z];
    O_BI[0] = X[0];
    O_BI[1] = Y[0];
    O_BI[2] = Z[0];
    O_BI[3] = X[1];
    O_BI[4] = Y[1];
    O_BI[5] = Z[1];
    O_BI[6] = X[2];
    O_BI[7] = Y[2];
    O_BI[8] = Z[2];

    //q = O2q(O);
    inv_rotation_to_quaternion(O_BI, Qbi_fp);
}


/**
* Does triad in NED convention, fixed point
*/
void inv_triad_fp(const long *accel, const long *compass, long *quat)
{
    //if (norm(accel)==0 || norm(compass)==0 )%|| norm(accel)> 1.1)
    //    q=qCurrent;
    //    return
    //end
    //
    //Z = accel/norm(accel);
    //compass = compass/norm(compass);
    //
    //X = cross(compass, Z);
    //X = X/norm(X);
    //Y = cross(Z, X);
    //
    //R = [X, Y, Z]';
    //q = dcmToQuat(R);

    long tmp;
    long R[9];
    long *X, *Y, *Z;
    int shift, pow2;

    X = R;
    Y = (long*)(R + 3);
    Z = (long*)(R + 6);

    // Normalize
    tmp = (long)(((long long)accel[0] * accel[0] +
                  (long long)accel[1] * accel[1] +
                  (long long)accel[2] * accel[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        Z[0] = (long)(((long long)accel[0] * tmp) >> shift);
        Z[1] = (long)(((long long)accel[1] * tmp) >> shift);
        Z[2] = (long)(((long long)accel[2] * tmp) >> shift);
    }
    else
        return;


    // Cross product Z x compass and get X
    // Cross Vert x compass --> North direction
    Y[0] = (long)(((long long)Z[1] * compass[2] - (long long)Z[2] * compass[1]) >> 30);
    Y[1] = (long)(((long long)Z[2] * compass[0] - (long long)Z[0] * compass[2]) >> 30);
    Y[2] = (long)(((long long)Z[0] * compass[1] - (long long)Z[1] * compass[0]) >> 30);

    tmp = (long)(((long long)Y[0] * Y[0] +
                  (long long)Y[1] * Y[1] +
                  (long long)Y[2] * Y[2]) >> 30);

    tmp = inverse_sqrt_q30(tmp, &pow2);

    if (tmp > 0)
    {
        shift = 30 - pow2;
        Y[0] = (long)(((long long)Y[0] * tmp) >> shift);
        Y[1] = (long)(((long long)Y[1] * tmp) >> shift);
        Y[2] = (long)(((long long)Y[2] * tmp) >> shift);
    }
    else
        return;

    // Cross product: YxZ and get X
    // Cross Vert x East -- > North direction
    // Result is unit norm since both vectors are normalized.
    X[0] = (long)(((long long)Y[1] * Z[2] - (long long)Y[2] * Z[1]) >> 30);
    X[1] = (long)(((long long)Y[2] * Z[0] - (long long)Y[0] * Z[2]) >> 30);
    X[2] = (long)(((long long)Y[0] * Z[1] - (long long)Y[1] * Z[0]) >> 30);

    inv_rotation_to_quaternion_fxp(R, quat);

    return;
}

/**
* @}
*/
