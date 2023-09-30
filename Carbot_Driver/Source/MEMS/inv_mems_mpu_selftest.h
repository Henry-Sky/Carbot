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


#ifndef INV_MEMS_SELF_TEST_H__
#define INV_MEMS_SELF_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#if (MEMS_CHIP == HW_ICM20648 || MEMS_CHIP == HW_ICM20609)
/**
*  @brief      Perform hardware self-test for Accel, Gyro and Compass.
*  @param[in]  None
*  @return     COMPASS_SUCESS<<2 | ACCEL_SUCCESS<<1 | GYRO_SUCCESS so 7 if all devices pass the self-test.
*/
int inv_mems_run_selftest(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // INV_MEMS_SELF_TEST_H__
