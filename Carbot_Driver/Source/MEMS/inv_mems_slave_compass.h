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
/** @defgroup	inv_mems_slave_compass	inv_slave_compass
    @ingroup 	Mems_driver
    @{
*/
#ifndef INV_MEMS_SLAVE_COMPASS_H_SDFWQN__
#define INV_MEMS_SLAVE_COMPASS_H_SDFWQN__

#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initializes the compass
* @return 	0 in case of success, -1 for any error
*/
int inv_setup_compass_akm(void);

/** @brief Self test for the compass
* @return 	0 in case of success, -1 for any error
*/
int inv_check_akm_self_test(void);

/** @brief Changes the scale of the compass
* @param[in] data  	new scale for the compass
* @return 	   		0 in case of success, -1 for any error
*/
int inv_write_akm_scale( int data);

/** @brief Reads the scale of the compass
* @param[out] scale  	pointer to recuperate the scale
* @return 	   			0 in case of success, -1 for any error
*/
int inv_read_akm_scale(int *scale);

/** @brief Stops the compass
* @return 	0 in case of success, -1 for any error
*/
int inv_suspend_akm(void);

/** @brief Starts the compass
* @return 	0 in case of success, -1 for any error
*/
int inv_resume_akm(void);

/** @brief Get compass power status
* @return 	1 in case compass is enabled, 0 if not started
*/
char inv_mems_compass_getstate(void);

/** @brief Calibrates the data
* @param[in] m  			pointer to the raw compass data  
* @param[out] compass_m 	pointer to the calibrated compass data
* @return 	   				0 in case of success, -1 for any error
*/
int inv_compass_dmp_cal(const signed char *m, const signed char *compass_m);

/**
* @brief Applies mounting matrix and scaling to raw compass data.
* @param[in] raw_data	 		Raw compass data
* @param[in] compensated_out   Compensated compass data
* @return 	   					0 in case of success, -1 for any error
*/
int inv_apply_raw_compass_matrix(short *raw_data, long *compensated_out);

#ifdef __cplusplus
}
#endif

#endif // INV_MEMS_SLAVE_COMPASS_H_SDFWQN__

/** @} */
