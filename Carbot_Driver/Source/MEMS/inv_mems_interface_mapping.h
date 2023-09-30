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
/** @defgroup inv_mems_interface_mapping inv_interface_mapping
	@ingroup Mems_dmp3
	@{
*/
#ifndef INV_MEMS_INTERFACE_MAPPING_H__DSFJSD__
#define INV_MEMS_INTERFACE_MAPPING_H__DSFJSD__

#include "mltypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

inv_error_t dmp_reset_odr_counters(void);

/** @brief Hook function to be implemented by at integration level to actually write to serial interface
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_reset_control_registers(void);

/** @brief Sets data output control register 1.
* @param[in] output_mask	Turns sensors on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*							DMP will also turn hw sensors on/off based on bits set in output_mask.
* @return 				0 on success, negative value on error.
*/
inv_error_t dmp_set_data_output_control1(int output_mask);

/** @brief Sets data output control register 2.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
* @return 				0 on success, negative value on error.
*/
inv_error_t dmp_set_data_output_control2(int output_mask);

/** @brief Sets data interrupt control register.
* @param[in] interrupt_ctl	Determines which sensors can generate interrupt according to following bit definition,
*							bit set indicates interrupt, bit clear indicates no interrupt.
* @return 				0 on success, negative value on error.
*/
inv_error_t dmp_set_data_interrupt_control(uint32_t interrupt_ctl);

/** @brief Sets FIFO watermark. DMP will send FIFO interrupt if FIFO count > FIFO watermark
* @param[in] fifo_wm	FIFO watermark set to 80% of actual FIFO size by default
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_FIFO_watermark(unsigned short fifo_wm);

/** @brief Sets data rdy status register.
* @param[in] data_rdy	Indicates which sensor data is available.
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_data_rdy_status(unsigned short data_rdy);

/** @brief Sets motion event control register.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
* @return 				0 on success, negative value on error.
*/
inv_error_t dmp_set_motion_event_control(unsigned short motion_mask);

/** @brief Sets sensor odr.
* @param[in] sensor		sensor number based on INV_SENSORS
* @param[in] divider	desired odr = base engine rate/(divider + 1)
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_sensor_rate(int sensor, short divider);

/** @brief Sets batch mode parameters.
* @param[in] thld	sets batch timeout in DMP ticks, e.g. batch 1 sec, thld= (1 sec * engine base rate in Hz)
* @param[in] mask	ties batch counter to engine specified with same bit definiton as HW register DATA_RDY_STATUS,
*					i.e. batch counter increments only if the engine specified is available in multi-rate setting
*	BIT 0 set: 1 - tie to gyro
*	BIT 1 set: 2 - tie to accel
*	BIT 2 set: 4 - tie to pressure in Diamond
*	BIT 3 set: 8 - tie to secondary
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_batchmode_params(unsigned int thld, short mask);

/** @brief Sets bias in DMP.
* @param[in] bias	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_bias(int *bias);

/** @brief Gets bias from DMP.
* @param[out] bias	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_get_bias(int *bias);

/** @brief Sets the gyro_sf used by quaternions on the DMP.
* @param[in] gyro_sf	value changes with gyro engine rate
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_gyro_sf(long gyro_sf);

/** @brief  Sets the accel gain used by accel quaternion on the DMP.
* @param[in] accel_gain		value changes with accel engine rate
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_accel_feedback_gain(int accel_gain);

/** @brief Sets accel cal parameters based on different accel engine rate/accel cal running rate
* @param[in] accel_cal
*	array is set as follows:
*	[0] = ACCEL_CAL_ALPHA_VAR
*	[1] = ACCEL_CAL_A_VAR
*   [2] = ACCEL_CAL_DIV - divider from hardware accel engine rate such that acce cal runs at accel_engine_rate/(d
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_accel_cal_params(int *accel_cal);

/** @brief Sets compass cal parameters based on different compass engine rate/compass cal running rate
* @param[in] compass_cal
*	array is set as follows:
*	[0] = CPASS_CAL_TIME_BUFFER
*	[1] = CPASS_CAL_ALPHA_VAR
*	[2] = CPASS_CAL_A_VAR
*	[3] = CPASS_CAL_RADIUS_3D_THRESH_ANOMALY
*	[4] = CPASS_CAL_NOMOT_VAR_THRESH
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_compass_cal_params(int *compass_cal);

/** @brief Sets compass orientation matrix to DMP.
* @param[in] compass_mtx 	matrix for the compass
* @return 				0 on success, negative value on error.
*/
inv_error_t dmp_set_compass_matrix(int *compass_mtx);

/** @brief Sets pedometer engine running rate.
* @param[in] ped_rate	divider based on accelerometer engine rate
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_pedometer_rate(int ped_rate);

/** @brief Enables/Disables the Tilt sensor
* @param[in] enable		0=off, 1=on
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_tilt_enable(unsigned short enable);

/** @brief Turns software wake on motion feature on/off.
* @param[in] enable		0=off, 1=on
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_wom_enable(unsigned char enable);

/** @brief Sets motion threshold to determine motion/no motion for wake on motion feature.
* @param[in] threshold	motion threshold
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_wom_motion_threshold(int threshold);

/** @brief Sets minimum time threshold of no motion before DMP goes to sleep.
* @param[in] threshold	time threshold
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_wom_time_threshold(unsigned short threshold);

/** @brief Gets pedometer step count.
* @param[out] steps		pointer on where data to be recuperated
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_get_pedometer_num_of_steps(unsigned long *steps);

/** @brief Sets mount matrix of accelerometer from chip to body frame for bring to see.
* @param[in] orientation 	array is set as follows (each value should be 0 or 1):
*	[0], [1], [2]
*	[3], [4], [5]
*	[6], [7], [8]
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_bts_accel_matrix(signed char *orientation);

/** @brief Loads the DMP firmware for the mems part.
* @param[in] dmp_image_sram      address to load DMP3 image from SRAM.
* @return 			0 on success, negative value on error.
*/
inv_error_t inv_load_firmware(const unsigned char *dmp_image_sram);

/** @brief Gets the start of the DMP firmware for the mems part.
* @param[in] dmp_cnfg	The configuration item
*/
void inv_get_dmp_start_address(unsigned short *dmp_cnfg);

/** @brief Sets the accelerometer scale
* @param[in] accel_fsr  To define the scale : The goal is to set 1g data to 2^25, 2g data to 2^26, etc.
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_accel_fsr(short accel_fsr);

/** @brief Sets the accelerometer scale at memory location ACC_SCALE2
* @param[in] accel_fsr  To define the scale : The goal is to set 1g data to 2^25, 2g data to 2^26, etc.
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_accel_scale2(short accel_fsr);

/** @brief Gets all steps for the pedometer
* @param[out] steps     pointer on where data to be stocked
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_get_pedometer_get_all_steps(unsigned long *steps);

/** @brief Updates the offset of step counter
* @param[in] steps  	new steps offset
* @return 			0 on success, negative value on error.
*/
inv_error_t inv_set_dmp_stepcounter_update_offset(unsigned long steps);

/** @brief  Resets the pick-up sensor
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_reset_pickup(void);

/** @brief  Clear BAC states when restarting BAC/SMD/Pedometer/Tilt to avoid false trigger
* @return 			0 on success, negative value on error.
* This avoids false triggering of BAC-related modules.
*/
inv_error_t dmp_reset_bac_states(void);

/** @brief  Set BAC decimation rate when restarting BAC/SMD/Pedometer/Tilt
* @param[in] accel_div  	accel smplrt_div value
* @return 			0 on success, negative value on error.
*/
inv_error_t dmp_set_bac_rate(short accel_div);
#ifdef __cplusplus
}
#endif
#endif // INV_INTERFACE_MAPPING_H__DSFJSD__

/** @} */
