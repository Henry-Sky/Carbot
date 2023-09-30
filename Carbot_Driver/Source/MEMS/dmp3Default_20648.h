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
#ifndef MEMS_20609

#ifndef _DMP_3_DEFAULT_20648_XFSD_H__
#define _DMP_3_DEFAULT_20648_XFSD_H__

#include "mltypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

inv_error_t inv_load_firmware_20648(const unsigned char *dmp_image_sram);
void inv_get_dmp_start_address_20648(unsigned short *dmp_cnfg);
inv_error_t dmp_reset_control_registers_20648(void);
inv_error_t dmp_set_data_output_control1_20648(int output_mask);
inv_error_t dmp_set_data_output_control2_20648(int output_mask);
inv_error_t dmp_set_data_interrupt_control_20648(uint32_t interrupt_ctl);
inv_error_t dmp_set_FIFO_watermark_20648(unsigned short fifo_wm);
inv_error_t dmp_set_data_rdy_status_20648(unsigned short data_rdy);
inv_error_t dmp_set_motion_event_control_20648(unsigned short motion_mask);
inv_error_t dmp_set_sensor_rate_20648(int sensor, short divider);
inv_error_t dmp_set_batchmode_params_20648(unsigned int thld, short mask);
inv_error_t dmp_set_bias_20648(int *bias);
inv_error_t dmp_get_bias_20648(int *bias);
inv_error_t dmp_set_gyro_sf_20648(long gyro_sf);
inv_error_t dmp_set_accel_feedback_gain_20648(int accel_gain);
inv_error_t dmp_set_accel_cal_params_20648(int *accel_cal);
inv_error_t dmp_set_compass_cal_params_20648(int *compass_cal);
inv_error_t dmp_set_compass_matrix_20648(int *compass_mtx);
inv_error_t dmp_get_pedometer_num_of_steps_20648(unsigned long *steps);
inv_error_t dmp_set_pedometer_rate_20648(int ped_rate);
inv_error_t dmp_set_wom_enable_20648(unsigned char enable);
inv_error_t dmp_set_wom_motion_threshold_20648(int threshold);
inv_error_t dmp_set_wom_time_threshold_20648(unsigned short threshold);
inv_error_t dmp_set_accel_fsr_20648(short accel_fsr);
inv_error_t dmp_set_accel_scale2_20648(short accel_fsr);
inv_error_t dmp_set_eis_auth_input_20648(long eis_auth_input);
inv_error_t dmp_get_eis_auth_output_20648(long *eis_auth_output);
inv_error_t dmp_set_bac_rate_20648(short accel_odr);
inv_error_t dmp_reset_bac_states_20648(void);
inv_error_t dmp_reset_odr_counters_20648(void);
inv_error_t dmp_reset_bac_states_20648(void);

#ifdef __cplusplus
}
#endif

// _DMP_3_DEFAULT_20648_XFSD_H__
#endif
#endif

