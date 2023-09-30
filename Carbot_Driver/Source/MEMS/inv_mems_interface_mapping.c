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

#include "inv_mems_interface_mapping.h"
#include "invn_types.h"


#include "inv_mems_hw_config.h"
#if defined MEMS_20630
    #include "dmp3Default_20630.h"
    #include "dmp3Driver.h"
#elif defined MEMS_30630
    #include "dmp3Default_30630.h"
    #include "dmp3Driver.h"
#elif defined MEMS_20645E
    #include "dmp3Default_20645E.h"
    #include "dmp3Driver.h"
#elif defined MEMS_20648
    #include "dmp3Default_20648.h"
    #include "dmp3Driver.h"
#elif defined MEMS_20609
    #include "dmp3Default_20608D.h"
    #include "dmp3Driver.h"
#else
    #error "Unsupported configuration"
#endif


// the step counter to be substracted
static unsigned long sStepCounterToBeSubtracted = 0;

// the old step
static unsigned long sOldSteps = 0;

inv_error_t dmp_reset_odr_counters()
{
    #if defined MEMS_20648
    return dmp_reset_odr_counters_20648();
    #elif defined MEMS_20609
    return dmp_reset_odr_counters_20609();
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_reset_control_registers()
{
    #if defined MEMS_20630
    return dmp_reset_control_registers_20630();
    #elif defined MEMS_30630
    return dmp_reset_control_registers_30630();
    #elif defined MEMS_20645E
    return dmp_reset_control_registers_20645E();
    #elif defined MEMS_20648
    return dmp_reset_control_registers_20648();
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_data_output_control1(int output_mask)
{
    #if defined MEMS_20630
    return dmp_set_data_output_control1_20630(output_mask);
    #elif defined MEMS_30630
    return dmp_set_data_output_control1_30630(output_mask);
    #elif defined MEMS_20645E
    return dmp_set_data_output_control1_20645E(output_mask);
    #elif defined MEMS_20648
    return dmp_set_data_output_control1_20648(output_mask);
    #elif defined MEMS_20609
    return dmp_set_dataout_control1_20609(output_mask);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_data_output_control2(int output_mask)
{
    #if defined MEMS_20630
    return dmp_set_data_output_control2_20630(output_mask);
    #elif defined MEMS_30630
    return  dmp_set_data_output_control2_30630(output_mask);
    #elif defined MEMS_20645E
    return  dmp_set_data_output_control2_20645E(output_mask);
    #elif defined MEMS_20648
    return dmp_set_data_output_control2_20648(output_mask);
    #elif defined MEMS_20609
    return dmp_set_dataout_control2_20609(output_mask);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_data_interrupt_control(uint32_t interrupt_ctl)
{
    #if defined MEMS_20630
    return dmp_set_data_interrupt_control_20630(interrupt_ctl);
    #elif defined MEMS_30630
    return dmp_set_data_interrupt_control_30630(interrupt_ctl);
    #elif defined MEMS_20645E
    return dmp_set_data_interrupt_control_20645E(interrupt_ctl);
    #elif defined MEMS_20648
    return dmp_set_data_interrupt_control_20648(interrupt_ctl);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_FIFO_watermark(unsigned short fifo_wm)
{
    #if defined MEMS_20630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_30630
    return dmp_set_FIFO_watermark_30630(fifo_wm);
    #elif defined MEMS_20645E
    return dmp_set_FIFO_watermark_20645E(fifo_wm);
    #elif defined MEMS_20648
    return dmp_set_FIFO_watermark_20648(fifo_wm);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_data_rdy_status(unsigned short data_rdy)
{
    #if defined MEMS_20630
    return dmp_set_data_rdy_status_20630(data_rdy);
    #elif defined MEMS_30630
    return dmp_set_data_rdy_status_30630(data_rdy);
    #elif defined MEMS_20645E
    return dmp_set_data_rdy_status_20645E(data_rdy);
    #elif defined MEMS_20648
    return dmp_set_data_rdy_status_20648(data_rdy);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_motion_event_control(unsigned short motion_mask)
{
    #if defined MEMS_20630
    return dmp_set_motion_event_control_20630(motion_mask);
    #elif defined MEMS_30630
    return dmp_set_motion_event_control_30630(motion_mask);
    #elif defined MEMS_20645E
    return dmp_set_motion_event_control_20645E(motion_mask);
    #elif defined MEMS_20648
    return dmp_set_motion_event_control_20648(motion_mask);
    #elif defined MEMS_20609
    return dmp_set_motion_interrupt_control_20609(motion_mask);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_sensor_rate(int sensor, short divider)
{
    #if defined MEMS_20630
    return dmp_set_sensor_rate_20630(sensor, divider);
    #elif defined MEMS_30630
    return dmp_set_sensor_rate_30630(sensor, divider);
    #elif defined MEMS_20645E
    return dmp_set_sensor_rate_20645E(sensor, divider);
    #elif defined MEMS_20648
    return dmp_set_sensor_rate_20648(sensor, divider);
    #elif defined MEMS_20609
    return dmp_set_sensor_rate_20609(sensor, divider);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_batchmode_params(unsigned int thld, short mask)
{
    #if defined MEMS_20630
    return dmp_set_batchmode_params_20630(thld, mask);
    #elif defined MEMS_30630
    return dmp_set_batchmode_params_30630(thld, mask);
    #elif defined MEMS_20645E
    return dmp_set_batchmode_params_20645E(thld, mask);
    #elif defined MEMS_20648
    return dmp_set_batchmode_params_20648(thld, mask);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_bias(int *bias)
{
    #if defined MEMS_20630
    return dmp_set_bias_20630(bias);
    #elif defined MEMS_30630
    return dmp_set_bias_30630(bias);
    #elif defined MEMS_20645E
    return dmp_set_bias_20645E(bias);
    #elif defined MEMS_20648
    return dmp_set_bias_20648(bias);
    #elif defined MEMS_20609
    return dmp_set_bias_20609(bias);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_get_bias(int *bias)
{
    #if defined MEMS_20630
    return dmp_get_bias_20630(bias);
    #elif defined MEMS_30630
    return dmp_get_bias_30630(bias);
    #elif defined MEMS_20645E
    return dmp_get_bias_20645E(bias);
    #elif defined MEMS_20648
    return dmp_get_bias_20648(bias);
    #elif defined MEMS_20609
    return dmp_get_bias_20609(bias);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_gyro_sf(long gyro_sf)
{
    #if defined MEMS_20630
    return dmp_set_gyro_sf_20630(gyro_sf);
    #elif defined MEMS_30630
    return dmp_set_gyro_sf_30630(gyro_sf);
    #elif defined MEMS_20645E
    return dmp_set_gyro_sf_20645E(gyro_sf);
    #elif defined MEMS_20648
    return dmp_set_gyro_sf_20648(gyro_sf);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_accel_feedback_gain(int accel_gain)
{
    #if defined MEMS_20630
    return dmp_set_accel_feedback_gain_20630(accel_gain);
    #elif defined MEMS_30630
    return dmp_set_accel_feedback_gain_30630(accel_gain);
    #elif defined MEMS_20645E
    return dmp_set_accel_feedback_gain_20645E(accel_gain);
    #elif defined MEMS_20648
    return dmp_set_accel_feedback_gain_20648(accel_gain);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_accel_cal_params(int *accel_cal)
{
    #if defined MEMS_20630
    return dmp_set_accel_cal_params_20630(accel_cal);
    #elif defined MEMS_30630
    return dmp_set_accel_cal_params_30630(accel_cal);
    #elif defined MEMS_20645E
    return dmp_set_accel_cal_params_20645E(accel_cal);
    #elif defined MEMS_20648
    return dmp_set_accel_cal_params_20648(accel_cal);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_compass_cal_params(int *compass_cal)
{
    #if defined MEMS_20630
    return dmp_set_compass_cal_params_20630(compass_cal);
    #elif defined MEMS_30630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20645E
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20648
    return dmp_set_compass_cal_params_20648(compass_cal);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_compass_matrix(int *compass_mtx)
{
    #if defined MEMS_20630
    return dmp_set_compass_matrix_20630(compass_mtx);
    #elif defined MEMS_30630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20645E
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20648
    return dmp_set_compass_matrix_20648(compass_mtx);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_pedometer_rate(int ped_rate)
{
    #if defined MEMS_20630
    return dmp_set_pedometer_rate_20630(ped_rate);
    #elif defined MEMS_30630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20645E
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20648
    return dmp_set_pedometer_rate_20648(ped_rate);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_tilt_enable(unsigned short enable)
{
    #if defined MEMS_20630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_30630
    return dmp_set_tilt_enable_30630(enable);
    #elif defined MEMS_20645E
    return dmp_set_tilt_enable_20645E(enable);
    #elif defined MEMS_20648
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_wom_enable(unsigned char enable)
{
    #if defined MEMS_20630
    return  dmp_set_wom_enable_20630(enable);
    #elif defined MEMS_30630
    return dmp_set_wom_enable_30630(enable);
    #elif defined MEMS_20645E
    return dmp_set_wom_enable_20645E(enable);
    #elif defined MEMS_20648
    return dmp_set_wom_enable_20648(enable);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_wom_motion_threshold(int threshold)
{
    #if defined MEMS_20630
    return  dmp_set_wom_motion_threshold_20630(threshold);
    #elif defined MEMS_30630
    return dmp_set_wom_motion_threshold_30630(threshold);
    #elif defined MEMS_20645E
    return dmp_set_wom_motion_threshold_20645E(threshold);
    #elif defined MEMS_20648
    return dmp_set_wom_motion_threshold_20648(threshold);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_wom_time_threshold(unsigned short threshold)
{
    #if defined MEMS_20630
    return  dmp_set_wom_time_threshold_20630(threshold);
    #elif defined MEMS_30630
    return dmp_set_wom_time_threshold_30630(threshold);
    #elif defined MEMS_20645E
    return dmp_set_wom_time_threshold_20645E(threshold);
    #elif defined MEMS_20648
    return dmp_set_wom_time_threshold_20648(threshold);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_get_pedometer_get_all_steps(unsigned long *steps)
{
    #if defined MEMS_20630
    return  dmp_get_pedometer_num_of_steps_20630(steps);
    #elif defined MEMS_30630
    return dmp_get_pedometer_num_of_steps_30630(steps);
    #elif defined MEMS_20645E
    return dmp_get_pedometer_num_of_steps_20645E(steps);
    #elif defined MEMS_20648
    return dmp_get_pedometer_num_of_steps_20648(steps);
    #elif defined MEMS_20609
    return dmp_get_pedometer_num_of_steps_20609(steps);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_get_pedometer_num_of_steps(unsigned long *steps)
{
    unsigned long lsteps = 0;
    inv_error_t result;

    result = dmp_get_pedometer_get_all_steps(&lsteps);

    // need to subtract the steps accumulated while Step Counter sensor is not active.
    *steps = lsteps - sStepCounterToBeSubtracted;
    sOldSteps = *steps;

    return result;
}

inv_error_t inv_load_firmware(const unsigned char *dmp_image_sram)
{
    #if defined MEMS_20630
    return  inv_load_firmware_20630(dmp_image_sram);
    #elif defined MEMS_30630
    return  inv_load_firmware_30630(dmp_image_sram);
    #elif defined MEMS_20645E
    return  inv_load_firmware_20645E(dmp_image_sram);
    #elif defined MEMS_20648
    return inv_load_firmware_20648(dmp_image_sram);
    #elif defined MEMS_20609
    return inv_load_firmware_20609(dmp_image_sram);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_bts_accel_matrix(signed char *orientation)
{
    #if defined MEMS_20630
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_30630
    return dmp_set_bts_accel_matrix_30630(orientation);
    #elif defined MEMS_20645E
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20648
    return INV_ERROR_INVALID_CONFIGURATION;
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

void inv_get_dmp_start_address(unsigned short *dmp_cnfg)
{
    #if defined MEMS_20630
    inv_get_dmp_start_address_20630(dmp_cnfg);
    #elif defined MEMS_30630
    inv_get_dmp_start_address_30630(dmp_cnfg);
    #elif defined MEMS_20645E
    inv_get_dmp_start_address_20645E(dmp_cnfg);
    #elif defined MEMS_20648
    inv_get_dmp_start_address_20648(dmp_cnfg);
    #elif defined MEMS_20609
    inv_get_dmp_start_address_20609(dmp_cnfg);
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_accel_fsr(short accel_fsr)
{
    #if defined MEMS_20630
    return dmp_set_accel_fsr_20630(accel_fsr);
    #elif defined MEMS_30630
    return dmp_set_accel_fsr_30630(accel_fsr);
    #elif defined MEMS_20645E
    return dmp_set_accel_fsr_20645E(accel_fsr);
    #elif defined MEMS_20648
    return dmp_set_accel_fsr_20648(accel_fsr);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t dmp_set_accel_scale2(short accel_fsr)
{
    #if defined MEMS_20630
    return dmp_set_accel_scale2_20630(accel_fsr);
    #elif defined MEMS_30630
    return INV_SUCCESS;
    #elif defined MEMS_20645E
    return INV_SUCCESS;
    #elif defined MEMS_20648
    return dmp_set_accel_scale2_20648(accel_fsr);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

inv_error_t inv_set_dmp_stepcounter_update_offset(unsigned long steps)
{
    sStepCounterToBeSubtracted = steps - sOldSteps;

    return 0;
}

// 5061: NEED TO MAKE SURE THIS IS FOR THE RIGHT CHIP
inv_error_t dmp_reset_pickup(void)
{
    #if defined MEMS_30630
    return dmp_reset_pickup_30630();
    #elif defined MEMS_20630
    return INV_SUCCESS;
    #elif defined MEMS_20645E
    return INV_SUCCESS;             // should there be a function for this chip?
    #elif defined MEMS_20648
    return INV_SUCCESS;
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

//Function to reset the BAC states when bac is restarted.
inv_error_t dmp_reset_bac_states(void)
{
    #if defined MEMS_30630
    return INV_SUCCESS;
    #elif defined MEMS_20630
    return INV_SUCCESS;
    #elif defined MEMS_20645E
    return INV_SUCCESS;             // should there be a function for this chip?
    #elif defined MEMS_20648
    return dmp_reset_bac_states_20648();
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

//Function to configure accel data decimation rate for BAC/Pickup
inv_error_t dmp_set_bac_rate(short accel_div)
{
    #if defined MEMS_30630
    return INV_SUCCESS;
    #elif defined MEMS_20630
    return INV_SUCCESS;
    #elif defined MEMS_20645E
    return INV_SUCCESS;
    #elif defined MEMS_20648
    short accel_odr = 0;

    if (accel_div <= 4)			// 1125/(4+1) = 225Hz
        accel_odr = 2;
    else if (accel_div <= 10)	// 1125/(10+1) = 102Hz
        accel_odr = 1;
    else
        accel_odr = 0;		// expect 56Hz as minimum rate

    return dmp_set_bac_rate_20648(accel_odr);
    #elif defined MEMS_20609
    return INV_SUCCESS;
    #else
#error "Unsupported configuration"
    #endif
}

/* Map DMP API to DMP driver */
// #include "inv_drv_hook.h"
// #include "data_converter.h"
#include "inv_mems_load_firmware.h"
// #include "inv_defines.h"
#include "inv_mems_transport.h"
#include "ml_math_func.h"

inv_error_t inv_dmpdriver_write_mems(unsigned short reg, unsigned int length, const unsigned char *data)
{
    return inv_write_mems(reg, length, data);
}

inv_error_t inv_dmpdriver_read_mems(unsigned short reg, unsigned int length, unsigned char *data)
{
    return inv_read_mems(reg, length, data);
}

inv_error_t inv_dmpdriver_mems_firmware_load(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{
    return inv_mems_firmware_load(data_start, size_start, load_addr);
}

unsigned char *inv_dmpdriver_int16_to_big8(short x, unsigned char *big8)
{
    return inv_int16_to_big8(x, big8);
}

unsigned char *inv_dmpdriver_int32_to_big8(long x, unsigned char *big8)
{
    return inv_int32_to_big8(x, big8);
}

long inv_dmpdriver_big8_to_int32(const unsigned char *big8)
{
    return inv_big8_to_int32(big8);
}


