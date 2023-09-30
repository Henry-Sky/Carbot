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

#if !defined MEMS_20609

#include "dmp3Driver.h"
#include "dmp3Default_20648.h"
#include "invn_types.h"

#define CFG_FIFO_SIZE                   (4184)

// data output control
#define DATA_OUT_CTL1			(4 * 16)
#define DATA_OUT_CTL2			(4 * 16 + 2)
#define DATA_INTR_CTL			(4 * 16 + 12)
#define FIFO_WATERMARK			(31 * 16 + 14)

// motion event control
#define MOTION_EVENT_CTL		(4 * 16 + 14)

// indicates to DMP which sensors are available
/*	1: gyro samples available
	2: accel samples available
	8: secondary samples available	*/
#define DATA_RDY_STATUS			(8 * 16 + 10)

// batch mode
#define BM_BATCH_CNTR			(27 * 16)
#define BM_BATCH_THLD			(19 * 16 + 12)
#define BM_BATCH_MASK			(21 * 16 + 14)

// sensor output data rate
#define ODR_ACCEL				(11 * 16 + 14)
#define ODR_GYRO				(11 * 16 + 10)
#define ODR_CPASS				(11 * 16 +  6)
#define ODR_ALS					(11 * 16 +  2)
#define ODR_QUAT6				(10 * 16 + 12)
#define ODR_QUAT9				(10 * 16 +  8)
#define ODR_PQUAT6				(10 * 16 +  4)
#define ODR_GEOMAG				(10 * 16 +  0)
#define ODR_PRESSURE			(11 * 16 + 12)
#define ODR_GYRO_CALIBR			(11 * 16 +  8)
#define ODR_CPASS_CALIBR		(11 * 16 +  4)

// sensor output data rate counter
#define ODR_CNTR_ACCEL			(9 * 16 + 14)
#define ODR_CNTR_GYRO			(9 * 16 + 10)
#define ODR_CNTR_CPASS			(9 * 16 +  6)
#define ODR_CNTR_ALS			(9 * 16 +  2)
#define ODR_CNTR_QUAT6			(8 * 16 + 12)
#define ODR_CNTR_QUAT9			(8 * 16 +  8)
#define ODR_CNTR_PQUAT6			(8 * 16 +  4)
#define ODR_CNTR_GEOMAG			(8 * 16 +  0)
#define ODR_CNTR_PRESSURE		(9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR	(9 * 16 +  8)
#define ODR_CNTR_CPASS_CALIBR	(9 * 16 +  4)

// mounting matrix
#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#define GYRO_SF					(19 * 16)
#define ACCEL_FB_GAIN			(34 * 16)
#define ACCEL_ONLY_GAIN			(16 * 16 + 12)

// bias calibration
#define GYRO_BIAS_X				(139 * 16 +  4)
#define GYRO_BIAS_Y				(139 * 16 +  8)
#define GYRO_BIAS_Z				(139 * 16 + 12)
#define GYRO_ACCURACY			(138 * 16 +  2)
#define GYRO_BIAS_SET			(138 * 16 +  6)
#define GYRO_LAST_TEMPR			(134 * 16)
#define GYRO_SLOPE_X			( 78 * 16 +  4)
#define GYRO_SLOPE_Y			( 78 * 16 +  8)
#define GYRO_SLOPE_Z			( 78 * 16 + 12)

#define ACCEL_BIAS_X            (110 * 16 +  4)
#define ACCEL_BIAS_Y            (110 * 16 +  8)
#define ACCEL_BIAS_Z            (110 * 16 + 12)
#define ACCEL_ACCURACY			(97 * 16)
#define ACCEL_CAL_RESET			(77 * 16)
#define ACCEL_VARIANCE_THRESH	(93 * 16)
#define ACCEL_CAL_RATE			(94 * 16 + 4)
#define ACCEL_PRE_SENSOR_DATA	(97 * 16 + 4)
#define ACCEL_COVARIANCE		(101 * 16 + 8)
#define ACCEL_ALPHA_VAR			(91 * 16)
#define ACCEL_A_VAR				(92 * 16)
#define ACCEL_CAL_INIT			(94 * 16 + 2)
#define ACCEL_CAL_SCALE_COVQ_IN_RANGE	(194 * 16)
#define ACCEL_CAL_SCALE_COVQ_OUT_RANGE	(195 * 16)
#define ACCEL_CAL_TEMPERATURE_SENSITIVITY	(194 * 16 + 4)
#define ACCEL_CAL_TEMPERATURE_OFFSET_TRIM	(194 * 16 + 12)

#define CPASS_BIAS_X            (126 * 16 +  4)
#define CPASS_BIAS_Y            (126 * 16 +  8)
#define CPASS_BIAS_Z            (126 * 16 + 12)
#define CPASS_ACCURACY			(37 * 16)
#define CPASS_BIAS_SET			(34 * 16 + 14)
#define MAR_MODE				(37 * 16 + 2)
#define CPASS_COVARIANCE		(115 * 16)
#define CPASS_COVARIANCE_CUR	(118 * 16 +  8)
#define CPASS_REF_MAG_3D		(122 * 16)
#define CPASS_CAL_INIT			(114 * 16)
#define CPASS_EST_FIRST_BIAS	(113 * 16)
#define MAG_DISTURB_STATE		(113 * 16 + 2)
#define CPASS_VAR_COUNT			(112 * 16 + 6)
#define CPASS_COUNT_7			( 87 * 16 + 2)
#define CPASS_MAX_INNO			(124 * 16)
#define CPASS_BIAS_OFFSET		(113 * 16 + 4)
#define CPASS_CUR_BIAS_OFFSET	(114 * 16 + 4)
#define CPASS_PRE_SENSOR_DATA	( 87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define CPASS_TIME_BUFFER		(112 * 16 + 14)
#define CPASS_RADIUS_3D_THRESH_ANOMALY	(112 * 16 + 8)

#define CPASS_STATUS_CHK		(25 * 16 + 12)

// 9-axis
#define MAGN_THR_9X				(80 * 16)
#define MAGN_LPF_THR_9X			(80 * 16 +  8)
#define QFB_THR_9X				(80 * 16 + 12)

// DMP running counter
#define DMPRATE_CNTR			(18 * 16 + 4)

// pedometer
#define PEDSTD_BP_B				(49 * 16 + 12)
#define PEDSTD_BP_A4			(52 * 16)
#define PEDSTD_BP_A3			(52 * 16 +  4)
#define PEDSTD_BP_A2			(52 * 16 +  8)
#define PEDSTD_BP_A1			(52 * 16 + 12)
#define PEDSTD_SB				(50 * 16 +  8)
#define PEDSTD_SB_TIME			(50 * 16 + 12)
#define PEDSTD_PEAKTHRSH		(57 * 16 +  8)
#define PEDSTD_TIML				(50 * 16 + 10)
#define PEDSTD_TIMH				(50 * 16 + 14)
#define PEDSTD_PEAK				(57 * 16 +  4)
#define PEDSTD_STEPCTR			(54 * 16)
#define PEDSTD_STEPCTR2			(58 * 16 +  8)
#define PEDSTD_TIMECTR			(60 * 16 +  4)
#define PEDSTD_DECI				(58 * 16)
#define PEDSTD_SB2				(60 * 16 + 14)
#define STPDET_TIMESTAMP		(18 * 16 +  8)
#define PEDSTEP_IND				(19 * 16 +  4)

// SMD
#define SMD_VAR_TH              (141 * 16 + 12)
#define SMD_VAR_TH_DRIVE        (143 * 16 + 12)
#define SMD_DRIVE_TIMER_TH      (143 * 16 +  8)
#define SMD_TILT_ANGLE_TH       (179 * 16 + 12)
#define BAC_SMD_ST_TH           (179 * 16 +  8)
#define BAC_ST_ALPHA4           (180 * 16 + 12)
#define BAC_ST_ALPHA4A          (176 * 16 + 12)

// Wake on Motion
#define WOM_ENABLE              (64 * 16 + 14)
#define WOM_STATUS              (64 * 16 + 6)
#define WOM_THRESHOLD           (64 * 16)
#define WOM_CNTR_TH             (64 * 16 + 12)

// Activity Recognition
#define BAC_RATE                (48  * 16 + 10)
#define BAC_STATE               (179 * 16 +  0)
#define BAC_STATE_PREV          (179 * 16 +  4)
#define BAC_ACT_ON              (182 * 16 +  0)
#define BAC_ACT_OFF             (183 * 16 +  0)
#define BAC_STILL_S_F           (177 * 16 +  0)
#define BAC_RUN_S_F             (177 * 16 +  4)
#define BAC_DRIVE_S_F           (178 * 16 +  0)
#define BAC_WALK_S_F            (178 * 16 +  4)
#define BAC_SMD_S_F             (178 * 16 +  8)
#define BAC_BIKE_S_F            (178 * 16 + 12)
#define BAC_E1_SHORT            (146 * 16 +  0)
#define BAC_E2_SHORT            (146 * 16 +  4)
#define BAC_E3_SHORT            (146 * 16 +  8)
#define BAC_VAR_RUN             (148 * 16 + 12)
#define BAC_TILT_INIT           (181 * 16 +  0)
#define BAC_MAG_ON              (225 * 16 +  0)
#define BAC_PS_ON               (74  * 16 +  0)
#define BAC_BIKE_PREFERENCE     (173 * 16 +  8)
#define BAC_MAG_I2C_ADDR        (229 * 16 +  8)
#define BAC_PS_I2C_ADDR         (75  * 16 +  4)
#define BAC_DRIVE_CONFIDENCE    (144 * 16 +  0)
#define BAC_WALK_CONFIDENCE     (144 * 16 +  4)
#define BAC_SMD_CONFIDENCE      (144 * 16 +  8)
#define BAC_BIKE_CONFIDENCE     (144 * 16 + 12)
#define BAC_STILL_CONFIDENCE    (145 * 16 +  0)
#define BAC_RUN_CONFIDENCE      (145 * 16 +  4)
#define BAC_MODE_CNTR           (150 * 16)
#define BAC_STATE_T_PREV        (185 * 16 +  4)
#define BAC_ACT_T_ON            (184 * 16 +  0)
#define BAC_ACT_T_OFF           (184 * 16 +  4)
#define BAC_STATE_WRDBS_PREV    (185 * 16 +  8)
#define BAC_ACT_WRDBS_ON        (184 * 16 +  8)
#define BAC_ACT_WRDBS_OFF       (184 * 16 + 12)
#define BAC_ACT_ON_OFF          (190 * 16 +  2)
#define PREV_BAC_ACT_ON_OFF     (188 * 16 +  2)
#define BAC_CNTR                (48  * 16 +  2)


// Flip/Pick-up
#define FP_VAR_ALPHA            (245 * 16 +  8)
#define FP_STILL_TH             (246 * 16 +  4)
#define FP_MID_STILL_TH         (244 * 16 +  8)
#define FP_NOT_STILL_TH         (246 * 16 +  8)
#define FP_VIB_REJ_TH           (241 * 16 +  8)
#define FP_MAX_PICKUP_T_TH      (244 * 16 + 12)
#define FP_PICKUP_TIMEOUT_TH    (248 * 16 +  8)
#define FP_STILL_CONST_TH       (246 * 16 + 12)
#define FP_MOTION_CONST_TH      (240 * 16 +  8)
#define FP_VIB_COUNT_TH         (242 * 16 +  8)
#define FP_STEADY_TILT_TH       (247 * 16 +  8)
#define FP_STEADY_TILT_UP_TH    (242 * 16 + 12)
#define FP_Z_FLAT_TH_MINUS      (243 * 16 +  8)
#define FP_Z_FLAT_TH_PLUS       (243 * 16 + 12)
#define FP_DEV_IN_POCKET_TH     (76  * 16 + 12)
#define FP_PICKUP_CNTR          (247 * 16 +  4)
#define FP_RATE                 (240 * 16 + 12)

// Accel FSR
#define ACC_SCALE               (30 * 16 + 0)
#define ACC_SCALE2              (79 * 16 + 4)

// S-Health keys
#define S_HEALTH_WALK_RUN_1		(213 * 16 +  12)
#define S_HEALTH_WALK_RUN_2		(213 * 16 +   8)
#define S_HEALTH_WALK_RUN_3		(213 * 16 +   4)
#define S_HEALTH_WALK_RUN_4		(213 * 16 +   0)
#define S_HEALTH_WALK_RUN_5		(212 * 16 +  12)
#define S_HEALTH_WALK_RUN_6		(212 * 16 +   8)
#define S_HEALTH_WALK_RUN_7		(212 * 16 +   4)
#define S_HEALTH_WALK_RUN_8		(212 * 16 +   0)
#define S_HEALTH_WALK_RUN_9		(211 * 16 +  12)
#define S_HEALTH_WALK_RUN_10	(211 * 16 +   8)
#define S_HEALTH_WALK_RUN_11	(211 * 16 +   4)
#define S_HEALTH_WALK_RUN_12	(211 * 16 +   0)
#define S_HEALTH_WALK_RUN_13	(210 * 16 +  12)
#define S_HEALTH_WALK_RUN_14	(210 * 16 +   8)
#define S_HEALTH_WALK_RUN_15	(210 * 16 +   4)
#define S_HEALTH_WALK_RUN_16	(210 * 16 +   0)
#define S_HEALTH_WALK_RUN_17	(209 * 16 +  12)
#define S_HEALTH_WALK_RUN_18	(209 * 16 +   8)
#define S_HEALTH_WALK_RUN_19	(209 * 16 +   4)
#define S_HEALTH_WALK_RUN_20	(209 * 16 +   0)
#define S_HEALTH_CADENCE1		(213 * 16 +  14)
#define S_HEALTH_CADENCE2		(213 * 16 +  10)
#define S_HEALTH_CADENCE3		(213 * 16 +   6)
#define S_HEALTH_CADENCE4		(213 * 16 +   2)
#define S_HEALTH_CADENCE5		(212 * 16 +  14)
#define S_HEALTH_CADENCE6		(212 * 16 +  10)
#define S_HEALTH_CADENCE7		(212 * 16 +   6)
#define S_HEALTH_CADENCE8		(212 * 16 +   2)
#define S_HEALTH_CADENCE9		(211 * 16 +  14)
#define S_HEALTH_CADENCE10		(211 * 16 +  10)
#define S_HEALTH_CADENCE11		(211 * 16 +   6)
#define S_HEALTH_CADENCE12		(211 * 16 +   2)
#define S_HEALTH_CADENCE13		(210 * 16 +  14)
#define S_HEALTH_CADENCE14		(210 * 16 +  10)
#define S_HEALTH_CADENCE15		(210 * 16 +   6)
#define S_HEALTH_CADENCE16		(210 * 16 +   2)
#define S_HEALTH_CADENCE17		(209 * 16 +  14)
#define S_HEALTH_CADENCE18		(209 * 16 +  10)
#define S_HEALTH_CADENCE19		(209 * 16 +   6)
#define S_HEALTH_CADENCE20		(209 * 16 +   2)
#define S_HEALTH_INT_PERIOD		(214 * 16 +   6)
#define S_HEALTH_INT_PERIOD2	(214 * 16 +  10)
#define S_HEALTH_BACKUP1		(214 * 16 +   0)
#define S_HEALTH_BACKUP2		(214 * 16 +   2)
#define S_HEALTH_RATE           (208 * 16 +  14)

// EIS authentication
#define EIS_AUTH_INPUT			(160 * 16 +   4)
#define EIS_AUTH_OUTPUT			(160 * 16 +   0)

#define DMP_START_ADDRESS   ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE   256
#define DMP_LOAD_START      0x90

#define DMP_CODE_SIZE 14250

#if !defined USE_DMP_LOAD_SRAM
static const unsigned char dmp_memory[DMP_CODE_SIZE] =
{
#include "dmp3Default_20648.txt"
};
#endif


/** Loads the dmp firmware for the 20648 part.
* @param[in] dmp_image_sram Load DMP3 image from SRAM.
*/
inv_error_t inv_load_firmware_20648(const unsigned char *dmp_image_sram)
{
    #if defined USE_DMP_LOAD_SRAM
    return inv_dmpdriver_mems_firmware_load(dmp_image_sram, DMP_CODE_SIZE, DMP_LOAD_START);
    #else
    (void) *dmp_image_sram;
    return inv_dmpdriver_mems_firmware_load(dmp_memory, DMP_CODE_SIZE, DMP_LOAD_START);
    #endif
}

/** Loads the dmp firmware for the 20648 part.
* @param[out] dmp_cnfg The config item
*/
void inv_get_dmp_start_address_20648(unsigned short *dmp_cnfg)
{
    *dmp_cnfg = DMP_START_ADDRESS;
}

/**
* Sets data output control register 1.
* @param[in] output_mask	Turns sensors on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*							DMP will also turn hw sensors on/off based on bits set in output_mask.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - enable/disable data output in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
inv_error_t dmp_set_data_output_control1_20648(int output_mask)
{

    inv_error_t result;
    unsigned char data_output_control_reg1[2];

    data_output_control_reg1[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg1[1] = (unsigned char)(output_mask & 0xff);

    result = inv_dmpdriver_write_mems(DATA_OUT_CTL1, 2, data_output_control_reg1);

    return result;
}

/**
* Sets data output control register 2.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	ACCEL_ACCURACY_SET	0x4000 - accel accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	GYRO_ACCURACY_SET	0x2000 - gyro accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	CPASS_ACCURACY_SET	0x1000 - compass accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	BATCH_MODE_EN		0x0100 - enable batching
*/
inv_error_t dmp_set_data_output_control2_20648(int output_mask)
{
    inv_error_t result;
    static unsigned char data_output_control_reg2[2] = {0};

    data_output_control_reg2[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg2[1] = (unsigned char)(output_mask & 0xff);

    result = inv_dmpdriver_write_mems(DATA_OUT_CTL2, 2, data_output_control_reg2);

    return result;
}

/**
* Clears all output control registers:
*	data output control register 1, data output control register 2, data interrupt control register, motion event control regsiter, data ready status register
*/
inv_error_t dmp_reset_control_registers_20648()
{
    inv_error_t result;
    unsigned char data[4] = {0};

    //reset data output control registers
    result = inv_dmpdriver_write_mems(DATA_OUT_CTL1, 2, &data[0]);
    result += inv_dmpdriver_write_mems(DATA_OUT_CTL2, 2, &data[0]);

    //reset data interrupt control register
    result += inv_dmpdriver_write_mems(DATA_INTR_CTL, 2, &data[0]);

    //reset motion event control register
    result += inv_dmpdriver_write_mems(MOTION_EVENT_CTL, 2, &data[0]);

    //reset data ready status register
    result += inv_dmpdriver_write_mems(DATA_RDY_STATUS, 2, &data[0]);
    //result += inv_dmpdriver_write_mems(DATA_RDY_STATUS, 2, inv_dmpdriver_int16_to_big8(3, data)); //fixme

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets data interrupt control register.
* @param[in] interrupt_ctl	Determines which sensors can generate interrupt according to following bit definition,
*							bit set indicates interrupt, bit clear indicates no interrupt.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - data output defined in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
inv_error_t dmp_set_data_interrupt_control_20648(uint32_t interrupt_ctl)
{
    inv_error_t result;
    unsigned char big8[2] = {0};

    result = inv_dmpdriver_write_mems(DATA_INTR_CTL, 2, inv_dmpdriver_int16_to_big8(interrupt_ctl, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets FIFO watermark. DMP will send FIFO interrupt if FIFO count > FIFO watermark
* @param[in] fifo_wm	FIFO watermark set to 80% of actual FIFO size by default
*/
inv_error_t dmp_set_FIFO_watermark_20648(unsigned short fifo_wm)
{
    inv_error_t result;
    unsigned char big8[2] = {0};

    result = inv_dmpdriver_write_mems(FIFO_WATERMARK, 2, inv_dmpdriver_int16_to_big8(fifo_wm, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets data rdy status register.
* @param[in] data_rdy	Indicates which sensor data is available.
*
*	gyro samples available		0x1
*	accel samples available		0x2
*	secondary samples available	0x8
*/
inv_error_t dmp_set_data_rdy_status_20648(unsigned short data_rdy)
{
    inv_error_t result;
    unsigned char big8[2] = {0};

    result = inv_dmpdriver_write_mems(DATA_RDY_STATUS, 2, inv_dmpdriver_int16_to_big8(data_rdy, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets motion event control register.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	BAC_WEAR_EN             0x8000 - wearable devices BAC is enabled
*       PEDOMETER_EN		0x4000 - pedometer engine
*	PEDOMETER_INT_EN	0x2000 - pedometer step detector interrupt
*	SMD_EN				0x0800 - significant motion detection interrupt
*	ACCEL_CAL_EN		0x0200 - accel calibration
*	GYRO_CAL_EN			0x0100 - gyro calibration
*	COMPASS_CAL_EN		0x0080 - compass calibration
*	NINE_AXIS_EN        0x0040 - 9-axis algorithm execution
*	GEOMAG_EN			0x0008 - Geomag algorithm execution
*	BAC_ACCEL_ONLY_EN   0x0002 - run BAC as accel only
*/
inv_error_t dmp_set_motion_event_control_20648(unsigned short output_mask)
{
    inv_error_t result;
    unsigned char motion_event_control_reg[2];

    motion_event_control_reg[0] = (unsigned char)(output_mask >> 8);
    motion_event_control_reg[1] = (unsigned char)(output_mask & 0xff);

    result = inv_dmpdriver_write_mems(MOTION_EVENT_CTL, 2, motion_event_control_reg);

    return result;
}

/**
* Sets sensor ODR.
* @param[in] sensor		sensor number based on INV_SENSORS
*	enum INV_SENSORS {
*		INV_SENSOR_ACCEL = 0,
*		INV_SENSOR_GYRO,
*	    INV_SENSOR_LPQ,
*		INV_SENSOR_COMPASS,
*		INV_SENSOR_ALS,
*		INV_SENSOR_SIXQ,
*		INV_SENSOR_NINEQ,
*		INV_SENSOR_GEOMAG,
*		INV_SENSOR_PEDQ,
*		INV_SENSOR_PRESSURE,
*		INV_SENSOR_CALIB_GYRO,
*		INV_SENSOR_CALIB_COMPASS,
*		INV_SENSOR_NUM_MAX,
*		INV_SENSOR_INVALID,
*	};
* @param[in] divider	desired ODR = base engine rate/(divider + 1)
*/
inv_error_t dmp_set_sensor_rate_20648(int invSensor, short divider)
{
    inv_error_t result;
    unsigned char big8[2] = {0};
    int odr_addr = 0;

    switch (invSensor)
    {
        case INV_SENSOR_ACCEL:
            odr_addr = ODR_ACCEL;
            break;

        case INV_SENSOR_GYRO:
            odr_addr = ODR_GYRO;
            break;

        case INV_SENSOR_COMPASS:
            odr_addr = ODR_CPASS;
            break;

        case INV_SENSOR_ALS:
            odr_addr = ODR_ALS;
            break;

        case INV_SENSOR_SIXQ:
            odr_addr = ODR_QUAT6;
            break;

        case INV_SENSOR_NINEQ:
            odr_addr = ODR_QUAT9;
            break;

        case INV_SENSOR_GEOMAG:
            odr_addr = ODR_GEOMAG;
            break;

        case INV_SENSOR_PEDQ:
            odr_addr = ODR_PQUAT6;
            break;

        case INV_SENSOR_PRESSURE:
            odr_addr = ODR_PRESSURE;
            break;

        case INV_SENSOR_CALIB_GYRO:
            odr_addr = ODR_GYRO_CALIBR;
            break;

        case INV_SENSOR_CALIB_COMPASS:
            odr_addr = ODR_CPASS_CALIBR;
            break;

        case INV_SENSOR_STEP_COUNTER:
            //odr_addr = PED_RATE + 2; //PED_RATE is a 4-byte address but only writing 2 bytes here
            break;
    }

    result = inv_dmpdriver_write_mems(odr_addr, 2, inv_dmpdriver_int16_to_big8(divider, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Resets batch counter and sets batch mode parameters.
* @param[in] thld	sets batch timeout in DMP ticks, e.g. batch 1 sec, thld= (1 sec * engine base rate in Hz)
* @param[in] mask	ties batch counter to engine specified with same bit definiton as HW register DATA_RDY_STATUS,
*					i.e. batch counter increments only if the engine specified is available in multi-rate setting
*	BIT 0 set: 1 - tie to gyro
*	BIT 1 set: 2 - tie to accel
*	BIT 2 set: 4 - tie to pressure in Diamond
*	BIT 3 set: 8 - tie to secondary
*/
inv_error_t dmp_set_batchmode_params_20648(unsigned int thld, short mask)
{
    inv_error_t result;
    unsigned char big8[4] = {0};
    unsigned char data[2] = {0};

    result = inv_dmpdriver_write_mems(BM_BATCH_CNTR, 4, big8);
    result += inv_dmpdriver_write_mems(BM_BATCH_THLD, 4, inv_dmpdriver_int32_to_big8(thld, big8));
    result += inv_dmpdriver_write_mems(BM_BATCH_MASK, 2, inv_dmpdriver_int16_to_big8(mask, data));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
*/
inv_error_t dmp_set_bias_20648(int *bias)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_write_mems(ACCEL_BIAS_X, 4, inv_dmpdriver_int32_to_big8(bias[0], big8));
    result += inv_dmpdriver_write_mems(ACCEL_BIAS_Y, 4, inv_dmpdriver_int32_to_big8(bias[1], big8));
    result += inv_dmpdriver_write_mems(ACCEL_BIAS_Z, 4, inv_dmpdriver_int32_to_big8(bias[2], big8));

    if (result)
        return result;

    result = inv_dmpdriver_write_mems(GYRO_BIAS_X, 4, inv_dmpdriver_int32_to_big8(bias[3], big8));
    result += inv_dmpdriver_write_mems(GYRO_BIAS_Y, 4, inv_dmpdriver_int32_to_big8(bias[4], big8));
    result += inv_dmpdriver_write_mems(GYRO_BIAS_Z, 4, inv_dmpdriver_int32_to_big8(bias[5], big8));

    if (result)
        return result;

    result = inv_dmpdriver_write_mems(CPASS_BIAS_X, 4, inv_dmpdriver_int32_to_big8(bias[6], big8));
    result += inv_dmpdriver_write_mems(CPASS_BIAS_Y, 4, inv_dmpdriver_int32_to_big8(bias[7], big8));
    result += inv_dmpdriver_write_mems(CPASS_BIAS_Z, 4, inv_dmpdriver_int32_to_big8(bias[8], big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Gets bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
*/
inv_error_t dmp_get_bias_20648(int *bias)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_read_mems(ACCEL_BIAS_X, 4, big8);
    bias[0] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(ACCEL_BIAS_Y, 4, big8);
    bias[1] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(ACCEL_BIAS_Z, 4, big8);
    bias[2] = inv_dmpdriver_big8_to_int32(big8);

    result = inv_dmpdriver_read_mems(GYRO_BIAS_X, 4, big8);
    bias[3] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(GYRO_BIAS_Y, 4, big8);
    bias[4] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(GYRO_BIAS_Z, 4, big8);
    bias[5] = inv_dmpdriver_big8_to_int32(big8);

    result = inv_dmpdriver_read_mems(CPASS_BIAS_X, 4, big8);
    bias[6] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(CPASS_BIAS_Y, 4, big8);
    bias[7] = inv_dmpdriver_big8_to_int32(big8);
    result += inv_dmpdriver_read_mems(CPASS_BIAS_Z, 4, big8);
    bias[8] = inv_dmpdriver_big8_to_int32(big8);

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets the gyro_sf used by quaternions on the DMP.
* @param[in] gyro_sf	see inv_set_gyro_sf() for value to set based on gyro rate and gyro fullscale range
*/
inv_error_t dmp_set_gyro_sf_20648(long gyro_sf)
{
    inv_error_t result;
    unsigned char big8[4];

    result = inv_dmpdriver_write_mems(GYRO_SF, 4, inv_dmpdriver_int32_to_big8(gyro_sf, big8));

    return result;
}

/**
* Sets the accel gain used by accel quaternion on the DMP.
* @param[in] accel_gain		value changes with accel engine rate
*/
inv_error_t dmp_set_accel_feedback_gain_20648(int accel_gain)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_write_mems(ACCEL_ONLY_GAIN, 4, inv_dmpdriver_int32_to_big8(accel_gain, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets accel cal parameters based on different accel engine rate/accel cal running rate
* @param[in] accel_cal
*	array is set as follows:
*	[0] = ACCEL_CAL_ALPHA_VAR
*	[1] = ACCEL_CAL_A_VAR
*   [2] = ACCEL_CAL_DIV - divider from hardware accel engine rate such that acce cal runs at accel_engine_rate/(divider+1)
*/
inv_error_t dmp_set_accel_cal_params_20648(int *accel_cal)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result  = inv_dmpdriver_write_mems(ACCEL_ALPHA_VAR, 4, inv_dmpdriver_int32_to_big8(accel_cal[ACCEL_CAL_ALPHA_VAR], big8));
    result |= inv_dmpdriver_write_mems(ACCEL_A_VAR, 4, inv_dmpdriver_int32_to_big8(accel_cal[ACCEL_CAL_A_VAR], big8));
    result |= inv_dmpdriver_write_mems(ACCEL_CAL_RATE, 2, inv_dmpdriver_int16_to_big8(accel_cal[ACCEL_CAL_DIV], big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets compass cal parameters based on different compass engine rate/compass cal running rate
* @param[in] compass_cal
*	array is set as follows:
*	[0] = CPASS_CAL_TIME_BUFFER
*	[1] = CPASS_CAL_ALPHA_VAR
*	[2] = CPASS_CAL_A_VAR
*	[3] = CPASS_CAL_RADIUS_3D_THRESH_ANOMALY
*	[4] = CPASS_CAL_NOMOT_VAR_THRESH
*/
inv_error_t dmp_set_compass_cal_params_20648(int *compass_cal)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_write_mems(CPASS_TIME_BUFFER, 2, inv_dmpdriver_int16_to_big8(compass_cal[CPASS_CAL_TIME_BUFFER], big8));
    result += inv_dmpdriver_write_mems(CPASS_RADIUS_3D_THRESH_ANOMALY, 4, inv_dmpdriver_int32_to_big8(compass_cal[CPASS_CAL_RADIUS_3D_THRESH_ANOMALY], big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets compass orientation matrix to DMP.
* @param[in] compass_mtx
*/
inv_error_t dmp_set_compass_matrix_20648(int *compass_mtx)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_write_mems(CPASS_MTX_00, 4, inv_dmpdriver_int32_to_big8(compass_mtx[0], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_01, 4, inv_dmpdriver_int32_to_big8(compass_mtx[1], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_02, 4, inv_dmpdriver_int32_to_big8(compass_mtx[2], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_10, 4, inv_dmpdriver_int32_to_big8(compass_mtx[3], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_11, 4, inv_dmpdriver_int32_to_big8(compass_mtx[4], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_12, 4, inv_dmpdriver_int32_to_big8(compass_mtx[5], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_20, 4, inv_dmpdriver_int32_to_big8(compass_mtx[6], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_21, 4, inv_dmpdriver_int32_to_big8(compass_mtx[7], big8));
    result += inv_dmpdriver_write_mems(CPASS_MTX_22, 4, inv_dmpdriver_int32_to_big8(compass_mtx[8], big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Gets pedometer step count.
* @param[in] steps
* @param[out] steps
*/
inv_error_t dmp_get_pedometer_num_of_steps_20648(unsigned long *steps)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_read_mems(PEDSTD_STEPCTR, 4, big8);

    if (result)
        return result;

    *steps = (big8[0] * (1L << 24)) + (big8[1] * (1L << 16)) + (big8[2] * 256) + big8[3];

    return INV_SUCCESS;
}

/**
* Sets pedometer engine running rate.
* @param[in] ped_rate	divider based on accel engine rate
*/
inv_error_t dmp_set_pedometer_rate_20648(int ped_rate)
{
    // inv_error_t result;
    // unsigned char big8[4]={0};
    // result = inv_dmpdriver_write_mems(PED_RATE, 4, inv_dmpdriver_int32_to_big8(ped_rate, big8));
    // if (result)
    //    return result;

    (void) ped_rate;
    return INV_SUCCESS;
}

/**
* Turns software wake on motion feature on/off.
* @param[in] enable		0=off, 1=on
*/
inv_error_t dmp_set_wom_enable_20648(unsigned char enable)
{
    inv_error_t result;
    unsigned char big8[2] = {0};

    if (enable)
    {
        big8[1] = 0x1;
    }

    result = inv_dmpdriver_write_mems(WOM_ENABLE, 2, big8);

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets motion threshold to determine motion/no motion for wake on motion feature.
* @param[in] threshold
*/
inv_error_t dmp_set_wom_motion_threshold_20648(int threshold)
{
    inv_error_t result;
    unsigned char big8[4] = {0};

    result = inv_dmpdriver_write_mems(WOM_THRESHOLD, 4, inv_dmpdriver_int32_to_big8(threshold, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets minimum time threshold of no motion before DMP goes to sleep.
* @param[in] threshold
*/
inv_error_t dmp_set_wom_time_threshold_20648(unsigned short threshold)
{
    inv_error_t result;
    unsigned char big8[2] = {0};

    result = inv_dmpdriver_write_mems(WOM_CNTR_TH, 2, inv_dmpdriver_int16_to_big8(threshold, big8));

    if (result)
        return result;

    return INV_SUCCESS;
}

/**
* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
* @param[in] fsr for accel parts
             2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

             For 2g parts, 2g = 2^15 -> 1g = 2^14,.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
             In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

             For 8g parts, 8g = 2^15 -> 1g = 2^12.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
             In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
*/
inv_error_t dmp_set_accel_fsr_20648(short accel_fsr)
{
    unsigned char reg[4];
    inv_error_t result;
    long scale;

    switch (accel_fsr)
    {
        case 2:
            scale =  33554432L;  // 2^25
            break;

        case 4:
            scale =  67108864L;  // 2^26
            break;

        case 8:
            scale = 134217728L;  // 2^27
            break;

        case 16:
            scale = 268435456L;  // 2^28
            break;

        case 32:
            scale = 536870912L;  // 2^29
            break;

        default:
            return INV_ERROR;
    }

    result = inv_dmpdriver_write_mems(ACC_SCALE, 4, inv_dmpdriver_int32_to_big8(scale, reg));

    if (result)
    {
        return result;
    }
    else
    {
        return INV_SUCCESS;
    }
}

/**
* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
* It is a reverse scaling of the scale factor written to ACC_SCALE.
* @param[in] fsr for accel parts
			 2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
*/
inv_error_t dmp_set_accel_scale2_20648(short accel_fsr)
{
    unsigned char reg[4];
    inv_error_t result;
    long scale;

    switch (accel_fsr)
    {
        case 2:
            scale = 524288L;  // 2^19
            break;

        case 4:
            scale = 262144L;  // 2^18
            break;

        case 8:
            scale = 131072L;  // 2^17
            break;

        case 16:
            scale = 65536L;  // 2^16
            break;

        case 32:
            scale = 32768L;  // 2^15
            break;

        default:
            return INV_ERROR;
    }

    result = inv_dmpdriver_write_mems(ACC_SCALE2, 4, inv_dmpdriver_int32_to_big8(scale, reg));

    if (result)
    {
        return result;
    }
    else
    {
        return INV_SUCCESS;
    }
}

/**
* Sets the input value for EIS library authentication.
* @param[in] eis_auth_input		random value between (-1,1) in Q30
*/
inv_error_t dmp_set_eis_auth_input_20648(long eis_auth_input)
{
    inv_error_t result;
    unsigned char big8[4];

    result = inv_dmpdriver_write_mems(EIS_AUTH_INPUT, 4, inv_dmpdriver_int32_to_big8(eis_auth_input, big8));

    return result;
}

/**
* Gets the output value from DMP for EIS library authentication.
* @param[out] &eis_auth_output
*/
inv_error_t dmp_get_eis_auth_output_20648(long *eis_auth_output)
{
    inv_error_t result;
    unsigned char big8[4];

    result = inv_dmpdriver_read_mems(EIS_AUTH_OUTPUT, 4, big8);

    *eis_auth_output = inv_dmpdriver_big8_to_int32(big8);

    return result;
}

/**
* BAC only works in 56 Hz. Set divider to make sure accel ODR into BAC is 56Hz.
* @param[in] accel_odr. 0:  56.25Hz
*                       1: 112.5 Hz
*                       2: 225   Hz
*                       3: 450   Hz
*                       4: 900   Hz
*/
inv_error_t dmp_set_bac_rate_20648(short accel_odr)
{
    unsigned char reg[4] = {0, 0, 0, 0};
    inv_error_t result;
    short odr;

    switch (accel_odr)
    {
        case 0:
            odr = 0;
            break;

        case 1:
            odr = 1;
            break;

        case 2:
            odr = 3;
            break;

        case 3:
            odr = 7;
            break;

        case 4:
            odr = 15;
            break;

        default:
            return INV_ERROR;
    }

    result = inv_dmpdriver_write_mems(BAC_RATE, 2, inv_dmpdriver_int16_to_big8(odr, reg));
    result |= inv_dmpdriver_write_mems(FP_RATE, 4, inv_dmpdriver_int32_to_big8((int)odr, reg));

    if (result)
    {
        return result;
    }
    else
    {
        return INV_SUCCESS;
    }
}

inv_error_t dmp_reset_odr_counters_20648()
{
    inv_error_t result;
    unsigned char data[4] = {0x00, 0x00, 0x00, 0x00};
    result = inv_dmpdriver_write_mems(ODR_CNTR_ACCEL, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_GYRO, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_QUAT6, 4, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_PQUAT6, 4, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_CPASS, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_ALS, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_GEOMAG, 4, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_PRESSURE, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_GYRO_CALIBR, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_CPASS_CALIBR, 2, data);
    result |= inv_dmpdriver_write_mems(ODR_CNTR_QUAT9, 2, data);
    return result;
}

/**
* Clear BAC states when restarting BAC/SMD/Pedometer/Tilt.
* This avoids false triggering of BAC-related modules.
*/
inv_error_t dmp_reset_bac_states_20648(void)
{
    inv_error_t result;
    unsigned char big8[4] = {0, 0, 0, 0};
    unsigned char big8_s[2] = {0, 0};
    long reset = 0;
    short reset_s = 0;

    result = inv_dmpdriver_write_mems(BAC_STATE,             4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_STATE_PREV,       4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_ON,           4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_OFF,          4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_STILL_S_F,        4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_RUN_S_F,          4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_DRIVE_S_F,        4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_WALK_S_F,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_SMD_S_F,          4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_BIKE_S_F,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_E1_SHORT,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_E2_SHORT,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_E3_SHORT,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_VAR_RUN,          4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_DRIVE_CONFIDENCE, 4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_WALK_CONFIDENCE,  4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_SMD_CONFIDENCE,   4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_BIKE_CONFIDENCE,  4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_STILL_CONFIDENCE, 4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_RUN_CONFIDENCE,   4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_MODE_CNTR,        4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_STATE_T_PREV,     4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_T_ON,         4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_T_OFF,        4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_STATE_WRDBS_PREV, 4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_WRDBS_ON,     4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_WRDBS_OFF,    4, inv_dmpdriver_int32_to_big8(reset, big8));
    result += inv_dmpdriver_write_mems(BAC_ACT_ON_OFF,       2, inv_dmpdriver_int16_to_big8(reset_s, big8_s));
    result += inv_dmpdriver_write_mems(PREV_BAC_ACT_ON_OFF,  2, inv_dmpdriver_int16_to_big8(reset_s, big8_s));
    result += inv_dmpdriver_write_mems(BAC_CNTR,             2, inv_dmpdriver_int16_to_big8(reset_s, big8_s));

    if (result)
        return result;

    return INV_SUCCESS;
}

#endif


