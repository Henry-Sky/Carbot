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
/** @defgroup mems_base_control base_control
	@ingroup  Mems_driver
	@{
*/
#ifndef MEMS_20609

#ifndef INV_MEMS_BASE_CONTROL_H__HWDFWQ__
#define INV_MEMS_BASE_CONTROL_H__HWDFWQ__

#include "mltypes.h"
#include "inv_mems_base_driver.h"
#include "inv_mems_defines.h"
#include "inv_mems_hw_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Define the Hardware engine*/
enum INV_HW_ENGINE
{
    HW_ENGINE_GYRO = 0,
    HW_ENGINE_ACCEL,
    HW_ENGINE_CPASS,
    HW_ENGINE_PRESSURE,
    HW_ENGINE_LIGHT,
    HW_ENGINE_TEMPERATURE,
    HW_ENGINE_HUMIDITY,
    HW_ENGINE_NUM_MAX,
};

// Determines which base sensor needs to be on based upon inv_androidSensorsOn_mask[0]
//#define INV_NEEDS_ACCEL_MASK	((1L<<1)|(1L<<3)|(1L<<9)|(1L<<10)|(1L<<11)|(1L<<15)|(1L<<17)|(1L<<18)|(1L<<19)|(1L<<20)|(1<<23)|(1<<25)|(1<<29)|(1<<30)|(1<<31))
//#define INV_NEEDS_GYRO_MASK		((1L<<3)|(1L<<4)|(1L<<9)|(1L<<10)|(1L<<11)|(1L<<15)|(1L<<16)|(1<<25)|(1<<26)|(1<<29)|(1<<30)|(1<<31))
//#define INV_NEEDS_COMPASS_MASK	((1L<<2)|(1L<<3)|(1L<<11)|(1L<<14)|(1L<<20)|(1<<24)|(1<<25)|(1<<31))
//#define INV_NEEDS_PRESSURE		((1L<<6)|(1<<28))

#define INV_NEEDS_ACCEL_MASK		(((uint32_t)1L<<1)|((uint32_t)1L<<3)|((uint32_t)1L<<9)|((uint32_t)1L<<10)|((uint32_t)1L<<11)|((uint32_t)1L<<15)|((uint32_t)1L<<17)|((uint32_t)1L<<18)|((uint32_t)1L<<19)|((uint32_t)1L<<20)|((uint32_t)1L<<23)|((uint32_t)1L<<25)|((uint32_t)1L<<29)|((uint32_t)1L<<30)|((uint32_t)1L<<31))
#define INV_NEEDS_GYRO_MASK		 	(((uint32_t)1L<<3)|((uint32_t)1L<<4)|((uint32_t)1L<<9)|((uint32_t)1L<<10)|((uint32_t)1L<<11)|((uint32_t)1L<<15)|((uint32_t)1L<<16)|((uint32_t)1L<<25)|((uint32_t)1L<<26)|((uint32_t)1L<<29)|((uint32_t)1L<<30)|((uint32_t)1L<<31))
#define INV_NEEDS_COMPASS_MASK  (((uint32_t)1L<<2)|((uint32_t)1L<<3)|((uint32_t)1L<<11)|((uint32_t)1L<<14)|((uint32_t)1L<<20)|((uint32_t)1L<<24)|((uint32_t)1L<<25)|((uint32_t)1L<<31))
#define INV_NEEDS_PRESSURE		 	(((uint32_t)1L<<6)|((uint32_t)1<<28))


// Determines which base sensor needs to be on based upon inv_androidSensorsOn_mask[1]
#define INV_NEEDS_ACCEL_MASK1	((1<<3)|(1<<5)|(1<<6)|(1<<7)|(1<<9))
#define INV_NEEDS_GYRO_MASK1	((1<<3)|(1<<4))
#define INV_NEEDS_COMPASS_MASK1	((1<<2)|(1<<7))

#define GYRO_AVAILABLE		0x1
#define ACCEL_AVAILABLE		0x2
#define SECONDARY_COMPASS_AVAILABLE	0x8
#define SECONDARY_PRESSURE_AVAILABLE	0x04

// data output control reg 1
#define ACCEL_SET		0x8000
#define GYRO_SET		0x4000
#define CPASS_SET		0x2000
#define ALS_SET			0x1000
#define QUAT6_SET		0x0800
#define QUAT9_SET		0x0400
#define PQUAT6_SET		0x0200
#if (MEMS_CHIP == HW_ICM20648)
#define GEOMAG_SET		0x0100
#else
#define FOOTER_SET		0x0100
#endif
#define PRESSURE_SET	0x0080
#define GYRO_CALIBR_SET	0x0040
#define CPASS_CALIBR_SET 0x0020
#define PED_STEPDET_SET	0x0010
#define HEADER2_SET		0x0008
#define PED_STEPIND_SET 0x0007

// data output control reg 2
#define ACCEL_ACCURACY_SET	0x4000
#define GYRO_ACCURACY_SET	0x2000
#if (MEMS_CHIP == HW_ICM20630) || (MEMS_CHIP == HW_ICM20648)
#define CPASS_ACCURACY_SET	0x1000
#endif
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E)
#define FSYNC_SET			0x0800
#define FLIP_PICKUP_SET     0x0400
#define ACT_RECOG_SET       0x0080 //not in HW_ICM30630
#endif
#if (MEMS_CHIP == HW_ICM30630) || (MEMS_CHIP == HW_ICM20648)
#define COMPASS_CAL_INPUT_SET	0x1000
#define FLIP_PICKUP_SET     0x0400
#define ACT_RECOG_SET       0x0080
#endif
#if (MEMS_CHIP != HW_ICM20648)
#define GEOMAG_EN			0x0200
#endif
#define BATCH_MODE_EN		0x0100

// motion event control reg
#if (MEMS_CHIP != HW_ICM30630)
#define INV_PEDOMETER_EN		0x4000
#if defined MEMS_WEARABLE_DEVICE
#define INV_BAC_WEARABLE                0x8000
#endif
#define INV_PEDOMETER_INT_EN	0x2000
#define INV_SMD_EN				0x0800
#endif
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E )
#define INV_BTS_EN				0x1000
#define FLIP_PICKUP_EN			0x0400
#elif (MEMS_CHIP == HW_ICM20648)
#define INV_BTS_EN				0x0020
#define FLIP_PICKUP_EN			0x0010
#define GEOMAG_EN   			0x0008
#endif
#define INV_ACCEL_CAL_EN		0x0200
#define INV_GYRO_CAL_EN			0x0100
#define INV_COMPASS_CAL_EN		0x0080
#define INV_NINE_AXIS_EN        0x0040

#if (MEMS_CHIP == HW_ICM20648)
#define INV_BRING_AND_LOOK_T0_SEE_EN  0x0004  // Aded by ONn for 20648
#else
#define INV_BTS_EN				0x1000
#endif

// data packet size reg 1
#define HEADER_SZ		2
#if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
#define ACCEL_DATA_SZ	6
#elif (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E )
#define ACCEL_DATA_SZ	12
#endif
#define GYRO_DATA_SZ	6
#if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
#define CPASS_DATA_SZ	6
#endif
#define ALS_DATA_SZ		8
#define QUAT6_DATA_SZ	12
#if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
#define QUAT9_DATA_SZ	14
#elif (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E)
#define QUAT9_DATA_SZ	12
#endif
#define PQUAT6_DATA_SZ	6
#if (MEMS_CHIP == HW_ICM20648)
#define GEOMAG_DATA_SZ	14
#endif
#define PRESSURE_DATA_SZ		6
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
#define GYRO_BIAS_DATA_SZ	6
#else
#define GYRO_CALIBR_DATA_SZ		12
#endif
#define CPASS_CALIBR_DATA_SZ	12
#define PED_STEPDET_TIMESTAMP_SZ	4
#if (MEMS_CHIP != HW_ICM20648)
#define FOOTER_SZ			2
#endif

// data packet size reg 2
#define HEADER2_SZ			2
#define ACCEL_ACCURACY_SZ	2
#define GYRO_ACCURACY_SZ	2
#define CPASS_ACCURACY_SZ	2
#define FSYNC_SZ			2
#define FLIP_PICKUP_SZ      2
#define ACT_RECOG_SZ        6
#if (MEMS_CHIP == HW_ICM20648)
#define ODR_CNT_GYRO_SZ	2
#endif

/** @brief Sets the odr for a sensor
* @param[in] androidSensor  Sensor Identity
* @param[in] delayInMs  the delay between two values in ms
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_odr(unsigned char androidSensor, unsigned short delayInMs);

/** @brief Enables / disables a sensor
* @param[in] androidSensor  Sensor Identity
* @param[in] enable			0=off, 1=on
* @return 					0 in case of success, -1 for any error
*/
inv_error_t inv_enable_sensor(unsigned char androidSensor, unsigned char enable);

/** @brief Enables / disables batch for the sensors
* @param[in] enable			0=off, 1=on
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_enable_batch(unsigned char enable);

#if (MEMS_CHIP == HW_ICM20648)
/** @brief Set batch mode status
* @param[in] enable			0=off, 1=on
*/
void inv_set_batch_mode_status(unsigned char enable);

/** @brief Get batch mode status
* @return 					0=batch mode disable, 1=batch mode enable
*/
unsigned char inv_get_batch_mode_status(void);
#endif

/** @brief Sets the timeout for the batch in second
* @param[in] batch_time_in_seconds  time in second
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_batch_timeout(unsigned short batch_time_in_seconds);

/** @brief Sets the timeout for the batch in millisecond
* @param[in] batch_time_in_ms  time in millisecond
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_batch_timeout_ms(unsigned short batch_time_in_ms);

/** @brief Enables / disables BAC
* @param[in] enable	0=off, 1=on
*/
void inv_enable_activity_classifier(unsigned char enable);

/** @brief Enables / disables tilt
* @param[in] enable	0=off, 1=on
*/
void inv_enable_tilt(unsigned char enable);

/** @brief Enables / disables bring to see
* @param[in] enable	0=off, 1=on
*/
void inv_enable_b2s(unsigned char enable);

/** @brief Returns the mask for the different sensors enabled
* @return the mask
*/
unsigned long *inv_get_androidSensorsOn_mask(void);

/** @brief Check if a sensor is enabled
* @return 1 if sensor is enabled
*/
unsigned long inv_androidSensor_enabled(unsigned char androidSensor);

/** @brief Returns a flag to know if the BAC is running
* @return 1 if started, 0 if stopped
*/
unsigned short inv_get_activitiy_classifier_on_flag(void);

/** @brief Enumeration for the Type of ODR : Millisecondes / Microsecondes / Ticks */
enum INV_ODR_TYPE
{
    ODR_IN_Ms,
    ODR_IN_Us,
    ODR_IN_Ticks
};

/** @brief Gets the odr for a sensor
* @param[in] SensorId  	Sensor Identity
* @param[out] odr  	pointer to the ODR for this sensor
* @param[in] odr_units  unit expected for odr, one of INV_ODR_TYPE
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_get_odr(unsigned char SensorId, uint32_t *odr, enum INV_ODR_TYPE odr_units);

/** @brief Sets accel quaternion gain according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_accel_quaternion_gain(unsigned short hw_smplrt_divider);

/** @brief Sets accel cal parameters according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_accel_cal_params(unsigned short hw_smplrt_divider);


#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)
/** @brief Enables / disables pickup gesture
* @param(in) enable: 1 for enable, 0 for disable
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_enable_pickup(unsigned char enable);
#endif


#ifdef __cplusplus
}
#endif
#endif // INV_MEMS_BASE_CONTROL_H__HWDFWQ__
#endif
/** @} */
