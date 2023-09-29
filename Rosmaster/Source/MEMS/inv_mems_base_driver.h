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
/** @defgroup mems_base_driver base_driver
	@ingroup  Mems_driver
	@{
*/
#ifdef MEMS_20648
#ifndef INV_MEMS_BASE_DRIVER_H__HWDFWQ__
#define INV_MEMS_BASE_DRIVER_H__HWDFWQ__

#include "mltypes.h"
#include "inv_mems_defines.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief struct for the base_driver : this contains the Mems information */
struct base_driver_t
{
    unsigned char wake_state;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char user_ctrl;
    unsigned char gyro_div;
    unsigned short secondary_div;
    short accel_div;
    unsigned char gyro_averaging;
    unsigned char accel_averaging;
    uint8_t gyro_fullscale;
    uint8_t accel_fullscale;
    uint8_t lp_en_support: 1;
    uint8_t firmware_loaded: 1;
    uint8_t serial_interface;
    uint8_t timebase_correction_pll;
};

/** @brief struct to be used by user */
extern struct base_driver_t base_state;

// Standard Functions
/** @brief Initializes the platform
* @param[in] type 				Define the interface for communicate : SERIAL_INTERFACE_I2C or SERIAL_INTERFACE_SPI
* @param[out] dmp_image_sram 4 	The image to be load
* @return 						0 on success, negative value on error.
*/
inv_error_t inv_initialize_lower_driver(enum MEMS_SERIAL_INTERFACE type, const unsigned char *dmp_image_sram);

#if defined MEMS_SECONDARY_DEVICE
/** @brief Initializes the compass and the id address
* @param[in] id 	address of compass component
* @return 			0 on success, negative value on error.
*/
inv_error_t inv_set_slave_compass_id(int id);

/** @brief Initializes the pressure
* @return 			0 on success, negative value on error.
*/
inv_error_t inv_set_slave_pressure_id(void);
#endif

/** @brief Selects the interface of communication with the board
* @param[in] type 	Define the interface for communicate : SERIAL_INTERFACE_I2C or SERIAL_INTERFACE_SPI
* @return 			0 on success, negative value on error.
*/
inv_error_t inv_set_serial_comm(enum MEMS_SERIAL_INTERFACE type);

/** @brief Wakes up mems platform
* @return 	0 on success, negative value on error.
*/
inv_error_t inv_wakeup_mems(void);

/** @brief Sleeps up mems platform
* @return 	0 on success, negative value on error.
*/
inv_error_t inv_sleep_mems(void);

/** @brief Sets the power state of the Ivory chip loop
* @param[in] func  		CHIP_AWAKE, CHIP_LP_ENABLE
* @param[in] on_off 	The functions are enabled if previously disabled and
*                		disabled if previously enabled based on the value of On/Off.
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_chip_power_state(unsigned char func, unsigned char on_off);

/** @brief Current wake status of the Mems chip
* @return the wake status
*/
uint8_t inv_get_chip_power_state(void);

/** @brief Sets up dmp start address and firmware
* @return  0 on success, negative value on error.
*/
inv_error_t inv_set_dmp_address(void);

/** @brief Sets up the secondary i2c bus
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_secondary(void);

/** @brief Enables accel and/or gyro and/or pressure if integrated with gyro and accel.
* @param[in] bit_mask 	A mask where 2 means turn on accel, 1 means turn on gyro, 4 is for pressure.
*            			By default, this only turns on a sensor if all sensors are off otherwise the DMP controls
*            			this register including turning off a sensor. To override this behavior add in a mask of 128.
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_enable_mems_hw_sensors(int bit_mask);

/** @brief Sets the dmp for a particular gyro configuration.
* @param[in] gyro_div 	Value written to GYRO_SMPLRT_DIV register, where
*            			0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
*           			10=102.2727Hz sample rate, ... etc.
* @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_gyro_sf(unsigned char div, int gyro_level);

/** @brief Sets the gyro sample rate
* @param[in] div 		Value written to GYRO_SMPLRT_DIV register
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_gyro_divider(unsigned char div);

/** @brief Returns the gyro sample rate
* @return Value written to GYRO_SMPLRT_DIV register.
*/
unsigned char inv_get_gyro_divider(void);

/** @brief Returns the real odr in Milliseconds, Micro Seconds or Ticks.
* @param[in] odrInDivider 	Odr In divider
* @param[in] odr_units 		Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks
* @return Odr in fucntion of enum.
*/
uint32_t inv_get_odr_in_units( unsigned short odrInDivider, unsigned char odr_units );

/** @brief Sets the accel sample rate
* @param[in] div 		Value written to ACCEL_SMPLRT_DIV register
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_accel_divider(short div);

/** @brief Returns the accel sample rate
* @return the divider for the accel
*/
short inv_get_accel_divider(void);

/** @brief Sets the I2C secondary device sample rate
* @param[in] div 		Value written to REG_I2C_MST_ODR_CONFIG register
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_secondary_divider(unsigned char div);

/** @brief Returns the I2C secondary device sample rate
* @return the divider for the I2C secondary device interface
*/
unsigned short inv_get_secondary_divider(void);

/** @brief Sets fullscale range of gyro in hardware.
* @param[in]  level  See mpu_gyro_fs.
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_gyro_fullscale(int level);

/** @brief Returns fullscale range of gyrometer in hardware
* @return the fullscale range
*/
uint8_t inv_get_gyro_fullscale(void);

/** @brief Sets fullscale range of gyro in hardware.
* @param[in]  level  See mpu_gyro_fs.
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_mems_gyro_fullscale(int level);

/** @brief Sets fullscale range of accel in hardware.
* @param[in]  	level  See mpu_accel_fs.
* @return 		0 on success, negative value on error.
*/
inv_error_t inv_set_accel_fullscale(int level);

/** @brief Returns fullscale range of accelerometer in hardware
* @return the fullscale range
*/
uint8_t inv_get_accel_fullscale(void);

/** @brief Sets fullscale range of accel in hardware.
* @param[in]  	level  See mpu_accel_fs.
* @return 		0 on success, negative value on error.
*/
inv_error_t inv_set_mems_accel_fullscale(int level);

/** @brief Asserts int1 interrupt when DMP execute INT1 cmd
* @param[in] enable		0=off, 1=on
* @return 				0 on success, negative value on error.
*/
inv_error_t inv_set_int1_assertion(int enable);

/** @brief Reads accelerometer data stored in hardware register
* @param[in] accel_hw_reg_data 	variable to be recuperated the accelerometer data
* @return 						0 on success, negative value on error.
*/
inv_error_t inv_accel_read_hw_reg_data(short accel_hw_reg_data[3]);

/** @brief Prevent LP_EN from being set to 1 again, this speeds up transaction
*/
void inv_mems_prevent_lpen_control(void);

/** @brief Allow LP_EN to be set to 1 again and sets it to 1 again if supported by chip
*/
void inv_mems_allow_lpen_control(void);

#if defined MEMS_SECONDARY_DEVICE
/** @brief Determine if compass could be successfully found and inited on board
* @return	1 on success, 0 if not available.
*/
int inv_mems_get_compass_availability(void);
/** @brief Determine if pressure could be successfully found and inited on board
* @return	1 on success, 0 if not available.
*/
int inv_mems_get_pressure_availability(void);
#endif

/** @brief Reset ODR counters in DMP
* @return	1 on success, 0 if not available.
*/
inv_error_t inv_reset_dmp_odr_counters(void);

#ifdef __cplusplus
}
#endif
#endif // INV_MEMS_BASE_DRIVER_H__HWDFWQ__
#endif
/** @} */
