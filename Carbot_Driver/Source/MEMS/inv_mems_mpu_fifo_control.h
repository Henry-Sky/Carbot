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
/** @defgroup inv_mems_mpu_fifo_control inv_mpu_fifo_control
	@ingroup  Mems_driver
	@{
*/	
#ifndef INV_MEMS_MPU_FIFO_CONTROL_H__GQWDHE__
#define INV_MEMS_MPU_FIFO_CONTROL_H__GQWDHE__

#include "mltypes.h"
#include "inv_mems_hw_config.h"
#include "invn_types.h"

/** @brief Struct for the fifo. this contains the sensor data */
struct inv_fifo_decoded_t
{
    long dmp_3e_6quat[3];
    long dmp_3e_9quat[3];
#if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
    int dmp_rv_accuracyQ29;
#endif
#if (MEMS_CHIP == HW_ICM20648)
    long dmp_3e_geomagquat[3];
    int dmp_geomag_accuracyQ29;
#endif
	short accel_s[3];
    long accel[3];
    short gyro[3];
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
    short gyro_bias[3];
#elif (MEMS_CHIP == HW_ICM20609)
    long gyro_bias[3];
#endif
    long gyro_calibr[3];
    long compass[3];
    long cpass_calibr[3];
    long ped_step_det_ts;
    short cpass_raw_data[3];
    short accel_accuracy;
    short gyro_accuracy;
    short cpass_accuracy;
#if defined MEMS_SECONDARY_DEVICE
    short pressure[3];
#endif
#if (MEMS_CHIP != HW_ICM20648)
	short fsync;
#endif
	long bac_ts;
	unsigned short bac_state;
	short flip_pickup;
    unsigned char cpass_calibr_12chars[12];
    unsigned char cpass_calibr_6chars[6];
    unsigned short header;
    unsigned short header2;
	unsigned short footer;
    int new_data;
};


#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Identify the interrupt
* @param[in] int_read	pointer to the DMP interrupt status
* @return 				0 on success, negative value on error.
*/	
inv_error_t inv_identify_interrupt(short *int_read);

/**
*  @internal
*  @brief  Clears the FIFO status and its content.
*  @note   Halt the DMP writing into the FIFO for the time
*          needed to reset the FIFO.
*  @return MPU_SUCCESS if successful, a non-zero error code otherwise.
*/
inv_error_t dmp_reset_fifo(void);

/** @brief Process the fifo.
* @param[in] left_in_fifo	pointer for the fifo to be processed
* @param[in] user_header	pointer for the user header
* @param[in] user_header2	pointer for the user header 2
* @param[in] time_stamp		pointer for the timestamp
* @return 				0 on success, negative value on error.
*/	
inv_error_t dmp_process_fifo( int *left_in_fifo, unsigned short *user_header, unsigned short *user_header2, mpu_time_t *time_stamp);

/** @brief Gets the accelerometer data 
* @param[out] acl[3]	the accelerometer data 
* @return 					0 on success, negative value on error.
*/		
inv_error_t dmp_get_accel(long acl[3]);

/** @brief Gets the raw gyrometer data 
* @param[out] raw_gyro[3]	the raw gyrometer data 
* @return 						0 on success, negative value on error.
*/	
inv_error_t dmp_get_raw_gyro(short raw_gyro[3]);
#if (MEMS_CHIP == HW_ICM20609)
inv_error_t dmp_get_gyro_bias(long gyro_bias[3]);

/** @brief Gets calibrated gyro value based on raw gyro and gyro bias
* @param[out] calibratedData[3]	Calibred Gyro x,y,z
* @param[in] raw[3]	    Gyro raw data x,y,z
* @param[in] bias[3]	Gyro bias x,y,z
* @return 				0 on success, negative value on error.
*/	
inv_error_t inv_mems_dmp_get_calibrated_gyro(signed long calibratedData[3], short raw[3], signed long bias[3]);
#endif
inv_error_t dmp_get_fifo_length( uint_fast16_t * len );
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
    
/** @brief Gets gyro bias
* @param[out] quat[3]	Gyro bias x,y,z
* @return 				0 on success, negative value on error.
*/	
inv_error_t dmp_get_gyro_bias(short gyro_bias[3]);

/** @brief Gets calibrated gyro value based on raw gyro and gyro bias
* @param[out] calibratedData[3]	Calibred Gyro x,y,z
* @param[in] raw[3]	    Gyro raw data x,y,z
* @param[in] bias[3]	Gyro bias x,y,z
* @return 				0 on success, negative value on error.
*/	
inv_error_t inv_mems_dmp_get_calibrated_gyro(signed long calibratedData[3], signed long raw[3], signed long bias[3]);

#else
 
/** @brief Gets the calibrated gyrometer data 
* @param[out] cal_gyro[3]	the calibrated gyrometer data 
* @return 						0 on success, negative value on error.
*/		
inv_error_t dmp_get_calibrated_gyro(long cal_gyro[3]);

#endif

/** @brief Gets the quaternion  6 axis data 
* @param[out] quat[3]	the quaternion 6 axis data 
* @return 				0 on success, negative value on error.
*/	
inv_error_t dmp_get_6quaternion(long quat[3]);

/** @brief Gets the quaternion  9 axis data 
* @param[out] quat[3]	the quaternion 9 axis data 
* @return 				0 on success, negative value on error.
*/	
inv_error_t dmp_get_9quaternion(long quat[3]);

#if (MEMS_CHIP == HW_ICM20648) 
/** @brief Gets the quaternion  GMRV data 
* @param[out] quat[3]	the quaternion GMRV 6 axis data 
* @return 				0 on success, negative value on error.
*/	
inv_error_t dmp_get_gmrvquaternion(long quat[3]);
#endif

/** @brief Gets the raw compass data 
* @param[out] cal_compass[3]	the raw compass data 
* @return 						0 on success, negative value on error.
*/	
inv_error_t dmp_get_raw_compass(long raw_compass[3]);

/** @brief Gets the calibrated compass data 
* @param[out] cal_compass[3]	the calibrated compass data 
* @return 						0 on success, negative value on error.
*/	
inv_error_t dmp_get_calibrated_compass(long cal_compass[3]);

#if defined MEMS_SECONDARY_DEVICE
/** @brief Gets the pressure data in raw format
* @param[out] pressure[3]	the pressure data in raw format
* pressure[0] = press_mbs<7:0>  : press_lsb<7:0>
* pressure[1] = press_xlbs<7:4> : temp_msb<7:0>
* pressure[2] = temp_lsb<7:0>   : temp_xlsb<7:4>
* @return 						0 on success, negative value on error.
*/	
inv_error_t dmp_get_pressure_raw(short pressure[3]);

/** @brief Gets the pressure data in Pascal and the temperature in degree Celsius
* @param[out] pressure	the pressure data in Pascal
* @param[out] temperature	the temperature data in degree Celsius
* @return 						0 on success, negative value on error.
*/	
inv_error_t dmp_get_pressure_temperature(unsigned int * pressure, signed int * temperature);
#endif

/** @brief Decodes the fifo packet 
* @param[in] fifo_ptr 	pointer to the fifo data
* @param[in] fd 		pointer to the fifo what contains the sensor data
* @return 				0 on success, negative value on error.
*/	
int inv_decode_one_ivory_fifo_packet(struct inv_fifo_decoded_t *fd, const unsigned char *fifo_ptr);

/** @brief Gets the state of the BAC sensor
* @param[in] bac_state	pointer for recuperate the state of BAC
* @return 					0 on success, negative value on error.
*/	
inv_error_t dmp_get_bac_state(uint16_t *bac_state);

/** @brief Gets the timestamp of the BAC sensor
* @param[in] bac_ts	pointer for recuperate the timestamp of BAC
* @return 					0 on success, negative value on error.
*/	
inv_error_t dmp_get_bac_ts(long *bac_ts);

/** @brief Gets the state of the pick up sensor
* @param[in] flip_pickup	pointer for recuperate the state of pickup
* @return 					0 on success, negative value on error.
*/	
inv_error_t dmp_get_flip_pickup_state(uint16_t *flip_pickup);

/** @brief Returns the accelerometer accuracy 
* @return the accelerometer accuracy value
*/	
int inv_get_accel_accuracy(void);

/** @brief Returns the gyrometer accuracy 
* @return the gyrometer accuracy value
*/	
int inv_get_gyro_accuracy(void);

/** @brief Returns the magnetometer accuracy 
* @return the magnetometer accuracy value
*/
int inv_get_mag_accuracy(void);

#if (MEMS_CHIP == HW_ICM20648)
/** @brief Returns the geomagnetic rotation vector accuracy 
* @return the geomagnetic rotation vector accuracy in Q29
*/	
int inv_get_gmrv_accuracy(void);
#endif

#if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
/** @brief Returns the rotation vector accuracy 
* @return the rotation vector accuracy value in Q29
*/	
int inv_get_rv_accuracy(void);
#endif

/** @brief Resets the fifo
* @param[in] value 	0=no, 1=yes
* @return 			0 on success, negative value on error.
*/	
inv_error_t mpu_set_FIFO_RST_Diamond(unsigned char value);


/** @brief Mirror DMP HW FIFO into SW FIFO
* @param[inout] left_in_fifo 	pointer to number of bytes in SW FIFO : before function is called, must contain number of
				bytes still present in FIFO which must not be overwritten
				after function is called, will contain number of bytes present in SW FIFO to be analyzed
* @param[out] total_sample_cnt 	number of total sensor samples present in SW FIFO
* @param[out] sample_cnt_array 	array of number of sensor samples present in SW FIFO for each sensor, should be inited to 0 before being called
* @return 			0 on success, negative value on error.
*/	
inv_error_t inv_mems_fifo_swmirror(int *left_in_fifo, unsigned short * total_sample_cnt, unsigned short * sample_cnt_array);


/** @brief Pop one sample out of SW FIFO
* @param[out] user_header 	Header value read from SW FIFO
* @param[out] user_header2 	Header2 value read from SW FIFO
* @param[inout] left_in_fifo 	Contains number of bytes still be parsed from SW FIFO
* @return 			0 on success, negative value on error.
*/	
inv_error_t inv_mems_fifo_pop( unsigned short *user_header, unsigned short *user_header2, int *left_in_fifo);
inv_error_t dmp_read_fifo(unsigned char *data, uint_fast16_t len);
#ifdef __cplusplus
}
#endif
#endif // INV_MEMS_MPU_FIFO_CONTROL_H__GQWDHE__


/** @} */

