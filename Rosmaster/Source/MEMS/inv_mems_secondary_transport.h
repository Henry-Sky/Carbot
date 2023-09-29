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
/** @defgroup	inv_mems_secondary_transport	inv_secondary_transport
    @ingroup 	Mems_driver
    @{
*/
#ifndef INV_MEMS_SECONDARY_TRANSPORT_H__
#define INV_MEMS_SECONDARY_TRANSPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @brief I2C from secondary device can stand on up to 4 channels. To perform automatic read and feed DMP :
- channel 0 is reserved for compass reading data
- channel 1 is reserved for compass writing one-shot acquisition register
- channel 2 is reserved for als reading data
- channel 3 is reserved for pressure reading data*/
#define COMPASS_I2C_SLV_READ		0
#define COMPASS_I2C_SLV_WRITE		1
#define ALS_I2C_SLV					2
#define PRESSURE_I2C_SLV			3

/** @brief Initializes the register for the i2c communication*/
void inv_init_secondary(void);

/** @brief Reads data in i2c a secondary device
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be read on the secondary device
* @param[in] len 	Size of data to be read
* @return 	   		0 in case of success, -1 for any error
*/
int inv_read_secondary(int index, unsigned char addr, unsigned char reg, char len);

/** @brief Reads data in i2c a secondary device directly 
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be read on the secondary device
* @param[in] len 	Size of data to be read
* @param[out] d 	pointer to the data to be read
* @return 	   		0 in case of success, -1 for any error
*/
int inv_execute_read_secondary(int index, unsigned char addr, int reg, int len, uint8_t *d);

/** @brief Writes data in i2c a secondary device
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be write on the secondary device
* @param[in] v 		the data to be written
* @return 	   		0 in case of success, -1 for any error
*/
int inv_write_secondary(int index, unsigned char addr, unsigned char reg, char v);

/** @brief Writes data in i2c a secondary device directly
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be write on the secondary device
* @param[in] v 		the data to be written
* @return 	   		0 in case of success, -1 for any error
*/
int inv_execute_write_secondary(int index, unsigned char addr, int reg, uint8_t v);

/** @brief Save current secondary I2C ODR configured
*/
void inv_mems_secondary_saveI2cOdr(void);

/** @brief Restore secondary I2C ODR configured based on the one saved with inv_mems_secondary_saveI2cOdr()
*/
void inv_mems_secondary_restoreI2cOdr(void);

/** @brief Stop one secondary I2C channel by writing 0 in its control register
* @param[in] index  	the channel id to be stopped
* @return 	   		0 in case of success, -1 for any error
* @warning It does not stop I2C secondary interface, just one channel
*/
inv_error_t inv_mems_secondary_stop_channel(int index);

/** @brief Enable secondary I2C interface
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_mems_secondary_enable_i2c(void);

/** @brief Stop secondary I2C interface
* @return 	   		0 in case of success, -1 for any error
* @warning It stops all I2C transactions, whatever the channel status
*/
inv_error_t inv_mems_secondary_disable_i2c(void);

/** @brief Changes the odr of the I2C master
* @param[in] divider  	frequency divider to BASE_SAMPLE_RATE
* @param[out] effectiveDivider  	divider finally applied to base sample rate, at which data will be actually read on I2C bus
* @return 	   		0 in case of success, -1 for any error
*/
int inv_mems_secondary_set_odr(int divider, unsigned int* effectiveDivider);

#ifdef __cplusplus
}
#endif

#endif // INV_MEMS_SECONDARY_TRANSPORT_H__

/** @} */
