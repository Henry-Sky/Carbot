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
/** @defgroup inv_mems_drv_hook inv_drv_hook
	@ingroup  Mems_common
	@{
*/
#include "int_types.h"

#ifndef __INV_MEMS_DRV_HOOK_H
#define __INV_MEMS_DRV_HOOK_H

/** @brief Max size that can be read across I2C or SPI data lines */
#define INV_MAX_SERIAL_READ 16
/** @brief Max size that can be written across I2C or SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

#define inv_log(str)

/** @brief Hook function to be implemented by at integration level to actually write to serial interface
* @param[in] reg        slave register address we want to write to
* @param[in] length     length in bytes of data to be written
* @param[in] data       pointer on where data to be written to SPI slave are stored
* @return 				0 in case of success, -1 for any error, -2 in case a timeout occured during SPI transaction */
int inv_serial_interface_write_hook(uint16_t reg, uint32_t length, const uint8_t *data);

/** @brief  Hook function to be implemented by at integration level to actually read to serial interface
* @param[in] reg        slave register address we want to read from
* @param[in] length     length in bytes of data to be read
* @param[out] data      pointer on where data read is to be stored
* @return 				0 in case of success, -1 for any error, -2 in case a timeout occured during SPI transaction */
int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data);

/** @brief Function to be sleep nTime in ms
* @param[in] nTime	time to sleep in ms
*/
void inv_sleep(unsigned long nTime);

/** @brief Function to be sleep nTime in 100 us
* @param[in] nTime	time to sleep in 100 us
*/
void inv_sleep_100us(unsigned long nHowMany100MicroSecondsToSleep ); // time in 100 us

/** @brief Hook function to be implemented by at integration level to actually write to serial interface
* @return the tick counter */
extern long long inv_get_tick_count(void);

#endif /* __INV_MEMS_DRV_HOOK_H */

/** @} */
