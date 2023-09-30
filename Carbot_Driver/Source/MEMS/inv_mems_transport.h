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
/** @defgroup	inv_mems_transport	inv_transport
    @ingroup 	Mems_driver
    @{
*/
#ifndef INV_MEMS_TRANSPORT_H__
#define INV_MEMS_TRANSPORT_H__

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Writes data to a register in DMP memory 
* @param[in] reg  	DMP memory address
* @param[in] length number of byte to be written
* @param[out] data	output data from the register
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_write_mems(unsigned short reg, unsigned int length, const unsigned char *data);


/** @brief Reads data to a register in DMP memory 
* @param[in] reg  	DMP memory address
* @param[in] length number of byte to be read
* @param[out] data	output data from the register
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_read_mems(unsigned short reg, unsigned int length, unsigned char *data);


/** @brief Writes data from a register on mems.
* @param[in] reg  	DMP memory address
* @param[in] length number of byte to be written
* @param[out] data	output data from the register
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_write_mems_reg(uint16_t reg, unsigned int length, const unsigned char *data);

/** @brief Writes a single byte of data from a register on mems with no power control
* @param[in] reg  	DMP memory address
* @param[out] data	Data to be written
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_write_single_mems_reg_core(uint16_t reg, const unsigned char data);

/** @brief Writes a single byte of data from a register on mems.
* @param[in] reg  	DMP memory address
* @param[out] data	Data to be written
* @return 	   		0 in case of success, -1 for any error
*/
inv_error_t inv_write_single_mems_reg(uint16_t reg, const unsigned char data);

/** @brief Reads data from a register on mems with no power control
* @param[in]  	Register address
* @param[in]  	Length of data
* @param[out]  	output data from the register
* @return 	   	0 in case of success, -1 for any error
*/
inv_error_t inv_read_mems_reg_core(uint16_t reg, unsigned int length, unsigned char *data);

/** @brief Reads data from a register on mems.
* @param[in]  	Register address
* @param[in]  	Length of data
* @param[out]  	output data from the register
* @return 	   	0 in case of success, -1 for any error
*/
inv_error_t inv_read_mems_reg(uint16_t reg, unsigned int length, unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif // INV_MEMS_TRANSPORT_H__

/** @} */

