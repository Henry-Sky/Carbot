/*
Copyright (C) 2014 InvenSense Corporation, All Rights Reserved.
*/

#ifndef INV_SLAVE_PRESSURE_H_SDFWQN__
#define INV_SLAVE_PRESSURE_H_SDFWQN__

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initializes the pressure sensor
* @return 	0 in case of success, -1 for any error
*/
int inv_mems_pressure_setup_bmp(void);

/** @brief Stops the pressure sensor
* @return 	0 in case of success, -1 for any error
*/
int inv_mems_pressure_suspend_bmp(void);

/** @brief Starts the pressure sensor, or reconfigure associated I2C channels based on compass status if already started
* @return 	0 in case of success, -1 for any error
*/
int inv_mems_pressure_resume_bmp(void);

/** @brief Get pressure power status
* @return 	1 in case pressure is enabled, 0 if not started
*/
char inv_mems_pressure_getstate(void);

/** @brief Convert pressure from DMP format to Pascal
* @param[in] raw_data array of 3 16bits in following format :
* - raw_data [0] = press_msb<7:0>   |press_lsb<7:0>
* - raw_data [1] = press_xlsb<7:4>|0|temp_msb<7:0>
* - raw_data [2] = temp_lsb<7:0>    |temp_xlsb<7:4>|0
* @param[out] pressPascal resulting pressure value in Pascal
* @param[out] temperatureCelsius resulting temperature in degree Celsius multiplied by 100
* @return 	1 in case pressure is enabled, 0 if not started
*/
int inv_mems_pressure_bmp280_rawToPascal(unsigned short raw_data[3], unsigned int *pressPascal, signed int *temperatureCelsius);

#ifdef __cplusplus
}
#endif

#endif // INV_SLAVE_PRESSURE_H_SDFWQN__

