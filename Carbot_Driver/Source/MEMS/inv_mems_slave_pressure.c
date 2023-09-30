/*
* ________________________________________________________________________________________________________
* Copyright © 2015-2015 InvenSense Inc. Portions Copyright © 2015-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/
#ifndef MEMS_20609

#include "inv_mems_slave_pressure.h"

#include "inv_mems_drv_hook.h"
#include "inv_mems_defines.h"
#include "inv_mems_data_converter.h"               // to get inv_q30_mult
#include "inv_mems_hw_config.h"
#include "inv_mems_transport.h"
#include "inv_mems_secondary_transport.h"
#include "inv_mems_slave_compass.h"
#include "inv_mems_interface_mapping.h"

#define REG_BMP280_ID			(0xD0)
#define DATA_BMP_ID             (0x58)
#define BMP280_PRESS_MSB	    (0xF7)
#define BMP280_CAL00	        (0x88)
#define BMP280_CTRL_MEAS	    (0xF4)
#define BMP280_NORMAL_MODE                   0x3
#define BMP280_OVERSAMPLING_2X               0x02
#define BMP280_OVERSAMPLING_16X              0x05
#define BMP280_ULTRAHIGHRESOLUTION_OSRS_P    (BMP280_OVERSAMPLING_16X << 2)
#define BMP280_ULTRAHIGHRESOLUTION_OSRS_T    (BMP280_OVERSAMPLING_2X << 5)
#define BMP280_RESET_REG        (0xE0)
#define BMP280_SOFT_RESET       (0xB6)


static uint8_t secondary_resume_state = 0;
static unsigned short sBmpCompensationParameters[12];
static int32_t sBmp280Fine;

/*
 *  inv_mems_pressure_setup_bmp() - Configure bmp series pressure.
 */
int inv_mems_pressure_setup_bmp()
{
    int result;
    unsigned char data[4];
    uint8_t lIdx = 0;

    /* First do a soft reset of the sensor */
    result = inv_execute_write_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_RESET_REG, BMP280_SOFT_RESET);

    if (result)
    {
        inv_log("Write secondary error: Pressure.\r\n");
        return result;
    }

    /* Then read whoami */
    result = inv_execute_read_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, REG_BMP280_ID, 1, data);

    if (result)
    {
        inv_log("Read secondary error: Pressure.\r\n");
        return result;
    }

    if (data[0] != DATA_BMP_ID)
    {
        inv_log("Pressure not found!!\r\n");
        return -1;
    }

    inv_log("Pressure found.\r\n");

    /* Now extract compensation parameters from the sensor */
    do
    {
        result = inv_execute_read_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CAL00 + (lIdx * 2), 2, (unsigned char *)&sBmpCompensationParameters[lIdx]);

        if (result)
            return result;

        lIdx++;
    }
    while (lIdx < 12);

    secondary_resume_state = 1;
    /* An put the sensor in low power */
    return inv_mems_pressure_suspend_bmp();
}

int inv_mems_pressure_suspend_bmp()
{
    int result;

    if (!secondary_resume_state)
        return 0;

    /* Need to write to a BMP280 register to stop automatic acquisition */
    if (inv_mems_compass_getstate() == 0)
    {
        /* Switch off I2C interface only if pressure is alone */
        result = inv_execute_write_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CTRL_MEAS, 0);
    }
    else
    {
        /* In case compass is still on, I2C is already active so just request for the write but do not disable whole I2C */
        result = inv_write_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CTRL_MEAS, 0);
        inv_sleep(SECONDARY_INIT_WAIT);
    }

    if (result)
        return result;

    /* Disable I2C channels for pressure and ALS since ALS channel only used for pressure */
    result = inv_mems_secondary_stop_channel(ALS_I2C_SLV);
    result |= inv_mems_secondary_stop_channel(PRESSURE_I2C_SLV);

    if (result)
        return result;

    secondary_resume_state = 0;

    return result;
}

int inv_mems_pressure_resume_bmp()
{
    int result;

    if (secondary_resume_state)
    {
        // we expect this part to be executed only if pressure is already on

        /* disable I2C interface so that configuration does not do a mess */
        result = inv_mems_secondary_disable_i2c();

        /* switch off als channel to reconfigure it safely */
        result |= inv_mems_secondary_stop_channel(ALS_I2C_SLV);

        if (inv_mems_compass_getstate() == 0)
        {
            /* if pressure is alone enabled (i.e. compass is off), then need to feed DMP with dummy data and matching worst case requirements being : 10 bytes for compass and 8 bytes for ALS */
            result |= inv_mems_secondary_stop_channel(COMPASS_I2C_SLV_READ);
            result |= inv_read_secondary(COMPASS_I2C_SLV_READ, PRESSURE_CHIP_ADDR, BMP280_CAL00, 10);
            result |= inv_read_secondary(ALS_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CAL00, 8);
        }
        else
        {
            /* if compass and pressure are to be enabled together, then still need to feed DMP with dummy data so that it has x bytes for compass and 18 bytes of dummy data as a total */
            /* So read compass I2C read length and determine number of dummy bytes to be read for fake ALS */
            uint8_t lCompassDataSize;
            result |= inv_read_mems_reg(REG_I2C_SLV0_CTRL, 1, &lCompassDataSize);
            lCompassDataSize &= 0x0F; // bits 3:0 matches number of bytes to be read;
            result |= inv_read_secondary(ALS_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CAL00, 18 - lCompassDataSize);
        }

        /* now that all 4 channels are reconfigured if needed, we can enable back I2C interface */
        result |= inv_mems_secondary_enable_i2c();

        return result;
    }
    else
    {
        /* Reconfigure I2C ODR to min value so that the write is sent as quickly as possible on I2C links, otherwise, the write is delayed by REG_I2C_MST_ODR_CONFIG
           and then we would have to add a sleep of 300ms, which would make the system unresponsive during BMP280 enabling time, which would be unacceptable */
        inv_mems_secondary_saveI2cOdr();
        result = inv_write_single_mems_reg(REG_I2C_MST_ODR_CONFIG, 5);

        /* Configure BMP280 in normal mode, so that it automatically perform regular data acquisition itself without having to restart it
           This way DMP is fed regularly automatically */
        if (inv_mems_compass_getstate() == 0)
        {
            result |= inv_execute_write_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CTRL_MEAS, BMP280_NORMAL_MODE | BMP280_ULTRAHIGHRESOLUTION_OSRS_P | BMP280_ULTRAHIGHRESOLUTION_OSRS_T);
        }
        else
        {
            result |= inv_write_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CTRL_MEAS, BMP280_NORMAL_MODE | BMP280_ULTRAHIGHRESOLUTION_OSRS_P | BMP280_ULTRAHIGHRESOLUTION_OSRS_T);
            inv_sleep(SECONDARY_INIT_WAIT);
            result |= inv_mems_secondary_stop_channel(PRESSURE_I2C_SLV);
        }

        inv_mems_secondary_restoreI2cOdr();

        if (result)
            return result;

        /* Dedicated I2C slave is used to read data from pressure */

        /* I2C slave is enabled, read automatically periodically 6 bytes from here */
        if (inv_mems_compass_getstate() == 0)
        {
            /* if pressure is alone enabled (i.e. compass is off), then need to feed DMP with dummy data and matching worst case requirements being : 10 bytes for compass and 8 bytes for ALS */
            result = inv_read_secondary(COMPASS_I2C_SLV_READ, PRESSURE_CHIP_ADDR, BMP280_CAL00, 10);
            result |= inv_read_secondary(ALS_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CAL00, 8);
        }
        else
        {
            /* if compass and pressure are to be enabled together, then still need to feed DMP with dummy data so that it has x bytes for compass and 18 bytes of dummy data as a total */
            /* So read compass I2C read length and determine number of dummy bytes to be read for fake ALS */
            uint8_t lCompassDataSize;
            result = inv_read_mems_reg(REG_I2C_SLV0_CTRL, 1, &lCompassDataSize);
            lCompassDataSize &= 0x0F; // bits 3:0 matches number of bytes to be read;
            result |= inv_read_secondary(ALS_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_CAL00, 18 - lCompassDataSize);
        }

        /* After the 18 bytes, DMP knows it is 6 bytes for pressure data */
        /* I2C slave is enabled, read automatically periodically 6 bytes from here */
        result |= inv_read_secondary(PRESSURE_I2C_SLV, PRESSURE_CHIP_ADDR, BMP280_PRESS_MSB, 6);

        if (result)
            return result;

        /* everything is ready, kick off I2C interface if not already enabled through compass*/
        if (inv_mems_compass_getstate() == 0)
            result = inv_mems_secondary_enable_i2c();

        secondary_resume_state = 1;

        return result;
    }
}

char inv_mems_pressure_getstate(void)
{
    return secondary_resume_state;
}


static int32_t BMP280_compensateTemp(int32_t parTemp)
{
    int32_t var1;
    int32_t var2;
    int32_t tempInDegC;

    var1 = ((((parTemp >> 3) - ((int32_t)sBmpCompensationParameters[0] << 1))) * ((int32_t)sBmpCompensationParameters[1])) >> 11;

    var2 = (((((parTemp >> 4) - ((int32_t)sBmpCompensationParameters[0])) * ((parTemp >> 4) - ((
                  int32_t)sBmpCompensationParameters[0]))) >> 12) * ((int32_t)sBmpCompensationParameters[2])) >> 14;

    sBmp280Fine = var1 + var2;

    tempInDegC = (sBmp280Fine * 5 + 128) >> 8;

    return tempInDegC;
}

static uint32_t BMP280_compensatePress(int32_t parPress)
{
    int32_t var1;
    int32_t var2;
    uint32_t pressInPa;

    var1 = (((int32_t)sBmp280Fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)sBmpCompensationParameters[8];
    var2 = var2 + ((var1 * ((int32_t)sBmpCompensationParameters[7])) << 1);
    var2 = (var2 >> 2) + ((int32_t)sBmpCompensationParameters[6] << 16);
    var1 = (((sBmpCompensationParameters[5] * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((
                int32_t)sBmpCompensationParameters[4]) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)sBmpCompensationParameters[3])) >> 15);

    if (!var1)
    {
        return 0;   //Avoid exception caused division by 0
    }

    pressInPa = (((uint32_t)(((int32_t)1048576) - parPress) - (var2 >> 12))) * 3125;

    if (pressInPa < 0x80000000)
    {
        pressInPa = (pressInPa << 1) / ((uint32_t)var1);
    }
    else
    {
        pressInPa = (pressInPa / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)sBmpCompensationParameters[11]) * ((int32_t)(((pressInPa >> 3) *
            (pressInPa >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(pressInPa >> 2)) * ((int32_t) sBmpCompensationParameters[10])) >> 13;
    pressInPa = (uint32_t)((int32_t)pressInPa + ((var1 + var2 + sBmpCompensationParameters[9]) >> 4));

    return pressInPa;
}

int inv_mems_pressure_bmp280_rawToPascal(unsigned short raw_data[3], unsigned int *pressPascal, signed int *temperatureCelsius)
{
    int T, P;

    P = (((unsigned int)raw_data[0]) << 4) + (((((unsigned int)raw_data[1]) & 0xF000) >> 12) & 0xF);
    T = (((((unsigned int)raw_data[1])) & 0x00FF) << 12) + ((((unsigned int)raw_data[2]) & 0xFFF0) >> 4);

    if ((P == 0x80000) && (T == 0x80000))
        return -1; // data is not ready

    *temperatureCelsius = BMP280_compensateTemp(T);
    *pressPascal =  BMP280_compensatePress(P);

    return 0;
}

#endif


