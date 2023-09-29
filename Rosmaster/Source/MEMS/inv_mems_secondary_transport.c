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

#include "inv_mems_transport.h"

#include "inv_mems_drv_hook.h"
#include "inv_mems_defines.h"
#ifndef MEMS_20609
    #include "inv_mems_base_driver.h"
#else
    #include "inv_mems_base_driver_20609.h"
#endif
#include "invn_types.h"

#include "inv_mems_secondary_transport.h"

/**
 *  struct inv_secondary_reg - secondary registers data structure.
 *  @addr:       address of the slave.
 *  @reg: register address of slave.
 *  @ctrl: control register.
 *  @d0: data out register.
 */
struct inv_secondary_reg
{
    unsigned short addr;
    unsigned short reg;
    unsigned short ctrl;
    unsigned short d0;
};
static struct inv_secondary_reg slv_reg[4];

#if (MEMS_CHIP != HW_ICM20609)
    static unsigned char sSavedI2cOdr;
#endif
void inv_init_secondary(void)
{
    slv_reg[0].addr = REG_I2C_SLV0_ADDR;
    slv_reg[0].reg  = REG_I2C_SLV0_REG;
    slv_reg[0].ctrl = REG_I2C_SLV0_CTRL;
    slv_reg[0].d0   = REG_I2C_SLV0_DO;

    slv_reg[1].addr = REG_I2C_SLV1_ADDR;
    slv_reg[1].reg  = REG_I2C_SLV1_REG;
    slv_reg[1].ctrl = REG_I2C_SLV1_CTRL;
    slv_reg[1].d0   = REG_I2C_SLV1_DO;

    slv_reg[2].addr = REG_I2C_SLV2_ADDR;
    slv_reg[2].reg  = REG_I2C_SLV2_REG;
    slv_reg[2].ctrl = REG_I2C_SLV2_CTRL;
    slv_reg[2].d0   = REG_I2C_SLV2_DO;
    #if (MEMS_CHIP != HW_ICM20609)
    slv_reg[3].addr = REG_I2C_SLV3_ADDR;
    slv_reg[3].reg  = REG_I2C_SLV3_REG;
    slv_reg[3].ctrl = REG_I2C_SLV3_CTRL;
    slv_reg[3].d0   = REG_I2C_SLV3_DO;
    #endif
    /* Make sure that by default all channels are disabled
    To not inherit from a previous configuration from a previous run*/
    inv_mems_secondary_stop_channel(0);
    inv_mems_secondary_stop_channel(1);
    inv_mems_secondary_stop_channel(2);
    inv_mems_secondary_stop_channel(3);
}

/* the following functions are used for configuring the secondary devices */

/*
* inv_configure_secondary_read(): set secondary registers for reading.
The chip must be set as bank 3 before calling.
* This is derived from inv_read_secondary in linux...
* for now, uses a very simple data struct for the registers
*
* index gives the mapping to the particular SLVx registers
* addr is the physical address of the device to be accessed
* reg is the device register we wish to access
* len is the number of bytes to be read
*
*/
inv_error_t inv_read_secondary(int index, inv_i2c_addr_t addr, unsigned char reg, char len)
{
    inv_error_t result = 0;
    unsigned char data;

    data = INV_MPU_BIT_I2C_READ | addr;
    result |= inv_write_mems_reg(slv_reg[index].addr, 1, &data);

    data = reg;
    result |= inv_write_mems_reg(slv_reg[index].reg, 1, &data);

    data = INV_MPU_BIT_SLV_EN | len;
    result |= inv_write_mems_reg(slv_reg[index].ctrl, 1, &data);

    return result;
}

inv_error_t inv_execute_read_secondary(int index, inv_i2c_addr_t addr, int reg, int len, uint8_t *d)
{
    inv_error_t result = 0;

    result |= inv_read_secondary(index, addr, reg, len);

    result |= inv_mems_secondary_enable_i2c();

    inv_sleep(SECONDARY_INIT_WAIT);

    result |= inv_mems_secondary_disable_i2c();

    result |= inv_read_mems_reg(REG_EXT_SLV_SENS_DATA_00, len, d);

    result |= inv_mems_secondary_stop_channel(index);

    return result;
}

/*
* inv_write_secondary(): set secondary registers for writing?.
The chip must be set as bank 3 before calling.
* This is derived from inv_write_secondary in linux...
* for now, uses a very simple data struct for the registers
*
* index gives the mapping to the particular SLVx registers
* addr is the physical address of the device to be accessed
* reg is the device register we wish to access
* len is the number of bytes to be read
*
*/
inv_error_t inv_write_secondary(int index, inv_i2c_addr_t addr, unsigned char reg, char v)
{
    inv_error_t result = 0;
    unsigned char data;

    data = (unsigned char)addr;
    result |= inv_write_mems_reg(slv_reg[index].addr, 1, &data);

    data = reg;
    result |= inv_write_mems_reg(slv_reg[index].reg, 1, &data);

    data = v;
    result |= inv_write_mems_reg(slv_reg[index].d0, 1, &data);

    data = INV_MPU_BIT_SLV_EN | 1;
    result |= inv_write_mems_reg(slv_reg[index].ctrl, 1, &data);

    return result;
}

inv_error_t inv_execute_write_secondary(int index, inv_i2c_addr_t addr, int reg, uint8_t v)
{
    inv_error_t result = 0;

    result |= inv_write_secondary(index, addr, reg, v);

    result |= inv_mems_secondary_enable_i2c();

    inv_sleep(SECONDARY_INIT_WAIT);

    result |= inv_mems_secondary_disable_i2c();

    result |= inv_mems_secondary_stop_channel(index);

    return result;
}
void inv_mems_secondary_saveI2cOdr(void)
{
    #if (MEMS_CHIP != HW_ICM20609)
    inv_read_mems_reg(REG_I2C_MST_ODR_CONFIG, 1, &sSavedI2cOdr);
    #else
    return;
    #endif
}

void inv_mems_secondary_restoreI2cOdr(void)
{
    #if (MEMS_CHIP != HW_ICM20609)
    inv_write_single_mems_reg(REG_I2C_MST_ODR_CONFIG, sSavedI2cOdr);
    #else
    return;
    #endif
}

inv_error_t inv_mems_secondary_stop_channel(int index)
{
    return inv_write_single_mems_reg(slv_reg[index].ctrl, 0);
}

inv_error_t inv_mems_secondary_enable_i2c(void)
{
    base_state.user_ctrl |= BIT_I2C_MST_EN;
    return inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);
}

inv_error_t inv_mems_secondary_disable_i2c(void)
{
    base_state.user_ctrl &= ~BIT_I2C_MST_EN;
    return inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);
}


int inv_mems_secondary_set_odr(int divider, unsigned int* effectiveDivider)
{
    int mst_odr_config = 0;

    // find 2^x = divider to fit BASE_SAMPLE_RATE/2^REG_I2C_MST_ODR_CONFIG
    do
    {
        divider >>= 1;
        mst_odr_config++;
    }
    while(divider >> 1);

    if (mst_odr_config < MIN_MST_ODR_CONFIG)
        mst_odr_config = MIN_MST_ODR_CONFIG;

    *effectiveDivider = 1 << mst_odr_config;

    return	inv_set_secondary_divider((unsigned char)mst_odr_config);
}
