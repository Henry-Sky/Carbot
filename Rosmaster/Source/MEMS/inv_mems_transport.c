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
#include "inv_mems_hw_config.h"

#if (MEMS_CHIP == HW_ICM20648)
    #include "inv_mems_base_control.h"
#endif
static unsigned char lLastBankSelected = 0xFF;

static uint8_t check_reg_access_lp_disable(unsigned short reg)
{
    switch(reg)
    {
        case 0x05:   /** LP_CONFIG reg */
        case 0x06:   /** PWR_MGMT_1 reg */
        case 0x07:   /** PWR_MGMT_2 reg */
        case 0x0f:   /** INT_PIN_CFG reg */
        case 0x10:   /** INT_ENABLE reg */
        case 0x70:   /** FIFO_COUNTH reg */
        case 0x71:   /** FIFO_COUNTL reg */
        case 0x72:   /** FIFO_R_W reg */
            #if (MEMS_CHIP == HW_ICM20648)
            return inv_get_batch_mode_status();
            #endif

        case 0x76:   /** FIFO_CFG reg */
        case 0x7e:   /** MEM_BANK_SEL reg */
        case 0x7f:   /** REG_BANK_SEL reg */
            return 0;

        //		break;
        default:

            break;
    }

    return 1;
}

/**
*  @brief      Set up the register bank register for accessing registers in 20630.
*  @param[in]  register bank number
*  @return     0 if successful.
*/

static inv_error_t inv_set_bank(unsigned char bank)
{
    #if (MEMS_CHIP != HW_ICM20609)
    int result;
    static unsigned char reg;

    //if bank reg was set before, just return
    static unsigned char lastBank = 0x7E;

    if(bank == lastBank)
        return 0;
    else
        lastBank = bank;

    result = inv_serial_interface_read_hook(REG_BANK_SEL, 1, &reg);

    if (result)
        return result;

    reg &= 0xce;
    reg |= (bank << 4);
    result = inv_serial_interface_write_hook(REG_BANK_SEL, 1, &reg);

    return result;
    #else
    return 0;
    #endif
}

/* the following functions are used for configuring the secondary devices */

/**
*  @brief      Write data to a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Length of data
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
inv_error_t inv_write_mems_reg(uint16_t reg, unsigned int length, const unsigned char *data)
{
    inv_error_t result = 0;
    unsigned int bytesWrite = 0;
    unsigned char regOnly = (unsigned char)(reg & 0x7F);

    unsigned char power_state = inv_get_chip_power_state();

    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    if(check_reg_access_lp_disable(reg))    // Check if register needs LP_EN to be disabled
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 0);  //Disable LP_EN

    result |= inv_set_bank(reg >> 7);

    while (bytesWrite < length)
    {
        int thisLen = min(INV_MAX_SERIAL_WRITE, length - bytesWrite);

        result |= inv_serial_interface_write_hook(regOnly + bytesWrite, thisLen, &data[bytesWrite]);

        if (result)
            return result;

        bytesWrite += thisLen;
    }

    if(check_reg_access_lp_disable(reg))   //Enable LP_EN since we disabled it at begining of this function.
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    return result;
}

/**
*  @brief      Write single byte of data to a register on MEMs with no power control
*  @param[in]  Register address
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
inv_error_t inv_write_single_mems_reg_core(uint16_t reg, const unsigned char data)
{
    inv_error_t result = 0;
    unsigned char regOnly = (unsigned char)(reg & 0x7F);

    result |= inv_set_bank(reg >> 7);
    result |= inv_serial_interface_write_hook(regOnly, 1, &data);

    return result;
}

/**
*  @brief      Write single byte of data to a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
inv_error_t inv_write_single_mems_reg(uint16_t reg, const unsigned char data)
{
    inv_error_t result = 0;

    unsigned char power_state = inv_get_chip_power_state();

    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    if(check_reg_access_lp_disable(reg))   // Check if register needs LP_EN to be disabled
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 0);  //Disable LP_EN

    result |= inv_write_single_mems_reg_core(reg, data);

    if(check_reg_access_lp_disable(reg))   //Enable LP_EN since we disabled it at begining of this function.
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    return result;
}

/**
*  @brief      Read data from a register on MEMs with no power control
*  @param[in]  Register address
*  @param[in]  Length of data
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
inv_error_t inv_read_mems_reg_core(uint16_t reg, unsigned int length, unsigned char *data)
{
    inv_error_t result = 0;
    unsigned int bytesRead = 0;
    unsigned char regOnly = (unsigned char)(reg & 0x7F);
    #if (MEMS_CHIP != HW_ICM30630)
    unsigned char dat[INV_MAX_SERIAL_READ + 1];
    int i;
    #endif

    result |= inv_set_bank(reg >> 7);

    while (bytesRead < length)
    {
        int thisLen = min(INV_MAX_SERIAL_READ, length - bytesRead);
        #if (MEMS_CHIP != HW_ICM30630)

        if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
        {
            result |= inv_serial_interface_read_hook(regOnly + bytesRead, thisLen, &dat[bytesRead]);
        }
        else
        {
            result |= inv_serial_interface_read_hook(regOnly + bytesRead, thisLen, &data[bytesRead]);
        }

        #else
        result |= inv_serial_interface_read_hook(regOnly + bytesRead, thisLen, &data[bytesRead]);
        #endif

        if (result)
            return result;

        bytesRead += thisLen;
    }

    #if(MEMS_CHIP != HW_ICM30630)

    if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
    {
        for (i = 0; i < length; i++)
        {
            *data++ = dat[i + 1];
        }
    }

    #endif

    return result;
}

/**
*  @brief      Read data from a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Length of data
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
inv_error_t inv_read_mems_reg(uint16_t reg, unsigned int length, unsigned char *data)
{
    inv_error_t result = 0;

    unsigned char power_state = inv_get_chip_power_state();

    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    if(check_reg_access_lp_disable(reg))   // Check if register needs LP_EN to be disabled
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 0);  //Disable LP_EN

    result = inv_read_mems_reg_core(reg, length, data);

    if(check_reg_access_lp_disable(reg))   //Enable LP_EN since we disabled it at begining of this function.
        result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    return result;
}

/**
*  @brief      Read data from a register in DMP memory
*  @param[in]  DMP memory address
*  @param[in]  number of byte to be read
*  @param[in]  input data from the register
*  @return     0 if successful.
*/
inv_error_t inv_read_mems(unsigned short reg, unsigned int length, unsigned char *data)
{
    int result = 0;
    unsigned int bytesWritten = 0;
    unsigned int thisLen;
    #if(MEMS_CHIP != HW_ICM30630)
    unsigned char i, dat[INV_MAX_SERIAL_READ + 1];
    #endif
    unsigned char power_state = inv_get_chip_power_state();
    unsigned char lBankSelected;
    unsigned char lStartAddrSelected;

    if(!data)
        return -1;

    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 0);

    result |= inv_set_bank(0);
    lBankSelected = (reg >> 8);

    if (lBankSelected != lLastBankSelected)
    {
        result |= inv_serial_interface_write_hook(REG_MEM_BANK_SEL, 1, &lBankSelected);

        if (result)
            return result;

        lLastBankSelected = lBankSelected;
    }

    while (bytesWritten < length)
    {
        lStartAddrSelected = (reg & 0xff);
        /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
        result |= inv_serial_interface_write_hook(REG_MEM_START_ADDR, 1, &lStartAddrSelected);

        if (result)
            return result;

        thisLen = min(INV_MAX_SERIAL_READ, length - bytesWritten);

        #if(MEMS_CHIP != HW_ICM30630)

        /* Write data */
        if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
        {
            result |= inv_serial_interface_read_hook(REG_MEM_R_W, thisLen, &dat[bytesWritten]);
        }
        else
        {
            result |= inv_serial_interface_read_hook(REG_MEM_R_W, thisLen, &data[bytesWritten]);
        }

        #else
        /* Write data */
        result |= inv_serial_interface_read_hook(REG_MEM_R_W, thisLen, &data[bytesWritten]);
        #endif

        if (result)
            return result;

        bytesWritten += thisLen;
        reg += thisLen;
    }

    #if(MEMS_CHIP != HW_ICM30630)

    if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
    {
        for (i = 0; i < length; i++)
        {
            *data++ = dat[i + 1];
        }
    }

    #endif

    //Enable LP_EN since we disabled it at begining of this function.
    result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    return result;
}

/**
*  @brief       Write data to a register in DMP memory
*  @param[in]   DMP memory address
*  @param[in]   number of byte to be written
*  @param[out]  output data from the register
*  @return     0 if successful.
*/
inv_error_t inv_write_mems(unsigned short reg, unsigned int length, const unsigned char *data)
{
    int result = 0;
    unsigned int bytesWritten = 0;
    unsigned int thisLen;
    unsigned char lBankSelected;
    unsigned char lStartAddrSelected;

    unsigned char power_state = inv_get_chip_power_state();

    if(!data)
        return -1;

    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 0);

    result |= inv_set_bank(0);
    lBankSelected = (reg >> 8);

    if (lBankSelected != lLastBankSelected)
    {
        result |= inv_serial_interface_write_hook(REG_MEM_BANK_SEL, 1, &lBankSelected);

        if (result)
            return result;

        lLastBankSelected = lBankSelected;
    }

    while (bytesWritten < length)
    {
        lStartAddrSelected = (reg & 0xff);
        /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
        result |= inv_serial_interface_write_hook(REG_MEM_START_ADDR, 1, &lStartAddrSelected);

        if (result)
            return result;

        thisLen = min(INV_MAX_SERIAL_WRITE, length - bytesWritten);

        /* Write data */
        result |= inv_serial_interface_write_hook(REG_MEM_R_W, thisLen, &data[bytesWritten]);

        if (result)
            return result;

        bytesWritten += thisLen;
        reg += thisLen;
    }

    //Enable LP_EN since we disabled it at begining of this function.
    result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    return result;
}

