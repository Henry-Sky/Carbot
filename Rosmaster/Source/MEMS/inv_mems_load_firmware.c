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

#include "inv_mems_load_firmware.h"
#include "inv_mems_hw_config.h"
#include "inv_mems_defines.h"
#ifndef MEMS_20609
    #include "inv_mems_base_driver.h"
#else
    #include "inv_mems_base_driver_20609.h"
#endif
#include "inv_mems_transport.h"
#include "inv_mems_interface_mapping.h"

#include "invn_types.h"


inv_error_t inv_mems_firmware_load(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{
    int write_size;
    int result;
    unsigned short memaddr;
    const unsigned char *data;
    unsigned short size;
    #if (MEMS_CHIP == HW_ICM20648 || MEMS_CHIP == HW_ICM20609)
    unsigned char data_cmp[INV_MAX_SERIAL_READ + 1];
    #else
    unsigned char data_cmp[INV_MAX_SERIAL_READ];
    #endif
    int flag = 0;
    memset (data_cmp, 0, (INV_MAX_SERIAL_READ + 1));
    // Write DMP memory
    data = data_start;
    size = size_start;
    memaddr = load_addr;

    while (size > 0)
    {
        write_size = min(size, INV_MAX_SERIAL_WRITE);

        if ((memaddr & 0xff) + write_size > 0x100)
        {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }

        result = inv_write_mems(memaddr, write_size, (unsigned char *)data);

        if (result)
            return result;

        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }

    //memcmp(data_cmp, data, write_size);
    // Verify DMP memory

    data = data_start;
    size = size_start;
    memaddr = load_addr;

    while (size > 0)
    {
        write_size = min(size, INV_MAX_SERIAL_READ);

        if ((memaddr & 0xff) + write_size > 0x100)
        {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }

        result = inv_read_mems(memaddr, write_size, data_cmp);

        if (result)
            flag++; // Error, DMP not written correctly

        if (memcmp(data_cmp, data, write_size))
            return -1;

        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }


    #if defined(WIN32)

    if(!flag)
        inv_log("DMP Firmware was updated successfully..\r\n");

    #endif

    return INV_SUCCESS;
}

