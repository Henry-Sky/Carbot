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
  
#ifndef _INV_MEMS_HW_CONFIG_H__
#define _INV_MEMS_HW_CONFIG_H__

// Ivory chip list
#define HW_ICM20630 0x10
#define HW_ICM20610 0x11
#define HW_ICM30630 0x12
#define HW_ICM20645_E 0x13	//ICM-20645 for embedded projects
#define HW_ICM20648 0x14
#define HW_ICM20609 0x15

// compass chip list
#define HW_AK8963 0x20
#define HW_AK8975 0x21
#define HW_AK8972 0x22
#define HW_AK09911 0x23
#define HW_AK09912 0x24
#define HW_AK09916 0x25

#if defined MEMS_20630
#define MEMS_CHIP HW_ICM20630
#elif defined MEMS_30630
#define MEMS_CHIP HW_ICM30630
#elif defined MEMS_20645E
#define MEMS_CHIP HW_ICM20645_E
#elif defined MEMS_20648
#define MEMS_CHIP HW_ICM20648
#elif defined MEMS_20609
#define MEMS_CHIP HW_ICM20609
#endif

extern const unsigned char ACCEL_GYRO_CHIP_ADDR;
//#if defined MEMS_SECONDARY_DEVICE
extern const unsigned char COMPASS_SLAVE_ID;
extern const unsigned char COMPASS_CHIP_ADDR;
extern const unsigned char PRESSURE_CHIP_ADDR;
//#endif
extern signed char ACCEL_GYRO_ORIENTATION[9];
extern signed char COMPASS_ORIENTATION[9];
extern long SOFT_IRON_MATRIX[9];

// _INV_MEMS_HW_CONFIG_H__
#endif
