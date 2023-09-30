/*
* ________________________________________________________________________________________________________
* Copyright © 2011-2015 InvenSense Inc. Portions Copyright © 2011-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/
/** @defgroup mems_mltypes mltypes
	@ingroup  Mems_common
	@{
*/
#ifndef MLTYPES_H
#define MLTYPES_H


# include "inv_mems_defines.h"

#ifndef REMOVE_INV_ERROR_T
    /*---------------------------
    *    ML Types
    *--------------------------*/

    /**
    *  @struct inv_error_t mltypes.h "mltypes"
    *  @brief  The MPL Error Code return type.
    *
    *  @code
    *      typedef unsigned char inv_error_t;
    *  @endcode
    */
    //typedef unsigned char inv_error_t;
    typedef int inv_error_t;
#endif

#endif				/* MLTYPES_H */

/** @} */

