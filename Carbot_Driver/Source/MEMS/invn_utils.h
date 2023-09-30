//----------------------------------------------------------------------------- 
/*
    Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively “Software”) is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
    and other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license agreement
    from InvenSense is strictly prohibited.
*/
//-----------------------------------------------------------------------------

#ifndef INVN_COMMON_INVN_UTILS_H_
#define INVN_COMMON_INVN_UTILS_H_

#include <string.h>

#include "invn/common/invn_assert.h"

__inline void * invn_memcpy(void * destination, const void * source, int num);
{
	//non overlap test
	invn_assert (source != 0);
	invn_assert (destination != 0);
	invn_assert ((char*)destination != (char*)source);
	invn_assert (((char*)destination + num <= (char*)source) || ((char*)source + num <= (char*)destination));

	return memcpy (destination, source, num);
}

#endif // INVN_COMMON_INVN_UTILS_H_
