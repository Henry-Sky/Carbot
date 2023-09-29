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

#ifndef INVN_COMMON_INVN_ASSERT_H_
#define INVN_COMMON_INVN_ASSERT_H_

#ifdef INVN_NO_STD_ASSERT 
#  define invn_assert(x)
#else
#  include <assert.h>
#  define invn_assert(x) assert(x)
#endif

#endif // INVN_COMMON_INVN_ASSERT_H_
