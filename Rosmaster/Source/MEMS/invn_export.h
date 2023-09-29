//----------------------------------------------------------------------------- 
/*
    Copyright © 2015 InvenSense Inc. All rights reserved.

    This software, related documentation and any modifications thereto (collectively “Software”) is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
    and other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license agreement
    from InvenSense is strictly prohibited.
*/
//-----------------------------------------------------------------------------
#ifndef INVN_EXPORT_H
#define INVN_EXPORT_H

#ifdef _MSC_VER
#  ifdef INVN_EXPORTS
#    define INVN_API __declspec(dllexport)
#  else
#    define INVN_API __declspec(dllimport)
#  endif
#else
#  define INVN_API
#endif

#endif // INVN_EXPORT_H
