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

#ifndef INVN_COMMON_INT_TYPES_H_
#define INVN_COMMON_INT_TYPES_H_

#ifdef _MSC_VER
#  include "inttypes.h"
#else
#  include <stdint.h>
#endif

//  Scalar 32 Signed
typedef int32_t sfix32En0;
typedef int32_t sfix32En1;
typedef int32_t sfix32En2;
typedef int32_t sfix32En3;
typedef int32_t sfix32En4;
typedef int32_t sfix32En5;
typedef int32_t sfix32En6;
typedef int32_t sfix32En7;
typedef int32_t sfix32En8;
typedef int32_t sfix32En9;
typedef int32_t sfix32En10;
typedef int32_t sfix32En11;
typedef int32_t sfix32En12;
typedef int32_t sfix32En13;
typedef int32_t sfix32En14;
typedef int32_t sfix32En15;
typedef int32_t sfix32En16;
typedef int32_t sfix32En17;
typedef int32_t sfix32En18;
typedef int32_t sfix32En19;
typedef int32_t sfix32En20;
typedef int32_t sfix32En21;
typedef int32_t sfix32En22;
typedef int32_t sfix32En23;
typedef int32_t sfix32En24;
typedef int32_t sfix32En25;
typedef int32_t sfix32En26;
typedef int32_t sfix32En27;
typedef int32_t sfix32En28;
typedef int32_t sfix32En29;
typedef int32_t sfix32En30;
typedef int32_t sfix32En31;
typedef int32_t sfix32En32;
typedef int32_t sfix32En33;
typedef int32_t sfix32En34;
typedef int32_t sfix32En35;
typedef int32_t sfix32En36;
typedef int32_t sfix32En37;
typedef int32_t sfix32En38;
typedef int32_t sfix32En39;
typedef int32_t sfix32En40;

//  Scalar 32 Unsigned
typedef uint32_t ufix32En0;
typedef uint32_t ufix32En1;
typedef uint32_t ufix32En2;
typedef uint32_t ufix32En3;
typedef uint32_t ufix32En4;
typedef uint32_t ufix32En5;
typedef uint32_t ufix32En6;
typedef uint32_t ufix32En7;
typedef uint32_t ufix32En8;
typedef uint32_t ufix32En9;
typedef uint32_t ufix32En10;
typedef uint32_t ufix32En11;
typedef uint32_t ufix32En12;
typedef uint32_t ufix32En13;
typedef uint32_t ufix32En14;
typedef uint32_t ufix32En15;
typedef uint32_t ufix32En16;
typedef uint32_t ufix32En17;
typedef uint32_t ufix32En18;
typedef uint32_t ufix32En19;
typedef uint32_t ufix32En20;
typedef uint32_t ufix32En21;
typedef uint32_t ufix32En22;
typedef uint32_t ufix32En23;
typedef uint32_t ufix32En24;
typedef uint32_t ufix32En25;
typedef uint32_t ufix32En26;
typedef uint32_t ufix32En27;
typedef uint32_t ufix32En28;
typedef uint32_t ufix32En29;
typedef uint32_t ufix32En30;
typedef uint32_t ufix32En31;
typedef uint32_t ufix32En32;
typedef uint32_t ufix32En33;
typedef uint32_t ufix32En34;
typedef uint32_t ufix32En35;
typedef uint32_t ufix32En36;
typedef uint32_t ufix32En37;
typedef uint32_t ufix32En38;
typedef uint32_t ufix32En39;
typedef uint32_t ufix32En40;

//  Scalar 16 Signed
typedef int16_t sfix16En11;
typedef int16_t sfix16En10;

//  Scalar 16 Unsigned
typedef uint16_t ufix16En11;
typedef uint16_t ufix16En15;


//! \def INVN_FLT_TO_SFIX
//!	Macro to convert a value from float to sfix32.
#define INVN_FLT_TO_SFIX(value, shift)	( (int32_t)  ((float)(value)*(1ULL << (shift)) + ( (value>=0)-0.5f )) )
//! \def INVN_FLT_TO_UFIX
//!	Macro to convert a value from float to ufix32.
#define INVN_FLT_TO_UFIX(value, shift)	( (uint32_t) ((float)(value)*(1ULL << (shift)) + 0.5f) )
//! \def INVN_DBL_TO_SFIX
//!	Macro to convert a value from double to sfix32.
#define INVN_DBL_TO_SFIX(value, shift)	( (int32_t)  ((double)(value)*(1ULL << (shift)) + ( (value>=0)-0.5 )) )
//! \def INVN_DBL_TO_UFIX
//!	Macro to convert a value from double to ufix32.
#define INVN_DBL_TO_UFIX(value, shift)	( (uint32_t) ((double)(value)*(1ULL << (shift)) + 0.5) )

//! \def INVN_SFIX_TO_FLT
//!	Macro to convert a value from sfix32 to float.
#define INVN_SFIX_TO_FLT(value, shift)	( (float)  (int32_t)(value) / (float)(1ULL << (shift)) )
//! \def INVN_UFIX_TO_FLT
//!	Macro to convert a value from ufix32 to float.
#define INVN_UFIX_TO_FLT(value, shift)	( (float) (uint32_t)(value) / (float)(1ULL << (shift)) )
//! \def INVN_SFIX_TO_DBL
//!	Macro to convert a value from sfix32 to double.
#define INVN_SFIX_TO_DBL(value, shift)	( (double)  (int32_t)(value) / (double)(1ULL << (shift)) )
//! \def INVN_UFIX_TO_DBL
//!	Macro to convert a value from ufix32 to double.
#define INVN_UFIX_TO_DBL(value, shift)	( (double) (uint32_t)(value) / (double)(1ULL << (shift)) )

//! \def INVN_CONVERT_FLT_TO_SFIX
//!	Macro to convert float values from an address into sfix32 values, and copy them to another address.
#define INVN_CONVERT_FLT_TO_SFIX(fltptr, fixptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_FLT_TO_SFIX((fltptr)[i], shift); }
//! \def INVN_CONVERT_FLT_TO_UFIX
//!	Macro to convert float values from an address into ufix32 values, and copy them to another address.
#define INVN_CONVERT_FLT_TO_UFIX(fltptr, fixptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_FLT_TO_UFIX((fltptr)[i], shift); }
//! \def INVN_CONVERT_DBL_TO_SFIX
//!	Macro to convert double values from an address into sfix32 values, and copy them to another address.
#define INVN_CONVERT_DBL_TO_SFIX(fltptr, fixptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_DBL_TO_SFIX((fltptr)[i], shift); }
//! \def INVN_CONVERT_DBL_TO_UFIX
//!	Macro to convert double values from an address into ufix32 values, and copy them to another address.
#define INVN_CONVERT_DBL_TO_UFIX(fltptr, fixptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_DBL_TO_UFIX((fltptr)[i], shift); }
//! \def INVN_CONVERT_SFIX_TO_FLT
//!	Macro to convert sfix32 values from an address into float values, and copy them to another address.
#define INVN_CONVERT_SFIX_TO_FLT(fixptr, fltptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_SFIX_TO_FLT((fixptr)[i], shift); }
//! \def INVN_CONVERT_UFIX_TO_FLT
//!	Macro to convert ufix32 values from an address into float values, and copy them to another address.
#define INVN_CONVERT_UFIX_TO_FLT(fixptr, fltptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_UFIX_TO_FLT((fixptr)[i], shift); }
//! \def INVN_CONVERT_SFIX_TO_DBL
//!	Macro to convert sfix32 values from an address into double values, and copy them to another address.
#define INVN_CONVERT_SFIX_TO_DBL(fixptr, fltptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_SFIX_TO_DBL((fixptr)[i], shift); }
//! \def INVN_CONVERT_UFIX_TO_DBL
//!	Macro to convert ufix32 values from an address into double values, and copy them to another address.
#define INVN_CONVERT_UFIX_TO_DBL(fixptr, fltptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_UFIX_TO_DBL((fixptr)[i], shift); }


#endif // INVN_COMMON_INT_TYPES_H_
