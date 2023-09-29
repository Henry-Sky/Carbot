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

#ifndef INVN_COMMON_INVN_ENUMS_H_
#define INVN_COMMON_INVN_ENUMS_H_

/** \defgroup InvnEnums Sensor Axis
	\brief This file contains definitions of sensor axis

	\ingroup invn_types
 */

/** \brief Sensor axis definitions.

    \ingroup SensorAxis
	\sa \ref Conventions
*/
typedef enum {
	INVN_SENSOR_AXIS_PLUS_X 	= 1,		/**< X axis */
	INVN_SENSOR_AXIS_MINUS_X 	= -1,		/**< -X axis */
	INVN_SENSOR_AXIS_PLUS_Y 	= 2,		/**< Y axis */
	INVN_SENSOR_AXIS_MINUS_Y 	= -2,		/**< -Y axis */
	INVN_SENSOR_AXIS_PLUS_Z 	= 3,		/**< Z axis */
	INVN_SENSOR_AXIS_MINUS_Z 	= -3,		/**< -Z axis */
	INVN_SENSOR_AXIS_UNKNOWN 	= 0,		/**< unknown axis */
} InvnSensorAxis;

#endif // INVN_COMMON_INVN_ENUMS_H_
