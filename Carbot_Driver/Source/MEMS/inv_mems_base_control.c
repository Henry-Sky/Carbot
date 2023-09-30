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

#ifndef MEMS_20609

#include "inv_mems_base_control.h"

#include "mlmath.h"
#include "inv_mems_interface_mapping.h"

#if defined MEMS_SECONDARY_DEVICE
    #include "inv_mems_slave_compass.h"
    #include "inv_mems_slave_pressure.h"
    #include "inv_mems_secondary_transport.h"
#endif

#if defined MEMS_AUGMENTED_SENSORS
    #include "inv_mems_augmented_sensors.h"
#endif

#include "invn_types.h"

//#define INV_SENSOR_MULTIFIFO
#define INV_ODR_MIN_DELAY   200     // Limited by 8-bit HW Gyro rate divider register "GYRO_SMPLRT_DIV"

static inv_error_t inv_enable_sensor_internal(unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep);
#if defined MEMS_SECONDARY_DEVICE
    static unsigned char sensor_needs_compass(unsigned char androidSensor);
    static unsigned char sensor_needs_pressure(unsigned char androidSensor);
#endif

static unsigned short inv_dmp_odr_dividers[INV_SENSOR_NUM_MAX] = {0};	// Actual ODRs in dividers from 1125Hz, in orders of INV_SENSORS, inited as unset
static unsigned short inv_dmp_odr_delays[INV_SENSOR_NUM_MAX] =          // Desired ODRs in milliseconds, in orders of INV_SENSORS, inited defaults as 200ms (5Hz)
{
    INV_ODR_MIN_DELAY,      // INV_SENSOR_ACCEL
    INV_ODR_MIN_DELAY,      // INV_SENSOR_GYRO,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_LPQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_COMPASS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_ALS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_SIXQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_NINEQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_GEOMAG,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_PEDQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_PRESSURE,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_CALIB_GYRO,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_CALIB_COMPASS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_STEP_COUNTER,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_ACTIVITY_CLASSIFIER,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_FLIP_PICKUP,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_BRING_TO_SEE,

    INV_ODR_MIN_DELAY,      // INV_SENSOR_SIXQ_accel,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_NINEQ_accel,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_GEOMAG_cpass,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_NINEQ_cpass,

    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_ACCEL,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_GYRO,
    //INV_ODR_MIN_DELAY,    // INV_SENSOR_WAKEUP_LPQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_COMPASS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_ALS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_SIXQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_NINEQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_GEOMAG,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_PEDQ,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_PRESSURE,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_CALIB_GYRO,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_CALIB_COMPASS,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_STEP_COUNTER,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_TILT_DETECTOR,
    //INV_ODR_MIN_DELAY,    // INV_SENSOR_WAKEUP_ACTIVITY_CLASSIFIER,

    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_SIXQ_accel,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_NINEQ_accel,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_GEOMAG_cpass,
    INV_ODR_MIN_DELAY,      // INV_SENSOR_WAKEUP_NINEQ_cpass,
};

#if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)
    static unsigned short bac_on = 0; // indicates if ANDROID_SENSOR_ACTIVITY_CLASSIFICATON is on
    static unsigned short bac_status = 0;
#endif
static unsigned short b2s_status = 0;
static unsigned short flip_pickup_status = 0;
static unsigned short inv_sensor_control = 0;
static unsigned short inv_sensor_control2 = 0;
static unsigned long inv_androidSensorsOn_mask[2] = {0}; // Each bit corresponds to a sensor being on

#if (MEMS_CHIP == HW_ICM30630)
    static unsigned char sGmrvIsOn = 0; // indicates if GMRV was requested to be ON by end-user. Once this variable is set, it is either GRV or GMRV which is enabled internally
#endif

const short inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX] =
{
    // Unsupported Sensors are -1
    -1, // Meta Data
        -32760, //0x8008, // Accelerometer
        0x0028, // Magnetic Field
        0x0408, // Orientation
        #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
        0x4048, // Gyroscope
        #else
        0x0048, // Gyroscope
        #endif
        0x1008, // Light
        0x0088, // Pressure
        -1, // Temperature
        -1, // Proximity <----------- fixme
        0x0808, // Gravity
        -30712, // 0x8808, // Linear Acceleration
        0x0408, // Rotation Vector
        -1, // Humidity
        -1, // Ambient Temperature
        0x2008, // Magnetic Field Uncalibrated
        0x0808, // Game Rotation Vector
        0x4008, // Gyroscope Uncalibrated
        0, // Significant Motion
        0x0018, // Step Detector
        0x0010, // Step Counter <----------- fixme
        #if (MEMS_CHIP == HW_ICM20648)
        0x0108, // Geomagnetic Rotation Vector
        #else
        0x0408, // Geomagnetic Rotation Vector
        #endif
        -1, //ANDROID_SENSOR_HEART_RATE,
        -1, //ANDROID_SENSOR_PROXIMITY,

        -32760, // ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
        0x0028, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
        0x0408, // ANDROID_SENSOR_WAKEUP_ORIENTATION,
        #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
        0x4048, // ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        #else
        0x0048, // ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        #endif
        0x1008, // ANDROID_SENSOR_WAKEUP_LIGHT,
        0x0088, // ANDROID_SENSOR_WAKEUP_PRESSURE,
        0x0808, // ANDROID_SENSOR_WAKEUP_GRAVITY,
        -30712, // ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
        0x0408, // ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
        -1,		// ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
        -1,		// ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
        0x2008, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
        0x0808, // ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
        0x4008, // ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
        0x0018, // ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
        0x0010, // ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
        #if (MEMS_CHIP == HW_ICM20648)
        0x0108, // ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
        #else
        0x0408, // ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
        #endif
        -1,		// ANDROID_SENSOR_WAKEUP_HEART_RATE,
        0,		// ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
    };

unsigned long inv_androidSensor_enabled(unsigned char androidSensor)
{
    return inv_androidSensorsOn_mask[(androidSensor >> 5)] & (1L << (androidSensor & 0x1F));
}


/******************/
typedef	struct
{
    enum	ANDROID_SENSORS	AndroidSensor;
    enum	INV_SENSORS		InvSensor;
}	MinDelayGenElementT;

/******************/
//	accel
static	const	MinDelayGenElementT	MinDelayGenAccelList	[]	=
{
    {	ANDROID_SENSOR_ACCELEROMETER,						INV_SENSOR_ACCEL				}
    ,	{	ANDROID_SENSOR_WAKEUP_ACCELEROMETER,				INV_SENSOR_WAKEUP_ACCEL			}
    ,	{	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,			INV_SENSOR_GEOMAG				}
    ,	{	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,	INV_SENSOR_WAKEUP_GEOMAG		}
    #if (MEMS_CHIP != HW_ICM30630)
    ,	{	ANDROID_SENSOR_STEP_DETECTOR,						INV_SENSOR_STEP_COUNTER			}
    ,	{	ANDROID_SENSOR_STEP_COUNTER,						INV_SENSOR_STEP_COUNTER			}
    ,	{	ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,				INV_SENSOR_WAKEUP_STEP_COUNTER	}
    ,	{	ANDROID_SENSOR_WAKEUP_STEP_COUNTER,					INV_SENSOR_WAKEUP_STEP_COUNTER	}
    ,	{	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,			INV_SENSOR_WAKEUP_STEP_COUNTER	}
    ,	{	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,				INV_SENSOR_WAKEUP_TILT_DETECTOR	}
    #endif
    ,	{	ANDROID_SENSOR_GRAVITY,								INV_SENSOR_SIXQ_accel			}
    ,	{	ANDROID_SENSOR_GAME_ROTATION_VECTOR,				INV_SENSOR_SIXQ_accel			}
    ,	{	ANDROID_SENSOR_LINEAR_ACCELERATION,					INV_SENSOR_SIXQ_accel			}
    ,	{	ANDROID_SENSOR_WAKEUP_GRAVITY,						INV_SENSOR_WAKEUP_SIXQ_accel	}
    ,	{	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,			INV_SENSOR_WAKEUP_SIXQ_accel	}
    ,	{	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,			INV_SENSOR_WAKEUP_SIXQ_accel	}
    ,	{	ANDROID_SENSOR_ORIENTATION,							INV_SENSOR_NINEQ_accel			}
    ,	{	ANDROID_SENSOR_ROTATION_VECTOR,						INV_SENSOR_NINEQ_accel			}
    ,	{	ANDROID_SENSOR_WAKEUP_ORIENTATION,					INV_SENSOR_WAKEUP_NINEQ_accel	}
    ,	{	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,				INV_SENSOR_WAKEUP_NINEQ_accel	}
};

//	gyro
static	const	MinDelayGenElementT	MinDelayGenGyroList	[]	=
{
    {	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,			INV_SENSOR_GYRO					}
    ,	{	ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,	INV_SENSOR_WAKEUP_GYRO			}
    ,	{	ANDROID_SENSOR_GYROSCOPE,						INV_SENSOR_CALIB_GYRO			}
    ,	{	ANDROID_SENSOR_WAKEUP_GYROSCOPE,				INV_SENSOR_WAKEUP_CALIB_GYRO	}
    ,	{	ANDROID_SENSOR_GRAVITY,							INV_SENSOR_SIXQ					}
    ,	{	ANDROID_SENSOR_GAME_ROTATION_VECTOR,			INV_SENSOR_SIXQ					}
    ,	{	ANDROID_SENSOR_LINEAR_ACCELERATION,				INV_SENSOR_SIXQ					}
    ,	{	ANDROID_SENSOR_WAKEUP_GRAVITY,					INV_SENSOR_WAKEUP_SIXQ			}
    ,	{	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,		INV_SENSOR_WAKEUP_SIXQ			}
    ,	{	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,		INV_SENSOR_WAKEUP_SIXQ			}
    ,	{	ANDROID_SENSOR_ORIENTATION,						INV_SENSOR_NINEQ				}
    ,	{	ANDROID_SENSOR_ROTATION_VECTOR,					INV_SENSOR_NINEQ				}
    ,	{	ANDROID_SENSOR_WAKEUP_ORIENTATION,				INV_SENSOR_WAKEUP_NINEQ			}
    ,	{	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,			INV_SENSOR_WAKEUP_NINEQ			}
};

#if (MEMS_CHIP == HW_ICM30630)
static	const	MinDelayGenElementT	MinDelayGenGyroForGmrvList	[]	=
    // special case for GMRV : it will enable GRV instead of GMRV if GMRV needs to be enabled in parallel to GRV
{
    {	ANDROID_SENSOR_GAME_ROTATION_VECTOR,			INV_SENSOR_GEOMAG				}
    ,	{	ANDROID_SENSOR_GAME_ROTATION_VECTOR,	        INV_SENSOR_WAKEUP_GEOMAG		}
};
#endif

#if defined MEMS_SECONDARY_DEVICE
//	Cpass
static	const	MinDelayGenElementT	MinDelayGenCpassList	[]	=
{
    {	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,			INV_SENSOR_COMPASS				}
    ,	{	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,	INV_SENSOR_WAKEUP_COMPASS		}
    ,	{	ANDROID_SENSOR_GEOMAGNETIC_FIELD,					INV_SENSOR_CALIB_COMPASS		}
    ,	{	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,				INV_SENSOR_WAKEUP_CALIB_COMPASS	}
    ,	{	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,			INV_SENSOR_GEOMAG_cpass			}
    ,	{	ANDROID_SENSOR_ORIENTATION,							INV_SENSOR_NINEQ_cpass			}
    ,	{	ANDROID_SENSOR_ROTATION_VECTOR,						INV_SENSOR_NINEQ_cpass			}
    ,	{	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,	INV_SENSOR_WAKEUP_GEOMAG_cpass	}
    ,	{	ANDROID_SENSOR_WAKEUP_ORIENTATION,					INV_SENSOR_WAKEUP_NINEQ_cpass	}
    ,	{	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,				INV_SENSOR_WAKEUP_NINEQ_cpass	}
};

//	Pressure
static	const	MinDelayGenElementT	MinDelayGenPressureList	[]	=
{
    {	ANDROID_SENSOR_PRESSURE,							INV_SENSOR_PRESSURE			}
    ,	{	ANDROID_SENSOR_WAKEUP_PRESSURE,						INV_SENSOR_WAKEUP_PRESSURE	}
};
#endif
/*************************/

static	const	MinDelayGenElementT	MinDelayGenAccel2List	[]	=
{
    {	ANDROID_SENSOR_ACCELEROMETER,						INV_SENSOR_ACCEL				}
    ,	{	ANDROID_SENSOR_WAKEUP_ACCELEROMETER,				INV_SENSOR_WAKEUP_ACCEL			}
    ,	{	ANDROID_SENSOR_LINEAR_ACCELERATION,					INV_SENSOR_SIXQ_accel			}
    ,	{	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,			INV_SENSOR_WAKEUP_SIXQ_accel	}
};

/**********/
static	const	MinDelayGenElementT	MinDelayGenAccel3List	[]	=
{
    {	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,			INV_SENSOR_GEOMAG			}
    ,	{	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,	INV_SENSOR_WAKEUP_GEOMAG	}
};

/**********/
#if (MEMS_CHIP != HW_ICM30630)
static	const	MinDelayGenElementT	MinDelayGenAccel4List	[]	=
{
    {	ANDROID_SENSOR_STEP_DETECTOR,						INV_SENSOR_STEP_COUNTER			}
    ,	{	ANDROID_SENSOR_STEP_COUNTER,						INV_SENSOR_STEP_COUNTER			}
    ,	{	ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,				INV_SENSOR_WAKEUP_STEP_COUNTER	}
    ,	{	ANDROID_SENSOR_WAKEUP_STEP_COUNTER,					INV_SENSOR_WAKEUP_STEP_COUNTER	}
    ,	{	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,			INV_SENSOR_WAKEUP_STEP_COUNTER	}
};
#endif

#if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)
/**********/
static	const	MinDelayGenElementT	MinDelayGenAccel5List	[]	=
{
    {	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,				INV_SENSOR_WAKEUP_TILT_DETECTOR	}
};
#endif

/**********//**********/
static	const	MinDelayGenElementT	MinDelayGenGyro2List	[]	=
{
    {	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,				INV_SENSOR_GYRO			}
    ,	{	ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,		INV_SENSOR_WAKEUP_GYRO	}
    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20648)
    ,   {	ANDROID_SENSOR_GYROSCOPE,							INV_SENSOR_CALIB_GYRO			}
    ,   {	ANDROID_SENSOR_WAKEUP_GYROSCOPE,					INV_SENSOR_WAKEUP_CALIB_GYRO	}
    #endif
};

/**********/
static	const	MinDelayGenElementT	MinDelayGenGyro3List	[]	=
{
    {	ANDROID_SENSOR_GYROSCOPE,							INV_SENSOR_CALIB_GYRO			}
    ,	{	ANDROID_SENSOR_WAKEUP_GYROSCOPE,					INV_SENSOR_WAKEUP_CALIB_GYRO	}
};

/**********/
static	const	MinDelayGenElementT	MinDelayGenGyro4List	[]	=
{
    {	ANDROID_SENSOR_GRAVITY,								INV_SENSOR_SIXQ			}
    ,	{	ANDROID_SENSOR_GAME_ROTATION_VECTOR,				INV_SENSOR_SIXQ			}
    ,	{	ANDROID_SENSOR_LINEAR_ACCELERATION,					INV_SENSOR_SIXQ			}
    ,	{	ANDROID_SENSOR_WAKEUP_GRAVITY,						INV_SENSOR_WAKEUP_SIXQ	}
    ,	{	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,			INV_SENSOR_WAKEUP_SIXQ	}
    ,	{	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,			INV_SENSOR_WAKEUP_SIXQ	}
};

#if (MEMS_CHIP == HW_ICM30630)
static	const	MinDelayGenElementT	MinDelayGenGyro4ForGmrvList	[]	=
    // special case for GMRV : it will enable GRV instead of GMRV if GMRV needs to be enabled in parallel to GRV or GMRV
{
    {	ANDROID_SENSOR_GAME_ROTATION_VECTOR,			    INV_SENSOR_GEOMAG				}
    ,	{	ANDROID_SENSOR_GAME_ROTATION_VECTOR,	            INV_SENSOR_WAKEUP_GEOMAG		}
};
#endif

/**********/
static	const	MinDelayGenElementT	MinDelayGenGyro5List	[]	=
{
    {	ANDROID_SENSOR_ORIENTATION,				INV_SENSOR_NINEQ		}
    ,	{	ANDROID_SENSOR_ROTATION_VECTOR,			INV_SENSOR_NINEQ		}
    ,	{	ANDROID_SENSOR_WAKEUP_ORIENTATION,		INV_SENSOR_WAKEUP_NINEQ	}
    ,	{	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,	INV_SENSOR_WAKEUP_NINEQ	}
};

#if defined MEMS_SECONDARY_DEVICE
/**********//**********/
static	const	MinDelayGenElementT	MinDelayGenCpass2List	[]	=
{
    {	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,			INV_SENSOR_COMPASS			}
    ,	{	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,	INV_SENSOR_WAKEUP_COMPASS	}
};

/**********/
static	const	MinDelayGenElementT	MinDelayGenCpass3List	[]	=
{
    {	ANDROID_SENSOR_GEOMAGNETIC_FIELD,					INV_SENSOR_CALIB_COMPASS		}
    ,	{	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,				INV_SENSOR_WAKEUP_CALIB_COMPASS	}
};

/**********//**********/
static	const	MinDelayGenElementT	MinDelayGenPressure2List	[]	=
{
    {	ANDROID_SENSOR_PRESSURE,						INV_SENSOR_PRESSURE			}
    ,	{	ANDROID_SENSOR_WAKEUP_PRESSURE,					INV_SENSOR_WAKEUP_PRESSURE	}
};
#endif
/**********/

typedef	unsigned	short	MinDelayT;
typedef	unsigned	short	u16;
typedef	unsigned	long	u32;

/******************/
#define	MinDelayGen(list)	MinDelayGenActual((list), sizeof((list)) / sizeof (MinDelayGenElementT))

static		MinDelayT	MinDelayGenActual
(
    const	MinDelayGenElementT	*	Element,
    u32						ElementQuan
)
{
    MinDelayT	MinDelay	=	(MinDelayT) - 1;

    while	(ElementQuan--)
    {
        if	(inv_androidSensor_enabled	(Element->AndroidSensor))
        {
            MinDelayT	OdrDelay	=	inv_dmp_odr_delays	[Element->InvSensor];

            if	(MinDelay >	OdrDelay)
            {
                MinDelay	=	OdrDelay;
            }
        }

        Element++;
    }	//	end while elements to process

    return	MinDelay;
}	//	end of MinDelayGenActual()

/******************/
static	inv_error_t	DividerRateSet
(
    MinDelayT			MinDelay,
    u16					HwSampleRateDivider,
    enum	INV_SENSORS	InvSensor
)
{
    inv_error_t	Result	=	0;

    if	(MinDelay != 0xFFFF)
    {
        u16	DmpOdrDivider	=	(MinDelay * 1125L) / (HwSampleRateDivider * 1000L);      // a divider from (1125Hz/hw_smplrt_divider).

        inv_dmp_odr_dividers[InvSensor] = HwSampleRateDivider * DmpOdrDivider;
        Result |= dmp_set_sensor_rate(InvSensor, (DmpOdrDivider - 1));
    }

    return	Result;
}	//	end of DividerRateGet()

/******************/
static	u16	SampleRateDividerGet	(MinDelayT	MinDelay)
{
    u16	Delay	=	min	(INV_ODR_MIN_DELAY, MinDelay); // because of GYRO_SMPLRT_DIV which relies on 8 bits, we can't have ODR value higher than 200ms
    return			Delay * 1125L / 1000L;      // a divider from 1125Hz.
}	//	end of SampleRateDividerGet()



/** @brief Get minimum ODR to be applied to accel engine based on all accel-based enabled sensors.
* @return ODR in ms we expect to be applied to accel engine
*/
static unsigned short getMinDlyAccel(void)
{
    unsigned short lMinOdr	=	MinDelayGen	(MinDelayGenAccelList);

    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if	(bac_status != 0)			lMinOdr	=	min	(	lMinOdr, inv_dmp_odr_delays	[INV_SENSOR_ACTIVITY_CLASSIFIER]	);

    #endif

    if	(flip_pickup_status != 0)	lMinOdr	=	min	(	lMinOdr, inv_dmp_odr_delays	[INV_SENSOR_FLIP_PICKUP]			);

    if	(b2s_status != 0)           lMinOdr	=	min	(	lMinOdr, inv_dmp_odr_delays	[INV_SENSOR_BRING_TO_SEE]			);

    #if (MEMS_CHIP == HW_ICM20648)

    /** To have correct algorithm performance and quick convergence of GMRV, it is advised to set accelerometer to 225Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 5	 );

    /** To have correct algorithm performance and quick convergence of RV, it is advised to set accelerometer to 225Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 5	 );

    /** To have correct algorithm performance and quick convergence of GRV, it is advised to set accelerometer to 225Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_GAME_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 5	 );

    #endif

    return lMinOdr;
}

/** @brief Get minimum ODR to be applied to gyro engine based on all gyro-based enabled sensors.
* @return ODR in ms we expect to be applied to gyro engine
*/
static unsigned short getMinDlyGyro(void)
{
    unsigned short lMinOdr	=	MinDelayGen	(MinDelayGenGyroList);
    #if (MEMS_CHIP == HW_ICM30630)
    /** Min ODR to be applied only for GMRV case */
    unsigned short lMinOdrForGmrv	=	MinDelayGen	(MinDelayGenGyroForGmrvList);
    #endif

    #if (MEMS_CHIP == HW_ICM20648)

    /** To have correct algorithm performance and quick convergence of RV, it is advised to set gyro to 225Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 5	 );

    #endif

    #if (MEMS_CHIP == HW_ICM30630)

    /** In case GMRV is ON, need to check for GMRV related ODR be applied to GRV too, since GRV will be internally enabled instead of GMRV itself */
    if(sGmrvIsOn)
        lMinOdr = min(lMinOdr, lMinOdrForGmrv);

    #endif

    return lMinOdr;
}

#if defined MEMS_SECONDARY_DEVICE
/** @brief Get minimum ODR to be applied to compass engine based on all compass-based enabled sensors.
* @return ODR in ms we expect to be applied to compass engine
*/
static unsigned short getMinDlyCompass(void)
{
    unsigned short lMinOdr	=	MinDelayGen	(MinDelayGenCpassList);

    #if (MEMS_CHIP == HW_ICM20648)

    /** To have correct algorithm performance and quick convergence of GMRV, it is advised to set compass to 70Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 15 );

    /** To have correct algorithm performance and quick convergence of RV, it is advised to set compass to 35Hz.
        In case power consumption is to be improved at the expense of performance, this setup should be commented out */
    if ( inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) || inv_androidSensor_enabled(ANDROID_SENSOR_ROTATION_VECTOR) )
        lMinOdr	=	min	(	lMinOdr, 28 );

    #endif

    return lMinOdr;
}
#endif

/******************/
inv_error_t inv_set_hw_smplrt_dmp_odrs()
{
    inv_error_t result = 0;

    unsigned short minDly, minDly_accel, minDly_gyro;
    #if defined MEMS_SECONDARY_DEVICE
    unsigned short minDly_cpass;
    unsigned short minDly_pressure;
    #endif
    unsigned short hw_smplrt_divider = 0;

// get min delays of all enabled sensors for each sensor engine group

    // Engine ACCEL Based
    minDly_accel = getMinDlyAccel();

    // Engine Gyro Based
    minDly_gyro  = getMinDlyGyro();

    #if defined MEMS_SECONDARY_DEVICE
    // Engine Cpass Based
    minDly_cpass = getMinDlyCompass();

    // Engine Pressure Based
    minDly_pressure	=	MinDelayGen	(MinDelayGenPressureList);
    #endif

    #ifndef INV_SENSOR_MULTIFIFO
    // get min delay of all enabled sensors of all sensor engine groups
    minDly = min(minDly_gyro, minDly_accel);
    #if defined MEMS_SECONDARY_DEVICE
    minDly = min(minDly, minDly_cpass);
    minDly = min(minDly, minDly_pressure);
    #endif

    if (minDly_accel != 0xFFFF)	        minDly_accel = minDly;

    if (minDly_gyro  != 0xFFFF)	        minDly_gyro  = minDly;

    #if defined MEMS_SECONDARY_DEVICE

    if (minDly_cpass != 0xFFFF)		minDly_cpass = minDly;

    if (minDly_pressure != 0xFFFF)	minDly_pressure = minDly;

    #endif
    #endif

// set odrs for each enabled sensors

    // Engine ACCEL Based
    if (minDly_accel != 0xFFFF)	// 0xFFFF -- none accel based sensor enable
    {
        static unsigned short lLastHwSmplrtDivider = 1;

        hw_smplrt_divider	=	SampleRateDividerGet	(minDly_accel);

        if (hw_smplrt_divider != lLastHwSmplrtDivider)
        {
            result	|=	inv_set_accel_quaternion_gain	(hw_smplrt_divider);
            result	|=	inv_set_accel_cal_params		(hw_smplrt_divider);
            result	|=	inv_set_accel_divider			(hw_smplrt_divider - 1);
            lLastHwSmplrtDivider = hw_smplrt_divider;
        }

        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenAccel2List), hw_smplrt_divider, INV_SENSOR_ACCEL);
        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenAccel3List), hw_smplrt_divider, INV_SENSOR_GEOMAG);

        #if (MEMS_CHIP != HW_ICM30630)
        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenAccel4List), hw_smplrt_divider, INV_SENSOR_STEP_COUNTER);
        #endif
        #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )
        {
            unsigned short lStepCounterMinDly =	MinDelayGen		(MinDelayGenAccel5List);

            if	(bac_status != 0)				lStepCounterMinDly = min(lStepCounterMinDly, inv_dmp_odr_delays[INV_SENSOR_ACTIVITY_CLASSIFIER]);

//			if (flip_pickup_status != 0)		lStepCounterMinDly = min(lStepCounterMinDly, inv_dmp_odr_delays[INV_SENSOR_FLIP_PICKUP])
            result			|=	DividerRateSet	(lStepCounterMinDly, hw_smplrt_divider, INV_SENSOR_STEP_COUNTER);
        }
        #endif
    }

    // Engine Gyro Based
    if (minDly_gyro != 0xFFFF)   // 0xFFFF -- none gyro based sensor enable
    {
        static unsigned short lLastHwSmplrtDivider = 1;
        hw_smplrt_divider	=	SampleRateDividerGet	(minDly_gyro);

        if (hw_smplrt_divider != lLastHwSmplrtDivider)
        {
            result	|=	inv_set_gyro_divider	((unsigned char)(hw_smplrt_divider - 1));
            lLastHwSmplrtDivider = hw_smplrt_divider;
        }

        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenGyro2List), hw_smplrt_divider, INV_SENSOR_GYRO);
        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenGyro3List), hw_smplrt_divider, INV_SENSOR_CALIB_GYRO);
        #if (MEMS_CHIP == HW_ICM30630)
        {
            unsigned short lMinOdrForGmrv;

            /** In case GMRV is ON, need to check for GMRV related ODR be applied to GRV too, since GRV will be internally enabled instead of GMRV itself */
            if (sGmrvIsOn)
                lMinOdrForGmrv = MinDelayGen	(MinDelayGenGyro4ForGmrvList);
            else
                lMinOdrForGmrv = 0xFFFF;

            result			|=	DividerRateSet	(min(lMinOdrForGmrv, MinDelayGen		(MinDelayGenGyro4List)), hw_smplrt_divider, INV_SENSOR_SIXQ);
        }
        #else
        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenGyro4List), hw_smplrt_divider, INV_SENSOR_SIXQ);
        #endif
        result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenGyro5List), hw_smplrt_divider, INV_SENSOR_NINEQ);
    }

    #if defined MEMS_SECONDARY_DEVICE

    // Engine Cpass and Pressure Based
    if ((minDly_cpass != 0xFFFF) || (minDly_pressure != 0xFFFF))
    {
        unsigned int lI2cEffectiveDivider = 0;

        // if compass or pressure are alone, compute 1st stage divider, otherwise it will be taken from accel or gyro
        if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
            hw_smplrt_divider	=	SampleRateDividerGet	(minDly);

        // Apply compass or pressure ODR to I2C and get effective ODR
        // so that 2nd level of divider can take into account real frequency we can expect
        // to determine its divider value
        result	|=	inv_mems_secondary_set_odr	(hw_smplrt_divider, &lI2cEffectiveDivider);

        // if compass or pressure are alone, recompute 1st stage divider based on configured divider for I2C
        // otherwise divider is taken from accel or gyro, so there is no need to recompute effective divider value
        // based on the divider we just applied
        if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
            hw_smplrt_divider = lI2cEffectiveDivider;

        if (minDly_cpass != 0xFFFF)
        {
            result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenCpass2List), hw_smplrt_divider, INV_SENSOR_COMPASS);
            result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenCpass3List), hw_smplrt_divider, INV_SENSOR_CALIB_COMPASS);
        }

        if (minDly_pressure != 0xFFFF)
            result			|=	DividerRateSet	(MinDelayGen		(MinDelayGenPressure2List), hw_smplrt_divider, INV_SENSOR_PRESSURE);
    }

    #endif
    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )
    result |= dmp_set_bac_rate(inv_get_accel_divider());
    #endif
    return result;
}

inv_error_t inv_set_odr(unsigned char androidSensor, unsigned short delayInMs)
{
    inv_error_t result;
    uint32_t fixed_ms;

    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )
    fixed_ms = 1000 / PED_BAC_RATE_HZ; // minimum rate for BAC

    if (1000 % PED_BAC_RATE_HZ)
        fixed_ms += 1;

    #else
    fixed_ms = delayInMs;
    #endif

    #if defined MEMS_SECONDARY_DEVICE

    if(sensor_needs_compass(androidSensor))
        if(!inv_mems_get_compass_availability())
            return INV_ERROR_INVALID_PARAMETER;

    if(sensor_needs_pressure(androidSensor))
        if(!inv_mems_get_pressure_availability())
            return INV_ERROR_INVALID_PARAMETER;

    #endif

    inv_mems_prevent_lpen_control();

    if (delayInMs == 0) delayInMs = 1;

    if (delayInMs > 1000) delayInMs = 1000;     // 1Hz / 1000ms

    switch (androidSensor)
    {
        case ANDROID_SENSOR_ACCELEROMETER:
            inv_dmp_odr_delays[INV_SENSOR_ACCEL] = delayInMs;
            break;

            #if (MEMS_CHIP != HW_ICM30630)

        case ANDROID_SENSOR_STEP_DETECTOR:
        case ANDROID_SENSOR_STEP_COUNTER:
            inv_dmp_odr_delays[INV_SENSOR_STEP_COUNTER] = fixed_ms;
            break;
            #endif

        case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            inv_dmp_odr_delays[INV_SENSOR_GEOMAG] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_GEOMAG_cpass] = delayInMs;
            break;

            #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

        case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
            inv_dmp_odr_delays[INV_SENSOR_ACTIVITY_CLASSIFIER] = fixed_ms;
            break;
            #endif

        case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
            inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
            break;

        case ANDROID_SENSOR_GYROSCOPE:
            #if (MEMS_CHIP == HW_ICM30630)
            inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
            #endif
            inv_dmp_odr_delays[INV_SENSOR_CALIB_GYRO] = delayInMs;
            break;

        case ANDROID_SENSOR_GRAVITY:
        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
            #if defined MEMS_AUGMENTED_SENSORS
            // if augmented sensors are handled by this driver,
            // then the fastest 6quat-based sensor which is enabled
            // should be applied to all 6quat-based sensors
            delayInMs = inv_mems_augmented_sensors_set_odr(androidSensor, delayInMs);
            #endif
            inv_dmp_odr_delays[INV_SENSOR_SIXQ] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel] = delayInMs;
            break;

        case ANDROID_SENSOR_ORIENTATION:
        case ANDROID_SENSOR_ROTATION_VECTOR:
            #if defined MEMS_AUGMENTED_SENSORS
            // if augmented sensors are handled by this driver,
            // then the fastest 9quat-based sensor which is enabled
            // should be applied to all 9quat-based sensors
            delayInMs = inv_mems_augmented_sensors_set_odr(androidSensor, delayInMs);
            #endif
            inv_dmp_odr_delays[INV_SENSOR_NINEQ] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_NINEQ_accel] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_NINEQ_cpass] = delayInMs;
            break;

        case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            inv_dmp_odr_delays[INV_SENSOR_COMPASS] = delayInMs;
            break;

        case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
            inv_dmp_odr_delays[INV_SENSOR_CALIB_COMPASS] = delayInMs;
            break;

        case ANDROID_SENSOR_LIGHT:
        case ANDROID_SENSOR_PROXIMITY:
            inv_dmp_odr_delays[INV_SENSOR_ALS] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_ACCELEROMETER:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_ACCEL] = delayInMs;
            break;

            #if (MEMS_CHIP != HW_ICM30630)

        case ANDROID_SENSOR_WAKEUP_STEP_DETECTOR:
        case ANDROID_SENSOR_WAKEUP_STEP_COUNTER:
        case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_STEP_COUNTER] = fixed_ms;
            break;
            #endif

        case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GEOMAG] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GEOMAG_cpass] = delayInMs;
            break;

            #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

        case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_TILT_DETECTOR] = fixed_ms;
            break;
            #endif

        case ANDROID_SENSOR_B2S:
            inv_dmp_odr_delays[INV_SENSOR_BRING_TO_SEE] = fixed_ms;
            break;

        case ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GYRO] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_GYROSCOPE:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_CALIB_GYRO] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_GRAVITY:
        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
            #if defined MEMS_AUGMENTED_SENSORS
            // if augmented sensors are handled by this driver,
            // then the fastest 6quat-based sensor which is enabled
            // should be applied to all 6quat-based sensors
            delayInMs = inv_mems_augmented_sensors_set_odr(androidSensor, delayInMs);
            #endif
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ_accel] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_ORIENTATION:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
            #if defined MEMS_AUGMENTED_SENSORS
            // if augmented sensors are handled by this driver,
            // then the fastest 9quat-based sensor which is enabled
            // should be applied to all 9quat-based sensors
            delayInMs = inv_mems_augmented_sensors_set_odr(androidSensor, delayInMs);
            #endif
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_accel] = delayInMs;
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_cpass] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_COMPASS] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_CALIB_COMPASS] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_LIGHT:
        case ANDROID_SENSOR_WAKEUP_PROXIMITY:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_ALS] = delayInMs;
            break;

        case ANDROID_SENSOR_PRESSURE:
            inv_dmp_odr_delays[INV_SENSOR_PRESSURE] = delayInMs;
            break;

        case ANDROID_SENSOR_WAKEUP_PRESSURE:
            inv_dmp_odr_delays[INV_SENSOR_WAKEUP_PRESSURE] = delayInMs;
            break;

        case ANDROID_SENSOR_FLIP_PICKUP:
            inv_dmp_odr_delays[INV_SENSOR_FLIP_PICKUP] = fixed_ms;
            break;

        // not support yet
        case ANDROID_SENSOR_META_DATA:
        case ANDROID_SENSOR_TEMPERATURE:
        case ANDROID_SENSOR_AMBIENT_TEMPERATURE:
        case ANDROID_SENSOR_HUMIDITY:
        case ANDROID_SENSOR_HEART_RATE:
        case ANDROID_SENSOR_SCREEN_ROTATION:
        case ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE:
        case ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY:
        case ANDROID_SENSOR_WAKEUP_HEART_RATE:
            break;

        default:
            break;
    }

    result = inv_set_hw_smplrt_dmp_odrs();
    result |= inv_set_gyro_sf(inv_get_gyro_divider(), inv_get_gyro_fullscale());

    // debug get odr
    // result should be SAME as you entered in Ms in the Rolldice console
    // i.e. If you use: O a 63 [ Press capital O then 'a' then 63 then ENTER]
    // You should get the nearest number to 63 here if you debug  the 'test_odr'

    //inv_get_odr( androidSensor, &test_odr );

    inv_mems_allow_lpen_control();
    return result;
}

/*
   inv_get_odr()
   Function to Query DMP3 DataRate (ODR)

   *odr = inv_get_odr_in_units( );

    The result in odr_units saved in *odr param
*/
inv_error_t inv_get_odr(unsigned char SensorId, uint32_t *odr, enum INV_ODR_TYPE odr_units)
{
    inv_error_t result = 0;

    if(!odr) // sanity
        return -1;

    *odr = 0;

    /*
      You can obtain the odr in Milliseconds, Micro Seconds or Ticks.
      Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks,
      when calling inv_get_odr_in_units().
    */

    switch (SensorId)
    {
        case ANDROID_SENSOR_ACCELEROMETER:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_ACCEL], odr_units );
            break;

            #if (MEMS_CHIP != HW_ICM30630)

        case ANDROID_SENSOR_STEP_DETECTOR:
        case ANDROID_SENSOR_STEP_COUNTER:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_STEP_COUNTER], odr_units );
            break;
            #endif

        case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_GEOMAG], odr_units );
            break;

            #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

        case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_ACTIVITY_CLASSIFIER], odr_units );
            break;
            #endif

        case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_GYRO], odr_units );
            break;

        case ANDROID_SENSOR_GYROSCOPE:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_CALIB_GYRO], odr_units );
            break;

        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_GRAVITY:
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_SIXQ], odr_units );
            break;

        case ANDROID_SENSOR_ORIENTATION:
        case ANDROID_SENSOR_ROTATION_VECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_NINEQ], odr_units );
            break;

        case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_COMPASS], odr_units );
            break;

        case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_CALIB_COMPASS], odr_units );
            break;

        case ANDROID_SENSOR_LIGHT:
        case ANDROID_SENSOR_PROXIMITY:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_ALS], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_ACCELEROMETER:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_ACCEL], odr_units );
            break;

            #if (MEMS_CHIP != HW_ICM30630)

        case ANDROID_SENSOR_WAKEUP_STEP_DETECTOR:
        case ANDROID_SENSOR_WAKEUP_STEP_COUNTER:
        case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_STEP_COUNTER], odr_units );
            break;
            #endif

        case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_GEOMAG], odr_units );
            break;

            #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )

        case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_TILT_DETECTOR], odr_units );
            break;
            #endif

        case ANDROID_SENSOR_B2S:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_BRING_TO_SEE], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_GYRO], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_GYROSCOPE:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_CALIB_GYRO], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_SIXQ], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_GRAVITY:
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_SIXQ_accel], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_ORIENTATION:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_NINEQ], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_COMPASS], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_CALIB_COMPASS], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_LIGHT:
        case ANDROID_SENSOR_WAKEUP_PROXIMITY:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_ALS], odr_units );
            break;

        case ANDROID_SENSOR_PRESSURE:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_PRESSURE], odr_units );
            break;

        case ANDROID_SENSOR_WAKEUP_PRESSURE:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_PRESSURE], odr_units );
            break;

        case ANDROID_SENSOR_FLIP_PICKUP:
            *odr = inv_get_odr_in_units( inv_dmp_odr_dividers[INV_SENSOR_FLIP_PICKUP], odr_units );
            break;

        // not support yet
        case ANDROID_SENSOR_META_DATA:
        case ANDROID_SENSOR_TEMPERATURE:
        case ANDROID_SENSOR_AMBIENT_TEMPERATURE:
        case ANDROID_SENSOR_HUMIDITY:
        case ANDROID_SENSOR_HEART_RATE:
        case ANDROID_SENSOR_SCREEN_ROTATION:
        case ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE:
        case ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY:
        case ANDROID_SENSOR_WAKEUP_HEART_RATE:
            *odr = 0;
            break;

        default:
            *odr = 0;
    }

    return result;
}

void inv_reGenerate_sensorControl(const short *sen_num_2_ctrl, unsigned short *sensor_control)
{
    short delta;
    int i, cntr;
    unsigned long tmp_androidSensorsOn_mask;

    *sensor_control = 0;

    for (i = 0; i < 2; i++)
    {
        cntr = 32 * i;
        tmp_androidSensorsOn_mask = inv_androidSensorsOn_mask[i];

        while (tmp_androidSensorsOn_mask)
        {
            if (tmp_androidSensorsOn_mask & 1)
            {
                delta = sen_num_2_ctrl[cntr];

                if (delta != -1) *sensor_control |= delta;
            }

            tmp_androidSensorsOn_mask >>= 1;
            cntr++;
        }
    }
}

/** Computes the sensor control register that needs to be sent to the DMP
* @param[in] androidSensor A sensor number, the numbers correspond to sensors.h definition in Android
* @param[in] enable non-zero to turn sensor on, 0 to turn sensor off
* @param[in] sen_num_2_ctrl Table matching android sensor number to bits in DMP control register
* @param[in,out] sensor_control Sensor control register to write to DMP to enable/disable sensors
*/
static void inv_convert_androidSensor_to_control(unsigned char androidSensor, unsigned char enable, const short *sen_num_2_ctrl, unsigned short *sensor_control)
{
    short delta = 0;

    #if (MEMS_CHIP == HW_ICM20645_E)

    if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON || androidSensor == ANDROID_SENSOR_FLIP_PICKUP || androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR)
    {
        if (enable)
            *sensor_control |= HEADER2_SET;
        else
            // control has to be regenerated when removing sensors because of overlap
            inv_reGenerate_sensorControl(sen_num_2_ctrl, sensor_control);

        if (bac_status || flip_pickup_status)
            *sensor_control |= HEADER2_SET;
    }

    #endif

    #if (MEMS_CHIP == HW_ICM30630)

    if (androidSensor == ANDROID_SENSOR_FLIP_PICKUP)
    {
        if (enable)
            *sensor_control |= HEADER2_SET;
        else
            // control has to be regenerated when removing sensors because of overlap
            inv_reGenerate_sensorControl(sen_num_2_ctrl, sensor_control);

        if (flip_pickup_status)
            *sensor_control |= HEADER2_SET;
    }

    #endif

    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (bac_status || flip_pickup_status)
        *sensor_control |= HEADER2_SET;

    #endif

    if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
        return; // Sensor not supported

    delta = sen_num_2_ctrl[androidSensor];

    if (delta == -1)
        return; // This sensor not supported

    if (enable)
    {
        inv_androidSensorsOn_mask[(androidSensor >> 5)] |= 1L << (androidSensor & 0x1F); // Set bit
        *sensor_control |= delta;
    }
    else
    {
        inv_androidSensorsOn_mask[(androidSensor >> 5)] &= ~(1L << (androidSensor & 0x1F)); // Clear bit
        // control has to be regenerated when removing sensors because of overlap
        inv_reGenerate_sensorControl(sen_num_2_ctrl, sensor_control);
    }

    #if (MEMS_CHIP == HW_ICM30630)

    if (flip_pickup_status)
        *sensor_control |= HEADER2_SET;

    #endif
    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (bac_status || flip_pickup_status)
        *sensor_control |= HEADER2_SET;

    #endif

    return;
}

// return true 1 if gyro was enabled, otherwise false 0
unsigned char inv_is_gyro_enabled(void)
{
    if ((inv_androidSensorsOn_mask[0] & INV_NEEDS_GYRO_MASK) || (inv_androidSensorsOn_mask[1] & INV_NEEDS_GYRO_MASK1))
        return 1;

    return 0;
}

inv_error_t inv_enable_sensor(unsigned char androidSensor, unsigned char enable)
{
    inv_error_t result = 0;
    #if (MEMS_CHIP == HW_ICM30630)
    static unsigned char sGyroIsOn = 0;
    static unsigned char sGrvIsOn  = 0;
    #endif

    // Fixes #5227 : The very first sensor on/off is not working
    // 1 because mems is considered in sleep by default
    // BEfore it was 0, it was why the first reg write was not taken in account !!!
    static char mems_put_to_sleep = 1;

    #if defined MEMS_SECONDARY_DEVICE

    if(sensor_needs_compass(androidSensor))
        if(!inv_mems_get_compass_availability())
            return INV_ERROR_INVALID_PARAMETER;

    if(sensor_needs_pressure(androidSensor))
        if(!inv_mems_get_pressure_availability())
            return INV_ERROR_INVALID_PARAMETER;

    #endif

    inv_mems_prevent_lpen_control();

    if( mems_put_to_sleep )
    {
        mems_put_to_sleep = 0;
        result |= inv_wakeup_mems();
    }

    #if (MEMS_CHIP == HW_ICM30630)

    if ( (androidSensor == ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) || (androidSensor == ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR) )
    {
        sGmrvIsOn = enable;

        if (enable)
        {
            if ( sGyroIsOn || sGrvIsOn )
                // enable GMRV while one of gyro based sensor already ON
                // enable internally GRV instead of GMRV
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 1, &mems_put_to_sleep);
            else
                // enable GMRV and no gyro based sensor already ON
                // enable internally GMRV
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 1, &mems_put_to_sleep);
        }
        else
        {
            if ( ! (sGyroIsOn || sGrvIsOn)  )
                // disable GMRV and no gyro based sensor is ON
                // disable internally GMRV
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 0, &mems_put_to_sleep);

            // else disable GMRV but one gyro based sensor still ON so keep internally GRV ON => nothing to do
        }

        return result;
    }

    if ( (androidSensor == ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED) || (androidSensor == ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED) )
    {
        sGyroIsOn = enable;

        if (enable)
        {
            if (sGmrvIsOn)
            {
                // enable gyro while GMRV already on
                // enable internally GRV instead of GMRV
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 0, &mems_put_to_sleep);
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 1, &mems_put_to_sleep);
            }

            // else enable gyro and no GMRV already ON => nothing to do
        }
        else
        {
            if ( sGmrvIsOn )

                // disable gyro while GMRV still on
                if ( !sGrvIsOn )
                {
                    // if GRV also OFF, disable internally GRV and enable internally GMRV instead
                    result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 0, &mems_put_to_sleep);
                    result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 1, &mems_put_to_sleep);
                }

            // else if GRV is still ON, enable internally GRV instead of GMRV => nothing to do
            // else disable gyro and no GMRV is ON => nothing to do
        }

    }

    if ( (androidSensor == ANDROID_SENSOR_GAME_ROTATION_VECTOR) || (androidSensor == ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR) )
    {
        sGrvIsOn = enable;

        if (enable)
        {
            if ( sGmrvIsOn )
            {
                // enable GRV while GMRV already on
                // enable internally GRV instead of GMRV
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 0, &mems_put_to_sleep);
            }

            // else enable GRV and no GMRV already ON

            result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 1, &mems_put_to_sleep);
        }
        else
        {
            if ( sGmrvIsOn )
            {
                // disable GRV while GMRV still on
                if ( ! sGyroIsOn )
                {
                    // if gyro also OFF, disable internally GRV and enable internally GMRV instead
                    result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 0, &mems_put_to_sleep);
                    result |= inv_enable_sensor_internal(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 1, &mems_put_to_sleep);
                }

                // else if gyro is still ON, enable internally GRV instead of GMRV => nothing to do
            }
            else
            {
                // disable GRV and no GMRV is ON
                result |= inv_enable_sensor_internal(ANDROID_SENSOR_GAME_ROTATION_VECTOR, 0, &mems_put_to_sleep);
            }

        }

        return result;
    }

    #endif

    result |= inv_enable_sensor_internal(androidSensor, enable, &mems_put_to_sleep);

    inv_mems_allow_lpen_control();
    return result;
}

static inv_error_t inv_enable_sensor_internal(unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep)
{
    inv_error_t result = 0;
    unsigned short inv_event_control = 0;
    #if (MEMS_CHIP != HW_ICM30630)
    static unsigned short smd_status = 0;
    static unsigned short ped_int_status = 0;
    #endif
    unsigned short data_rdy_status = 0;

    unsigned long steps = 0;

    #if (MEMS_CHIP != HW_ICM30630)

    if (androidSensor == ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION)
    {
        if (enable)
            smd_status = INV_SMD_EN;
        else
            smd_status = 0;
    }

    if (androidSensor == ANDROID_SENSOR_STEP_DETECTOR)
    {
        if (enable)
            ped_int_status = INV_PEDOMETER_INT_EN;
        else
            ped_int_status = 0;
    }

    #endif

    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (androidSensor == ANDROID_SENSOR_FLIP_PICKUP)
    {
        if (enable)
        {
            flip_pickup_status = FLIP_PICKUP_SET;
            // 5061-add-call to reset flip pickup parameters here
            dmp_reset_pickup();
        }
        else
            flip_pickup_status = 0;
    }

    if (androidSensor == ANDROID_SENSOR_B2S)
    {
        if(enable)
            b2s_status = INV_BTS_EN;
        else
            b2s_status = 0;
    }

    #endif

    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )

    if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON)
        inv_enable_activity_classifier(enable);

    if (androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR)
        inv_enable_tilt(enable);

    #endif

    inv_convert_androidSensor_to_control(androidSensor, enable,
                                         inv_androidSensor_to_control_bits, &inv_sensor_control);
    #if (MEMS_CHIP == HW_ICM30630)

    // For 30630, raw gyro and gyro bias are sent out by turning on GYRO_SET
    // So to calculate calibrated gyro, GYRO_SET needs to be turned on, too
    if (inv_sensor_control & GYRO_CALIBR_SET)
        inv_sensor_control |= GYRO_SET;

    #endif
    result = dmp_set_data_output_control1(inv_sensor_control);

    if (b2s_status)
        result |= dmp_set_data_interrupt_control(inv_sensor_control | 0x8008);
    // result |= dmp_set_data_interrupt_control(inv_sensor_control|0x0000);
    else
        result |= dmp_set_data_interrupt_control(inv_sensor_control);

    if (inv_sensor_control & ACCEL_SET)
        inv_sensor_control2 |= ACCEL_ACCURACY_SET;
    else
        inv_sensor_control2 &= ~ACCEL_ACCURACY_SET;

    if (  (inv_sensor_control & GYRO_CALIBR_SET)
        #if (MEMS_CHIP == HW_ICM30630)
            || (inv_sensor_control & GYRO_SET)
        #endif
       )
        inv_sensor_control2 |= GYRO_ACCURACY_SET;
    else
        inv_sensor_control2 &= ~GYRO_ACCURACY_SET;

    #if defined MEMS_SECONDARY_DEVICE

    if (inv_sensor_control & CPASS_CALIBR_SET || inv_sensor_control & QUAT9_SET
        #if (MEMS_CHIP == HW_ICM20648)
            || inv_sensor_control & GEOMAG_SET
        #endif
       )
        inv_sensor_control2 |= CPASS_ACCURACY_SET;
    else
        inv_sensor_control2 &= ~CPASS_ACCURACY_SET;

    #endif
    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if(bac_status)
        inv_sensor_control2 |= bac_status;
    else
        inv_sensor_control2 &= ~bac_status;

    #endif
    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if(flip_pickup_status)
        inv_sensor_control2 |= flip_pickup_status;
    else
        inv_sensor_control2 &= ~flip_pickup_status;


    // inv_event_control   |= b2s_status;

    #if (MEMS_CHIP == HW_ICM20648)

    if(androidSensor == ANDROID_SENSOR_B2S)
    {
        if(enable)
            inv_event_control |= INV_BRING_AND_LOOK_T0_SEE_EN;
    }

    #endif
    #endif

    #if (MEMS_CHIP != HW_ICM20648)

    if(inv_androidSensorsOn_mask[0] & (1L << ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR))
        inv_sensor_control2 |= GEOMAG_EN;
    else
        inv_sensor_control2 &= ~GEOMAG_EN;

    #endif

    result |= dmp_set_data_output_control2(inv_sensor_control2);

    // sets DATA_RDY_STATUS in DMP based on which sensors are on
    if (inv_androidSensorsOn_mask[0] & INV_NEEDS_GYRO_MASK || inv_androidSensorsOn_mask[1] & INV_NEEDS_GYRO_MASK1)
        data_rdy_status |= GYRO_AVAILABLE;

    if (inv_androidSensorsOn_mask[0] & INV_NEEDS_ACCEL_MASK || inv_androidSensorsOn_mask[1] & INV_NEEDS_ACCEL_MASK1)
        data_rdy_status |= ACCEL_AVAILABLE;

    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (flip_pickup_status || b2s_status)
        data_rdy_status |= ACCEL_AVAILABLE;

    #endif
    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (bac_status)
        data_rdy_status |= ACCEL_AVAILABLE;

    #endif

    if (inv_androidSensorsOn_mask[0] & INV_NEEDS_COMPASS_MASK || inv_androidSensorsOn_mask[1] & INV_NEEDS_COMPASS_MASK1)
        data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;

    if (inv_androidSensorsOn_mask[0] & INV_NEEDS_PRESSURE)
        data_rdy_status |= SECONDARY_PRESSURE_AVAILABLE;

    // turn on gyro cal only if gyro is available
    if (data_rdy_status & GYRO_AVAILABLE)
        inv_event_control |= INV_GYRO_CAL_EN;

    inv_event_control |= INV_ACCEL_CAL_EN | INV_COMPASS_CAL_EN;
    #if (MEMS_CHIP != HW_ICM30630)
    inv_event_control |= smd_status | ped_int_status;
    #endif

    if (inv_sensor_control & QUAT9_SET)
        inv_event_control |= INV_NINE_AXIS_EN;

    #if (MEMS_CHIP != HW_ICM30630)

    if (inv_sensor_control & (PED_STEPDET_SET | PED_STEPIND_SET) || inv_event_control & INV_SMD_EN)
    {
        #if (MEMS_CHIP == HW_ICM20648)
        #if defined MEMS_WEARABLE_DEVICE
        inv_event_control |= INV_BAC_WEARABLE;
        #endif
        #endif
        inv_event_control |= INV_PEDOMETER_EN;
    }

    #endif
    #if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (inv_sensor_control2 & ACT_RECOG_SET)
    {
        inv_event_control |= INV_PEDOMETER_EN;
        #if (MEMS_CHIP == HW_ICM20648)
        #if defined MEMS_WEARABLE_DEVICE
        inv_event_control |= INV_BAC_WEARABLE;
        #endif
        #endif
    }

    #endif
    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648)

    if (inv_sensor_control2 & FLIP_PICKUP_SET)
    {
        inv_event_control |= FLIP_PICKUP_EN;
        // 5061-add-call to reset flip pickup parameters here
        dmp_reset_pickup();
    }

    #endif

    #if (MEMS_CHIP == HW_ICM20648)

    if (inv_sensor_control & GEOMAG_SET)
        inv_event_control |= GEOMAG_EN;

    #endif

    result |= dmp_set_motion_event_control(inv_event_control);

    #if defined MEMS_AUGMENTED_SENSORS

    // A sensor was just enabled/disabled, need to recompute the required ODR for all augmented sensor-related sensors
    // The fastest ODR will always be applied to other related sensors
    if ( (androidSensor == ANDROID_SENSOR_GRAVITY) || (androidSensor == ANDROID_SENSOR_GAME_ROTATION_VECTOR) || (androidSensor == ANDROID_SENSOR_LINEAR_ACCELERATION) )
    {
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_SIXQ]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel]);
    }

    if ( (androidSensor == ANDROID_SENSOR_ORIENTATION) || (androidSensor == ANDROID_SENSOR_ROTATION_VECTOR) )
    {
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_NINEQ]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_NINEQ_accel]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_NINEQ_cpass]);
    }

    if ( (androidSensor == ANDROID_SENSOR_WAKEUP_GRAVITY) || (androidSensor == ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR) || (androidSensor == ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION) )
    {
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ_accel]);
    }

    if ( (androidSensor == ANDROID_SENSOR_WAKEUP_ORIENTATION) || (androidSensor == ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) )
    {
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_accel]);
        inv_mems_augmented_sensors_update_odr(androidSensor, &inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_cpass]);
    }

    #endif

    result |= inv_set_hw_smplrt_dmp_odrs();
    result |= inv_set_gyro_sf(inv_get_gyro_divider(), inv_get_gyro_fullscale());

    if (!inv_sensor_control && !(inv_androidSensorsOn_mask[0] & (1L << ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION)) && !b2s_status)
    {
        *mems_put_to_sleep = 1 ;
        result |= inv_sleep_mems();
    }

    // DMP no longer controls PWR_MGMT_2 because of hardware bug, 0x80 set to override default behaviour of inv_enable_mems_hw_sensors()
    result |= inv_enable_mems_hw_sensors((int)data_rdy_status | 0x80);

    // set DATA_RDY_STATUS in DMP
    if ( (data_rdy_status & SECONDARY_COMPASS_AVAILABLE) ||
            (data_rdy_status & SECONDARY_PRESSURE_AVAILABLE) )
    {
        data_rdy_status &= ~SECONDARY_PRESSURE_AVAILABLE;
        data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;
    }

    result |= dmp_set_data_rdy_status(data_rdy_status);

    // To have the all steps when you enable the sensor
    if (androidSensor == ANDROID_SENSOR_STEP_COUNTER)
    {
        if (enable)
        {
            dmp_get_pedometer_get_all_steps(&steps);
            inv_set_dmp_stepcounter_update_offset(steps);
        }
    }

    return result;
}

#if (MEMS_CHIP == HW_ICM20645_E|| MEMS_CHIP == HW_ICM20648)
void inv_enable_activity_classifier(unsigned char enable)
{
    bac_on = enable;

    if (enable)
    {
        bac_status = ACT_RECOG_SET;
        dmp_set_tilt_enable(0);
    }
    else
    {
        // only disable tilt engine if no request for tilt sensor
        if (!inv_androidSensor_enabled(ANDROID_SENSOR_WAKEUP_TILT_DETECTOR))
        {
            bac_status = 0;
            dmp_set_tilt_enable(0);
        }
        // do not diable BAC engine if tilt sensor is still on because tilt is part of BAC
        else
        {
            dmp_set_tilt_enable(1);
        }
    }
}

void inv_enable_tilt(unsigned char enable)
{
    if (enable)
    {
        bac_status = ACT_RECOG_SET;

        if (!bac_on)
            dmp_set_tilt_enable(1); // turn off BAC FIFO output except tilt if BAC sensor is off
    }
    else
    {
        // do not disable BAC engine if BAC sensor is still on even though tilt is off
        if (!bac_on)
        {
            bac_status = 0;
            dmp_set_tilt_enable(0);
        }
    }
}
#endif

inv_error_t inv_enable_batch(unsigned char enable)
{
    int ret = 0;

    if(enable)
        inv_sensor_control2 |= BATCH_MODE_EN;
    else
        inv_sensor_control2 &= ~BATCH_MODE_EN;

    ret = dmp_set_data_output_control2(inv_sensor_control2);

    #if (MEMS_CHIP == HW_ICM20648)
    /* give batch mode status to mems transport layer
    to allow disable/enable LP_EN when reading FIFO in batch mode */
    inv_set_batch_mode_status(enable);
    #endif

    return ret;
}

#if (MEMS_CHIP == HW_ICM20648)
static unsigned char sBatchMode = 0;

void inv_set_batch_mode_status(unsigned char enable)
{
    if(enable)
        sBatchMode = 1;
    else
        sBatchMode = 0;
}

unsigned char inv_get_batch_mode_status(void)
{
    return sBatchMode;
}
#endif

inv_error_t inv_set_batch_timeout(unsigned short batch_time_in_seconds)
{
    unsigned int timeout = 0;

    if(inv_sensor_control & GYRO_CALIBR_SET || inv_sensor_control & QUAT6_SET || inv_sensor_control & QUAT9_SET || inv_sensor_control & GYRO_SET)   // If Gyro based sensor is enabled.
    {
        timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE / (inv_get_gyro_divider() + 1)));
        return dmp_set_batchmode_params(timeout, GYRO_AVAILABLE);
    }

    if(inv_sensor_control & ACCEL_SET
        #if (MEMS_CHIP == HW_ICM20648)
            || inv_sensor_control & GEOMAG_SET
        #endif
      )      // If Accel is enabled and no Gyro based sensor is enabled.
    {
        timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE / (inv_get_accel_divider() + 1)));
        return dmp_set_batchmode_params(timeout, ACCEL_AVAILABLE);
    }

    #if defined MEMS_SECONDARY_DEVICE

    if(inv_sensor_control & CPASS_SET || inv_sensor_control & CPASS_CALIBR_SET || inv_sensor_control & PRESSURE_SET)
    {
        int rc = 0;

        timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE / inv_get_secondary_divider()));

        if(inv_sensor_control & PRESSURE_SET)
        {
            rc = dmp_set_batchmode_params(timeout, SECONDARY_PRESSURE_AVAILABLE);
        }

        if(inv_sensor_control & CPASS_SET || inv_sensor_control & CPASS_CALIBR_SET)
        {
            rc |= dmp_set_batchmode_params(timeout, SECONDARY_COMPASS_AVAILABLE);
        }

        return rc;
    }

    #endif

    return -1;  // Call batch only when a sensor is enabled.
}

inv_error_t inv_set_batch_timeout_ms(unsigned short batch_time_in_ms)
{
    unsigned int timeout = 0;

    if(inv_sensor_control & GYRO_CALIBR_SET || inv_sensor_control & QUAT6_SET || inv_sensor_control & QUAT9_SET || inv_sensor_control & GYRO_SET)   // If Gyro based sensor is enabled.
    {
        timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE / (inv_get_gyro_divider() + 1))) / 1000);
        return dmp_set_batchmode_params(timeout, GYRO_AVAILABLE);
    }

    if(inv_sensor_control & ACCEL_SET
        #if (MEMS_CHIP == HW_ICM20648)
            || inv_sensor_control & GEOMAG_SET
        #endif
      )      // If Accel is enabled and no Gyro based sensor is enabled.
    {
        timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE / (inv_get_accel_divider() + 1))) / 1000);
        return dmp_set_batchmode_params(timeout, ACCEL_AVAILABLE);
    }

    #if defined MEMS_SECONDARY_DEVICE

    if(inv_sensor_control & CPASS_SET || inv_sensor_control & CPASS_CALIBR_SET || inv_sensor_control & PRESSURE_SET)
    {
        int rc = 0;

        timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE / inv_get_secondary_divider())) / 1000);

        if(inv_sensor_control & PRESSURE_SET)
        {
            rc = dmp_set_batchmode_params(timeout, SECONDARY_PRESSURE_AVAILABLE);
        }

        if(inv_sensor_control & CPASS_SET || inv_sensor_control & CPASS_CALIBR_SET)
        {
            rc |= dmp_set_batchmode_params(timeout, SECONDARY_COMPASS_AVAILABLE);
        }

        return rc;
    }

    #endif

    return -1;  // Call batch only when a sensor is enabled.
}

/** Each bit corresponds to a sensor being on (Sensors 0 to 21)
*/
unsigned long *inv_get_androidSensorsOn_mask()
{
    return inv_androidSensorsOn_mask;
}

#if (MEMS_CHIP == HW_ICM20645_E || MEMS_CHIP == HW_ICM20648 )
unsigned short inv_get_activitiy_classifier_on_flag()
{
    return bac_on;
}
#endif

/** @brief Sets accel quaternion gain according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
inv_error_t inv_set_accel_quaternion_gain(unsigned short hw_smplrt_divider)
{
    inv_error_t result = INV_SUCCESS;
    int accel_gain = 15252014L; //set 225Hz gain as default

    switch (hw_smplrt_divider)
    {
        case 5:     //1125Hz/5 = 225Hz
            accel_gain = 15252014L;
            break;

        case 10:    //1125Hz/10 = 112Hz
            accel_gain = 30504029L;
            break;

        case 11:    //1125Hz/11 = 102Hz
            accel_gain = 33554432L;
            break;

        case 22:    //1125Hz/22 = 51Hz
            accel_gain = 67108864L;
            break;

        default:
            accel_gain = 15252014L;
            break;
    }

    result = dmp_set_accel_feedback_gain(accel_gain);

    return result;
}

inv_error_t inv_set_accel_cal_params(unsigned short hw_smplrt_divider)
{
    inv_error_t result = INV_SUCCESS;
    int accel_cal_params[NUM_ACCEL_CAL_PARAMS] = {0};

    if (hw_smplrt_divider <= 5)   // freq = 225Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 1026019965L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 47721859L;
    }
    else if (hw_smplrt_divider <= 10)     // 225Hz > freq >= 112Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 977872018L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 95869806L;
    }
    else if (hw_smplrt_divider <= 11)     // 112Hz > freq >= 102Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 858993459L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 214748365L;
        accel_cal_params[ACCEL_CAL_DIV] = 1;
    }
    else if (hw_smplrt_divider <= 20)     // 102Hz > freq >= 56Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 882002213L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 191739611L;
    }
    else if (hw_smplrt_divider <= 22)     // 56Hz > freq >= 51Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 858993459L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 214748365L;
    }
    else if (hw_smplrt_divider <= 75)     // 51Hz > freq >= 15Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 357913941L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 715827883L;
    }
    else if (hw_smplrt_divider <= 225)     // 15Hz > freq >= 5Hz
    {
        accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 107374182L;
        accel_cal_params[ACCEL_CAL_A_VAR] = 966367642L;
    }

    result = dmp_set_accel_cal_params(accel_cal_params);

    return result;
}

/* 5061:  this should be used to disable PICKUp after it triggers once
 * DO WE NEED TO CLEAR A BIT IN EVENT CONTROL?
 */
#if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E ||MEMS_CHIP == HW_ICM20648)
inv_error_t inv_enable_pickup(unsigned char enable)
{
    int ret = 0;

    if(enable)
        inv_sensor_control2 |= FLIP_PICKUP_EN;
    else
        inv_sensor_control2 &= ~FLIP_PICKUP_EN;

    ret = dmp_set_data_output_control2(inv_sensor_control2);

    return ret;
}
#endif


#if defined MEMS_SECONDARY_DEVICE
static unsigned char sensor_needs_compass(unsigned char androidSensor)
{
    switch(androidSensor)
    {
        case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
        case ANDROID_SENSOR_ROTATION_VECTOR:
        case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
        case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
        case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
            return 1;

        default :
            return 0;
    }
}
static unsigned char sensor_needs_pressure(unsigned char androidSensor)
{
    switch(androidSensor)
    {
        case ANDROID_SENSOR_PRESSURE:
        case ANDROID_SENSOR_WAKEUP_PRESSURE:
            return 1;

        default :
            return 0;
    }
}
#endif
#endif
