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
#if !defined MEMS_20609

#include "inv_mems_base_driver.h"

#include "inv_mems_transport.h"
#include "inv_mems_hw_config.h"
#include "inv_mems_interface_mapping.h"
#include "inv_mems_base_control.h"

#if defined MEMS_SECONDARY_DEVICE
    #include "inv_mems_slave_compass.h"
    #include "inv_mems_slave_pressure.h"
    #include "inv_mems_secondary_transport.h"
#endif

#include "invn_types.h"


struct base_driver_t base_state;
static uint8_t sAllowLpEn = 1;
#if defined MEMS_SECONDARY_DEVICE
    static uint8_t s_compass_available = 0;
    static uint8_t s_pressure_available = 0;
#endif

void inv_mems_prevent_lpen_control(void)
{
    sAllowLpEn = 0;
}
void inv_mems_allow_lpen_control(void)
{
    sAllowLpEn = 1;
    inv_set_chip_power_state(CHIP_LP_ENABLE, 1);
}
static uint8_t inv_mems_get_lpen_control(void)
{
    return sAllowLpEn;
}

/*!
 ******************************************************************************
 *   @brief     This function sets the power state of the Ivory chip
 *				loop
 *   @param[in] Function - CHIP_AWAKE, CHIP_LP_ENABLE
 *   @param[in] On/Off - The functions are enabled if previously disabled and
                disabled if previously enabled based on the value of On/Off.
 ******************************************************************************
 */
inv_error_t inv_set_chip_power_state(unsigned char func, unsigned char on_off)
{
    inv_error_t status = 0;

    switch(func)
    {

        case CHIP_AWAKE:
            if(on_off)
            {
                if((base_state.wake_state & CHIP_AWAKE) == 0)  // undo sleep_en
                {
                    base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
                    status = inv_write_single_mems_reg_core(REG_PWR_MGMT_1, base_state.pwr_mgmt_1);
                    base_state.wake_state |= CHIP_AWAKE;
                    inv_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
                }
            }
            else
            {
                if(base_state.wake_state & CHIP_AWAKE)  // set sleep_en
                {
                    base_state.pwr_mgmt_1 |= BIT_SLEEP;
                    status = inv_write_single_mems_reg_core(REG_PWR_MGMT_1, base_state.pwr_mgmt_1);
                    base_state.wake_state &= ~CHIP_AWAKE;
                    inv_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
                }
            }

            break;

        case CHIP_LP_ENABLE:
            if(base_state.lp_en_support == 1)
            {
                if(on_off)
                {
                    if( (inv_mems_get_lpen_control()) && ((base_state.wake_state & CHIP_LP_ENABLE) == 0))
                    {
                        base_state.pwr_mgmt_1 |= BIT_LP_EN; // lp_en ON
                        status = inv_write_single_mems_reg_core(REG_PWR_MGMT_1, base_state.pwr_mgmt_1);
                        base_state.wake_state |= CHIP_LP_ENABLE;
                        #if (MEMS_CHIP != HW_ICM20648)
                        inv_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
                        #endif
                    }
                }
                else
                {
                    if(base_state.wake_state & CHIP_LP_ENABLE)
                    {
                        base_state.pwr_mgmt_1 &= ~BIT_LP_EN; // lp_en off
                        status = inv_write_single_mems_reg_core(REG_PWR_MGMT_1, base_state.pwr_mgmt_1);
                        base_state.wake_state &= ~CHIP_LP_ENABLE;
                        inv_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
                    }
                }
            }

            break;

        default:
            break;

    }// end switch

    return status;
}

/*!
 ******************************************************************************
 *   @return    Current wake status of the Ivory chip.
 ******************************************************************************
 */
uint8_t inv_get_chip_power_state()
{
    return base_state.wake_state;
}

/** Wakes up DMP3 (MEMS).
*/
inv_error_t inv_wakeup_mems()
{
    unsigned char data;
    inv_error_t result = INV_SUCCESS;

    result = inv_set_chip_power_state(CHIP_AWAKE, 1);

    if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
    {
        base_state.user_ctrl |= BIT_I2C_IF_DIS;
        inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);
    }

    data = 0x47;	// FIXME, should set up according to sensor/engines enabled.
    result |= inv_write_mems_reg(REG_PWR_MGMT_2, 1, &data);

    if(base_state.firmware_loaded == 1)
    {
        base_state.user_ctrl |= BIT_DMP_EN | BIT_FIFO_EN;
        result |= inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);
    }

    result |= inv_set_chip_power_state(CHIP_LP_ENABLE, 1);
    return result;
}

/** Puts DMP3 (MEMS) into the lowest power state. Assumes sensors are all off.
*/
inv_error_t inv_sleep_mems()
{
    inv_error_t result;
    unsigned char data;

    data = 0x7F;
    result = inv_write_mems_reg(REG_PWR_MGMT_2, 1, &data);

    result |= inv_set_chip_power_state(CHIP_AWAKE, 0);

    return result;
}

inv_error_t inv_set_dmp_address()
{
    inv_error_t result;
    unsigned char dmp_cfg[2] = {0};
    unsigned short config;

    // Write DMP Start address
    inv_get_dmp_start_address(&config);
    /* setup DMP start address and firmware */
    dmp_cfg[0] = (unsigned char)((config >> 8) & 0xff);
    dmp_cfg[1] = (unsigned char)(config & 0xff);

    result = inv_write_mems_reg(REG_PRGM_START_ADDRH, 2, dmp_cfg);
    return result;
}

/**
*  @brief      Set up the secondary I2C bus on 20630.
*  @param[in]  MPU state varible
*  @return     0 if successful.
*/

inv_error_t inv_set_secondary()
{
    inv_error_t r = 0;
    static uint8_t lIsInited = 0;

    if(lIsInited == 0)
    {
        r = inv_write_single_mems_reg(REG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR);
        r |= inv_write_single_mems_reg(REG_I2C_MST_ODR_CONFIG, MIN_MST_ODR_CONFIG);
        r |= inv_write_single_mems_reg(REG_I2C_MST_DELAY_CTRL, BIT_DELAY_ES_SHADOW);
        lIsInited = 1;
    }

    return r;
}

/** Should be called once on power up. Loads DMP3, initializes internal variables needed
*   for other lower driver functions.
*/
inv_error_t inv_initialize_lower_driver(enum MEMS_SERIAL_INTERFACE type, const unsigned char *dmp_image_sram)
{
    inv_error_t result = 0;
    static unsigned char data;

    /* Reset the chip */
    result |= inv_write_single_mems_reg(REG_PWR_MGMT_1, BIT_H_RESET);
    inv_sleep(POWER_UP_TIME);

    // Set varialbes to default values
    memset(&base_state, 0, sizeof(base_state));
    base_state.pwr_mgmt_1 = BIT_CLK_PLL;
    base_state.pwr_mgmt_2 = BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY | BIT_PWR_PRESSURE_STBY;
    base_state.serial_interface = type;
    result |= inv_read_mems_reg(REG_USER_CTRL, 1, &base_state.user_ctrl);

    result |= inv_wakeup_mems();

    result |= inv_read_mems_reg(REG_WHO_AM_I, 1, &data);


    #if defined MEMS_SECONDARY_DEVICE
    /* secondary cycle mode should be set all the time */
    data = BIT_I2C_MST_CYCLE | BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE;
    #else
    data = BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE;
    #endif
    result |= inv_write_mems_reg(REG_LP_CONFIG, 1, &data);

    // Disable Ivory DMP.
    if(base_state.serial_interface == SERIAL_INTERFACE_SPI)
        base_state.user_ctrl = BIT_I2C_IF_DIS;
    else
        base_state.user_ctrl = 0;

    result |= inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);

    //Setup Ivory DMP.
    result |= inv_load_firmware(dmp_image_sram);

    if(result)
        return result;
    else
        base_state.firmware_loaded = 1;

    result |= inv_set_dmp_address();
    // Turn off all sensors on DMP by default.
    //result |= dmp_set_data_output_control1(0);   // FIXME in DMP, these should be off by default.
    result |= dmp_reset_control_registers();

    // set FIFO watermark to 80% of actual FIFO size
    result |= dmp_set_FIFO_watermark(800);

    // Enable Interrupts.
    data = 0x2;
    result |= inv_write_mems_reg(REG_INT_ENABLE, 1, &data); // Enable DMP Interrupt
    data = 0x1;
    result |= inv_write_mems_reg(REG_INT_ENABLE_2, 1, &data); // Enable FIFO Overflow Interrupt



    #if (MEMS_CHIP == HW_ICM20648)
    // TRACKING : To have accelerometers datas and the interrupt without gyro enables.
    data = 0XE4;
    result |= inv_write_mems_reg(REG_SINGLE_FIFO_PRIORITY_SEL, 1, &data);

    // Disable HW temp fix
    inv_read_mems_reg(REG_HW_FIX_DISABLE, 1, &data);
    data |= 0x08;
    inv_write_mems_reg(REG_HW_FIX_DISABLE, 1, &data);
    #endif

    // Setup MEMs properties.
    base_state.accel_averaging = 1; //Change this value if higher sensor sample avergaing is required.
    base_state.gyro_averaging = 1;  //Change this value if higher sensor sample avergaing is required.
    inv_set_gyro_divider(FIFO_DIVIDER);       //Initial sampling rate 1125Hz/10+1 = 102Hz.
    inv_set_accel_divider(FIFO_DIVIDER);      //Initial sampling rate 1125Hz/10+1 = 102Hz.
    
    // measuring span
    result |= inv_set_gyro_fullscale(MPU_FS_2000dps);
    result |= inv_set_accel_fullscale(MPU_FS_2G);

    // FIFO Setup.
    result |= inv_write_single_mems_reg(REG_FIFO_CFG, BIT_SINGLE_FIFO_CFG); // FIFO Config. fixme do once? burst write?
    result |= inv_write_single_mems_reg(REG_FIFO_RST, 0x1f); // Reset all FIFOs.
    result |= inv_write_single_mems_reg(REG_FIFO_RST, 0x1e); // Keep all but Gyro FIFO in reset.
    result |= inv_write_single_mems_reg(REG_FIFO_EN, 0x0); // Slave FIFO turned off.
    result |= inv_write_single_mems_reg(REG_FIFO_EN_2, 0x0); // Hardware FIFO turned off.

    result |= inv_read_mems(MPU_SOFT_UPDT_ADDR, 1, &data);
    #if (MEMS_CHIP == HW_ICM20648)

    // Check board version
    if (data & 0x8)
        return INV_ERROR_INVALID_CONFIGURATION;

    #endif

    base_state.lp_en_support = 1;

    #if (MEMS_CHIP == HW_ICM30630 || MEMS_CHIP == HW_ICM20645_E)
    // Check LP_EN support.
    data &= MPU_SOFT_UPTD_MASK;

    if (data != 0x04)
        base_state.lp_en_support = 0;

    #endif

    if(base_state.lp_en_support == 1)
        inv_set_chip_power_state(CHIP_LP_ENABLE, 1);

    result |= inv_sleep_mems();

    return result;
}

#if defined MEMS_SECONDARY_DEVICE

static void activate_compass(void)
{
    s_compass_available = 1;
}

static void desactivate_compass(void)
{
    s_compass_available = 0;
}

int inv_mems_get_compass_availability(void)
{
    return s_compass_available;
}

static void activate_pressure(void)
{
    s_pressure_available = 1;
}

static void desactivate_pressure(void)
{
    s_pressure_available = 0;
}

int inv_mems_get_pressure_availability(void)
{
    return s_pressure_available;
}

inv_error_t inv_set_slave_compass_id(int id)
{
    inv_error_t result = 0;

    //result = inv_wakeup_mems();
    //if (result)
    //	return result;

    inv_mems_prevent_lpen_control();
    activate_compass();

    inv_init_secondary();

    // Set up the secondary I2C bus on 20630.
    inv_set_secondary();

    //Setup Compass
    result = inv_setup_compass_akm();

    //Setup Compass mounting matrix into DMP
    result |= inv_compass_dmp_cal(ACCEL_GYRO_ORIENTATION, COMPASS_ORIENTATION);

    if (result)
        desactivate_compass();

    //result = inv_sleep_mems();
    inv_mems_allow_lpen_control();
    return result;
}
    
inv_error_t inv_set_slave_pressure_id(void)
{
    inv_error_t result = 0;

    //result = inv_wakeup_mems();
    //if (result)
    //	return result;

    inv_mems_prevent_lpen_control();
    activate_pressure();

    inv_init_secondary();

    // Set up the secondary I2C bus on 20630.
    inv_set_secondary();

    //Setup Compass
    result = inv_mems_pressure_setup_bmp();

    if (result)
        desactivate_pressure();

    //result = inv_sleep_mems();
    inv_mems_allow_lpen_control();
    return result;
}
#endif

inv_error_t inv_set_gyro_divider(unsigned char div)
{
    base_state.gyro_div = div;
    return inv_write_mems_reg(REG_GYRO_SMPLRT_DIV, 1, &div);
}

unsigned char inv_get_gyro_divider()
{
    return base_state.gyro_div;
}

inv_error_t inv_set_secondary_divider(unsigned char div)
{
    base_state.secondary_div = 1UL << div;

    return inv_write_single_mems_reg(REG_I2C_MST_ODR_CONFIG, div);
}

unsigned short inv_get_secondary_divider()
{
    return base_state.secondary_div;
}

inv_error_t inv_set_accel_divider(short div)
{
    unsigned char data[2] = {0};

    base_state.accel_div = div;
    data[0] = (unsigned char)(div >> 8);
    data[1] = (unsigned char)(div & 0xff);

    return inv_write_mems_reg(REG_ACCEL_SMPLRT_DIV_1, 2, data);
}

short inv_get_accel_divider()
{
    return base_state.accel_div;
}


// not exported in Headers
extern unsigned long inv_androidSensor_enabled(unsigned char androidSensor);
unsigned char inv_is_gyro_enabled(void);

/*
 You can obtain the real odr in Milliseconds, Micro Seconds or Ticks.
 Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks,
 when calling inv_get_odr_in_units().
*/
uint32_t inv_get_odr_in_units( unsigned short odrInDivider, unsigned char odr_units )
{
    uint32_t odr = 0;
    uint32_t Us = 0;
    unsigned char PLL = 0, gyro_is_on = 0;

    if(base_state.timebase_correction_pll == 0)
        inv_read_mems_reg(REG_TIMEBASE_CORRECTION_PLL, 1, &base_state.timebase_correction_pll);

    PLL = base_state.timebase_correction_pll;

    // check if Gyro is currently enabled
    gyro_is_on = inv_is_gyro_enabled();

    if( PLL < 0x80 ) // correction positive
    {
        // In Micro Seconds
        Us = (odrInDivider * 1000000L / 1125L) * (1270L) / (1270L + (gyro_is_on ? PLL : 0));

    }
    else
    {

        PLL &= 0x7F;

        // In Micro Seconds
        Us = (odrInDivider * 1000000L / 1125L) * (1270L) / (1270L - (gyro_is_on ? PLL : 0));
    }

    switch( odr_units )
    {

        // ret in Milliseconds
        case ODR_IN_Ms:
            odr = Us / 1000;
            break;

        // ret in Micro
        case ODR_IN_Us:
            odr = Us;
            break;

        // ret in Ticks
        case ODR_IN_Ticks:
            odr = (Us / 1000) * (32768 / 1125); // According to Mars
            break;
    }

    return odr;
}

/**
* Sets the DMP for a particular gyro configuration.
* @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
*            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
*            10=102.2727Hz sample rate, ... etc.
* @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
*/
inv_error_t inv_set_gyro_sf(unsigned char div, int gyro_level)
{
    long gyro_sf;
    static long lLastGyroSf = 0;
    inv_error_t result = 0;

    if(base_state.timebase_correction_pll == 0)
        result |= inv_read_mems_reg(REG_TIMEBASE_CORRECTION_PLL, 1, &base_state.timebase_correction_pll);

    {
        unsigned    long    long    const   MagicConstant       =   264446880937391LL;
        unsigned    long    long    const   MagicConstantScale  =   100000LL;
        unsigned    long    long            ResultLL;

        if  (base_state.timebase_correction_pll & 0x80)
        {
            ResultLL    = (MagicConstant * (long long)(1UL << gyro_level) * (1 + div) / (1270 - (base_state.timebase_correction_pll & 0x7F)) / MagicConstantScale);
        }
        else
        {
            ResultLL    = (MagicConstant * (long long)(1UL << gyro_level) * (1 + div) / (1270 + base_state.timebase_correction_pll) / MagicConstantScale);
        }

        /*
            In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
            Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
            the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
        */
        if  (ResultLL > 0x7FFFFFFF)
        {
            gyro_sf =   0x7FFFFFFF;
        }
        else
        {
            gyro_sf =   (long)ResultLL;
        }
    }

    if (gyro_sf != lLastGyroSf)
    {
        result |= dmp_set_gyro_sf(gyro_sf);
        lLastGyroSf = gyro_sf;
    }

    return result;
}

inv_error_t inv_set_gyro_fullscale(int level)
{
    inv_error_t result;
    base_state.gyro_fullscale = level;
    result = inv_set_mems_gyro_fullscale(level);
    result |= inv_set_gyro_sf(base_state.gyro_div, level);

    return result;
}

uint8_t inv_get_gyro_fullscale()
{
    return base_state.gyro_fullscale;
}


inv_error_t inv_set_mems_gyro_fullscale(int level)
{
    inv_error_t result = 0;
    unsigned char gyro_config_1_reg;
    unsigned char gyro_config_2_reg;
    unsigned char dec3_cfg;

    if (level >= NUM_MPU_GFS)
        return INV_ERROR_INVALID_PARAMETER;

    result |= inv_read_mems_reg(REG_GYRO_CONFIG_1, 1, &gyro_config_1_reg);
    gyro_config_1_reg &= 0xC0;
    gyro_config_1_reg |= (level << 1) | 1;  //fchoice = 1, filter = 0.
    result |= inv_write_mems_reg(REG_GYRO_CONFIG_1, 1, &gyro_config_1_reg);

    result |= inv_read_mems_reg(REG_GYRO_CONFIG_2, 1, &gyro_config_2_reg);
    gyro_config_2_reg &= 0xF8;

    switch(base_state.gyro_averaging)
    {
        case 1:
            dec3_cfg = 0;
            break;

        case 2:
            dec3_cfg = 1;
            break;

        case 4:
            dec3_cfg = 2;
            break;

        case 8:
            dec3_cfg = 3;
            break;

        case 16:
            dec3_cfg = 4;
            break;

        case 32:
            dec3_cfg = 5;
            break;

        case 64:
            dec3_cfg = 6;
            break;

        case 128:
            dec3_cfg = 7;
            break;

        default:
            dec3_cfg = 0;
            break;
    }

    gyro_config_2_reg |= dec3_cfg;
    result |= inv_write_single_mems_reg(REG_GYRO_CONFIG_2, gyro_config_2_reg);

    return result;
}


inv_error_t inv_set_accel_fullscale(int level)
{
    inv_error_t result;
    base_state.accel_fullscale = level;
    result = inv_set_mems_accel_fullscale(level);
    result |= dmp_set_accel_fsr(2 << level);
    result |= dmp_set_accel_scale2(2 << level);

    return result;
}

uint8_t inv_get_accel_fullscale()
{
    return base_state.accel_fullscale;
}


inv_error_t inv_set_mems_accel_fullscale(int level)
{
    inv_error_t result = 0;
    unsigned char accel_config_1_reg;
    unsigned char accel_config_2_reg;
    unsigned char dec3_cfg;

    if (level >= NUM_MPU_AFS)
        return INV_ERROR_INVALID_PARAMETER;

    result |= inv_read_mems_reg(REG_ACCEL_CONFIG, 1, &accel_config_1_reg);
    accel_config_1_reg &= 0xC0;

    if(base_state.accel_averaging > 1)
        accel_config_1_reg |= (7 << 3) | (level << 1) | 1;   //fchoice = 1, filter = 7.
    else
        accel_config_1_reg |= (level << 1);  //fchoice = 0, filter = 0.

    result |= inv_write_single_mems_reg(REG_ACCEL_CONFIG, accel_config_1_reg);

    switch(base_state.accel_averaging)
    {
        case 1:
            dec3_cfg = 0;
            break;

        case 4:
            dec3_cfg = 0;
            break;

        case 8:
            dec3_cfg = 1;
            break;

        case 16:
            dec3_cfg = 2;
            break;

        case 32:
            dec3_cfg = 3;
            break;

        default:
            dec3_cfg = 0;
            break;
    }

    result |= inv_read_mems_reg(REG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);
    accel_config_2_reg &= 0xFC;

    accel_config_2_reg |=  dec3_cfg;
    result |= inv_write_single_mems_reg(REG_ACCEL_CONFIG_2, accel_config_2_reg);

    return result;
}


inv_error_t inv_enable_mems_hw_sensors(int bit_mask)
{
    inv_error_t rc = INV_SUCCESS;

    if ((base_state.pwr_mgmt_2 == (BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY | BIT_PWR_PRESSURE_STBY)) | (bit_mask & 0x80))
    {
        // All sensors off, or override is on
        base_state.pwr_mgmt_2 = 0; // Zero means all sensors are on

        // Gyro and Accel were off
        if ((bit_mask & 2) == 0)
        {
            base_state.pwr_mgmt_2 = BIT_PWR_ACCEL_STBY; // Turn off accel
        }

        if ((bit_mask & 1) == 0)
        {
            base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY; // Turn off gyro
        }

        if ((bit_mask & 4) == 0)
        {
            base_state.pwr_mgmt_2 |= BIT_PWR_PRESSURE_STBY; // Turn off pressure
        }

        rc |= inv_write_mems_reg(REG_PWR_MGMT_2, 1, &base_state.pwr_mgmt_2);
    }

    #if defined MEMS_SECONDARY_DEVICE
    {
        if (bit_mask & SECONDARY_COMPASS_AVAILABLE)
        {
            rc |= inv_resume_akm();
        }
        else
            rc |= inv_suspend_akm();

        if (bit_mask & SECONDARY_PRESSURE_AVAILABLE)
        {
            rc |= inv_mems_pressure_resume_bmp();
        }
        else
        {
            rc |= inv_mems_pressure_suspend_bmp();
        }
    }
    #endif
    return rc;
}

inv_error_t inv_set_serial_comm(enum MEMS_SERIAL_INTERFACE type)
{
    base_state.serial_interface = type;

    return INV_SUCCESS;
}


inv_error_t inv_set_int1_assertion(int enable)
{
    inv_error_t   result = 0;
    // unsigned char reg_pin_cfg;
    unsigned char reg_int_enable;

    // INT1 held until interrupt status is cleared
    /*
    result         |= inv_read_mems_reg(REG_INT_PIN_CFG, 1, &reg_pin_cfg);
    reg_pin_cfg    |= BIT_INT_LATCH_EN ;	// Latchen : BIT5 held the IT until register is read
    result         |= inv_write_single_mems_reg(REG_INT_PIN_CFG, reg_pin_cfg);
    */

    // Set int1 enable
    result         |= inv_read_mems_reg(REG_INT_ENABLE, 1, &reg_int_enable);

    if(enable)
    {
        // Enable bit
        reg_int_enable |= BIT_DMP_INT_EN;
    }
    else
    {
        // Disable bit
        reg_int_enable &= ~BIT_DMP_INT_EN;
    }

    result         |= inv_write_single_mems_reg(REG_INT_ENABLE, reg_int_enable);

    return result;
}


/**
*  @brief      Read accel data stored in hw reg
*  @param[in]  level  See mpu_accel_fs
*  @return     INV_SUCCESS if successful
*/
inv_error_t inv_accel_read_hw_reg_data(short accel_hw_reg_data[3])
{
    inv_error_t   result        = 0;
    uint8_t       accel_data[6];      // Store 6 bytes for that

    // read mem regs
    result         = inv_read_mems_reg(REG_ACCEL_XOUT_H_SH, 6, (unsigned char *) &accel_data);

    // Assign axys !
    accel_hw_reg_data[0] = (accel_data[0] << 8) + accel_data[1];
    accel_hw_reg_data[1] = (accel_data[2] << 8) + accel_data[3];
    accel_hw_reg_data[2] = (accel_data[4] << 8) + accel_data[5];

    return result;
}

/** @brief Reset ODR counters in DMP
* @return	0 on success, 1 if not available.
*/
inv_error_t inv_reset_dmp_odr_counters(void)
{
    inv_error_t ret = 0;
    unsigned char reg;

    reg = base_state.user_ctrl;

    reg &= ~BIT_DMP_EN;
    reg &= ~BIT_FIFO_EN;
    ret |= inv_write_single_mems_reg(REG_USER_CTRL, reg);
    inv_sleep(MSEC_PER_SEC / MPU_DEFAULT_DMP_FREQ);
    ret |= dmp_reset_odr_counters();
    ret |= inv_write_single_mems_reg(REG_USER_CTRL, base_state.user_ctrl);

    return ret;
}
#endif

