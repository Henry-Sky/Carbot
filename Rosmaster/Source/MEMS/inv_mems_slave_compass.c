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

#include "inv_mems_slave_compass.h"

#include "inv_mems_drv_hook.h"
#include "inv_mems_defines.h"
#include "inv_mems_data_converter.h"               // to get inv_q30_mult
#include "inv_mems_hw_config.h"
#include "inv_mems_transport.h"
#include "inv_mems_secondary_transport.h"
#include "inv_mems_slave_pressure.h"
#include "inv_mems_interface_mapping.h"



long final_matrix[9] = {0};
uint8_t compass_sens[3] = {0};
const short *st_upper;
const short *st_lower;
int scale = 0;
uint8_t dmp_on = 1;

/* AKM definitions */
#define REG_AKM_ID               0x00
#define REG_AKM_INFO             0x01
#define REG_AKM_STATUS           0x02
#define REG_AKM_MEASURE_DATA     0x03
#define REG_AKM_MODE             0x0A
#define REG_AKM_ST_CTRL          0x0C
#define REG_AKM_SENSITIVITY      0x10
#define REG_AKM8963_CNTL1        0x0A

/* AK09911 register definition */
#define REG_AK09911_DMP_READ    0x3
#define REG_AK09911_STATUS1     0x10
#define REG_AK09911_CNTL2       0x31
#define REG_AK09911_SENSITIVITY 0x60
#define REG_AK09911_MEASURE_DATA     0x11

/* AK09912 register definition */
#define REG_AK09912_DMP_READ    0x3
#define REG_AK09912_STATUS1     0x10
#define REG_AK09912_CNTL1       0x30
#define REG_AK09912_CNTL2       0x31
#define REG_AK09912_SENSITIVITY 0x60
#define REG_AK09912_MEASURE_DATA     0x11

/* AK09916 register definition */
#define REG_AK09916_DMP_READ    0x3
#define REG_AK09916_STATUS1     0x10
#define REG_AK09916_CNTL2       0x31
#define REG_AK09916_MEASURE_DATA 0x11

#define DATA_AKM_ID              0x48
#define DATA_AKM_MODE_PD	 0x00
#define DATA_AKM_MODE_SM	 0x01
#define DATA_AKM_MODE_ST	 0x08
#define DATA_AK09911_MODE_ST	 0x10
#define DATA_AK09912_MODE_ST	 0x10
#define DATA_AK09916_MODE_ST	 0x10
#define DATA_AKM_MODE_FR	 0x0F
#define DATA_AK09911_MODE_FR     0x1F
#define DATA_AK09912_MODE_FR     0x1F
#define DATA_AKM_SELF_TEST       0x40
#define DATA_AKM_DRDY            0x01
#define DATA_AKM8963_BIT         0x10
#define DATA_AKM_STAT_MASK       0x0C

/* 0.3 uT * (1 << 30) */
#define DATA_AKM8975_SCALE       322122547
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8972_SCALE       644245094
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8963_SCALE0      644245094
/* 0.15 uT * (1 << 30) */
#define DATA_AKM8963_SCALE1      161061273
/* 0.6 uT * (1 << 30) */
#define DATA_AK09911_SCALE       644245094
/* 0.15 uT * (1 << 30) */
#define DATA_AK09912_SCALE       161061273
/* 0.15 uT * (1 << 30) */
#define DATA_AK09916_SCALE       161061273
#define DATA_MLX_SCALE           (4915 * (1L << 15))
#define DATA_MLX_SCALE_EMPIRICAL (26214 * (1L << 15))

#define DATA_AKM8963_SCALE_SHIFT      4
#define DATA_AKM_MIN_READ_TIME            (9 * NSEC_PER_MSEC)

/* AK09912C NSF */
/* 0:disable, 1:Low, 2:Middle, 3:High */
#define DATA_AK9912_NSF  1
#define DATA_AK9912_NSF_SHIFT 5

#define DEF_ST_COMPASS_WAIT_MIN     (10 * 1000)
#define DEF_ST_COMPASS_WAIT_MAX     (15 * 1000)
#define DEF_ST_COMPASS_TRY_TIMES    10
#define DEF_ST_COMPASS_8963_SHIFT   2
#define X                           0
#define Y                           1
#define Z                           2

/* milliseconds between each access */
#define AKM_RATE_SCALE       10

#define DATA_AKM_99_BYTES_DMP   10
#define DATA_AKM_89_BYTES_DMP   9

static const short AKM8975_ST_Lower[3] = {-100, -100, -1000};
static const short AKM8975_ST_Upper[3] = {100, 100, -300};

static const short AKM8972_ST_Lower[3] = {-50, -50, -500};
static const short AKM8972_ST_Upper[3] = {50, 50, -100};

static const short AKM8963_ST_Lower[3] = {-200, -200, -3200};
static const short AKM8963_ST_Upper[3] = {200, 200, -800};

static const short AK09911_ST_Lower[3] = {-30, -30, -400};
static const short AK09911_ST_Upper[3] = {30, 30, -50};

static const short AK09912_ST_Lower[3] = {-200, -200, -1600};
static const short AK09912_ST_Upper[3] = {200, 200, -400};

static const short AK09916_ST_Lower[3] = { -200, -200, -1000 };
static const short AK09916_ST_Upper[3] = { 200, 200, -200 };

static uint8_t secondary_resume_state = 0;

static uint8_t sModeRegAddr;

/*
 *  inv_setup_compass_akm() - Configure akm series compass.
 */
int inv_setup_compass_akm()
{
    int result;
    unsigned char data[4];
    uint8_t sens, cmd;

    /* Read WHOAMI through I2C SLV for compass */
    result = inv_execute_read_secondary(COMPASS_I2C_SLV_READ, COMPASS_CHIP_ADDR, REG_AKM_ID, 1, data);

    if (result)
    {
        inv_log("Read secondary error: Compass.\r\n");
        return result;
    }

    if (data[0] != DATA_AKM_ID)
    {
        inv_log("Compass not found!!\r\n");
        return -1;
    }

    inv_log("Compass found.\r\n");


    /* Read conf and configure compass through I2C SLV for compass and subsequent channel */

    /* set AKM to Fuse ROM access mode */
    if (HW_AK09911 == COMPASS_SLAVE_ID)
    {
        sModeRegAddr = REG_AK09911_CNTL2;
        sens = REG_AK09911_SENSITIVITY;
        cmd = DATA_AK09911_MODE_FR;
    }
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
    {
        sModeRegAddr = REG_AK09912_CNTL2;
        sens = REG_AK09912_SENSITIVITY;
        cmd = DATA_AK09912_MODE_FR;
    }
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
    {
        sModeRegAddr = REG_AK09916_CNTL2;
        compass_sens[0] = 128;
        compass_sens[1] = 128;
        compass_sens[2] = 128;
    }
    else
    {
        sModeRegAddr = REG_AKM_MODE;
        sens = REG_AKM_SENSITIVITY;
        cmd = DATA_AKM_MODE_FR;
    }

    scale = 1;

    if (HW_AK8975 == COMPASS_SLAVE_ID)
    {
        st_upper = AKM8975_ST_Upper;
        st_lower = AKM8975_ST_Lower;
    }
    else if (HW_AK8972 == COMPASS_SLAVE_ID)
    {
        st_upper = AKM8972_ST_Upper;
        st_lower = AKM8972_ST_Lower;
    }
    else if (HW_AK8963 == COMPASS_SLAVE_ID)
    {
        st_upper = AKM8963_ST_Upper;
        st_lower = AKM8963_ST_Lower;
    }
    else if (HW_AK09911 == COMPASS_SLAVE_ID)
    {
        st_upper = AK09911_ST_Upper;
        st_lower = AK09911_ST_Lower;
    }
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
    {
        st_upper = AK09912_ST_Upper;
        st_lower = AK09912_ST_Lower;
    }
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
    {
        st_upper = AK09916_ST_Upper;
        st_lower = AK09916_ST_Lower;
        goto skip_akm_fuse_rom_read;
    }
    else
    {
        return -EINVAL;
    }

    result = inv_read_secondary(COMPASS_I2C_SLV_READ, COMPASS_CHIP_ADDR, sens, THREE_AXES); //skip

    if (result)
        return result;

    result = inv_execute_write_secondary(COMPASS_I2C_SLV_WRITE, COMPASS_CHIP_ADDR, sModeRegAddr, cmd); //skip

    if (result)
        return result;

    result = inv_read_mems_reg(REG_EXT_SLV_SENS_DATA_00, THREE_AXES, compass_sens);//skip

    if (result)
        return result;

    if (HW_AK09912 == COMPASS_SLAVE_ID)
    {
        result = inv_execute_write_secondary(COMPASS_I2C_SLV_WRITE, COMPASS_CHIP_ADDR, REG_AK09912_CNTL1,
                                             DATA_AK9912_NSF << DATA_AK9912_NSF_SHIFT);//skip

        if (result)
            return result;
    }

    /* Set compass in power down through I2C SLV for compass */
    result = inv_execute_write_secondary(COMPASS_I2C_SLV_WRITE, COMPASS_CHIP_ADDR, sModeRegAddr, DATA_AKM_MODE_PD);

    if (result)
        return result;

skip_akm_fuse_rom_read:
    secondary_resume_state = 1;
    return inv_suspend_akm();
}

int inv_check_akm_self_test()
{
    int result;
    unsigned char data[6], mode, addr;
    unsigned char counter, cntl;
    short x, y, z;
    unsigned char *sens;
    int shift;
    unsigned char slv_ctrl[2];
    unsigned char odr_cfg;

    addr = COMPASS_CHIP_ADDR;
    sens = compass_sens;

    /* back up registers */
    /* SLV0_CTRL */
    result = inv_read_mems_reg(REG_I2C_SLV0_CTRL, 1, &slv_ctrl[0]);

    if (result)
        return result;

    result = inv_write_single_mems_reg(REG_I2C_SLV0_CTRL, 0);

    if (result)
        return result;

    /* SLV1_CTRL */
    result = inv_read_mems_reg(REG_I2C_SLV1_CTRL, 1, &slv_ctrl[1]);

    if (result)
        return result;

    result = inv_write_single_mems_reg(REG_I2C_SLV1_CTRL, 0);

    if (result)
        return result;

    /* I2C_MST ODR */
    result = inv_read_mems_reg(REG_I2C_MST_ODR_CONFIG, 1, &odr_cfg);

    if (result)
        return result;

    result = inv_write_single_mems_reg(REG_I2C_MST_ODR_CONFIG, 0);

    if (result)
        return result;

    if (HW_AK09911 == COMPASS_SLAVE_ID)
        mode = REG_AK09911_CNTL2;
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
        mode = REG_AK09912_CNTL2;
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
        mode = REG_AK09916_CNTL2;
    else
        mode = REG_AKM_MODE;

    /* set to power down mode */
    result = inv_execute_write_secondary(0, addr, mode, DATA_AKM_MODE_PD);

    if (result)
        goto AKM_fail;

    /* write 1 to ASTC register */
    if ((HW_AK09911 != COMPASS_SLAVE_ID) &&
            (HW_AK09912 != COMPASS_SLAVE_ID) &&
            (HW_AK09916 != COMPASS_SLAVE_ID))
    {
        result = inv_execute_write_secondary(0, addr, REG_AKM_ST_CTRL, DATA_AKM_SELF_TEST);

        if (result)
            goto AKM_fail;
    }

    /* set self test mode */
    if (HW_AK09911 == COMPASS_SLAVE_ID)
        result = inv_execute_write_secondary(0, addr, mode, DATA_AK09911_MODE_ST);
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
        result = inv_execute_write_secondary(0, addr, mode, DATA_AK09912_MODE_ST);
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
        result = inv_execute_write_secondary(0, addr, mode, DATA_AK09916_MODE_ST);
    else
        result = inv_execute_write_secondary(0, addr, mode,	DATA_AKM_MODE_ST);

    if (result)
        goto AKM_fail;

    counter = DEF_ST_COMPASS_TRY_TIMES;

    while (counter > 0)
    {
//		usleep_range(DEF_ST_COMPASS_WAIT_MIN, DEF_ST_COMPASS_WAIT_MAX);
        inv_sleep(15);

        if (HW_AK09911 == COMPASS_SLAVE_ID)
            result = inv_execute_read_secondary(0, addr, REG_AK09911_STATUS1, 1, data);
        else if (HW_AK09912 == COMPASS_SLAVE_ID)
            result = inv_execute_read_secondary(0, addr, REG_AK09912_STATUS1, 1, data);
        else if (HW_AK09916 == COMPASS_SLAVE_ID)
            result = inv_execute_read_secondary(0, addr, REG_AK09916_STATUS1, 1, data);
        else
            result = inv_execute_read_secondary(0, addr, REG_AKM_STATUS, 1, data);

        if (result)
            goto AKM_fail;

        if ((data[0] & DATA_AKM_DRDY) == 0)
            counter--;
        else
            counter = 0;
    }

    if ((data[0] & DATA_AKM_DRDY) == 0)
    {
        result = -EINVAL;
        goto AKM_fail;
    }

    if (HW_AK09911 == COMPASS_SLAVE_ID)
    {
        result = inv_execute_read_secondary(0, addr, REG_AK09911_MEASURE_DATA, BYTES_PER_SENSOR, data);
    }
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
    {
        result = inv_execute_read_secondary(0, addr, REG_AK09912_MEASURE_DATA, BYTES_PER_SENSOR, data);
    }
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
    {
        result = inv_execute_read_secondary(0, addr, REG_AK09916_MEASURE_DATA, BYTES_PER_SENSOR, data);
    }
    else
    {
        result = inv_execute_read_secondary(0, addr, REG_AKM_MEASURE_DATA, BYTES_PER_SENSOR, data);
    }

    if (result)
        goto AKM_fail;

    x = ((short)data[1]) << 8 | data[0];
    y = ((short)data[3]) << 8 | data[2];
    z = ((short)data[5]) << 8 | data[4];

    if (HW_AK09911 == COMPASS_SLAVE_ID)
        shift = 7;
    else
        shift = 8;

    x = ((x * (sens[0] + 128)) >> shift);
    y = ((y * (sens[1] + 128)) >> shift);
    z = ((z * (sens[2] + 128)) >> shift);

    if (HW_AK8963 == COMPASS_SLAVE_ID)
    {
        result = inv_execute_read_secondary(0, addr, REG_AKM8963_CNTL1, 1, &cntl);

        if (result)
            goto AKM_fail;

        if (0 == (cntl & DATA_AKM8963_BIT))
        {
            x <<= DEF_ST_COMPASS_8963_SHIFT;
            y <<= DEF_ST_COMPASS_8963_SHIFT;
            z <<= DEF_ST_COMPASS_8963_SHIFT;
        }
    }

    result = -EINVAL;

    if (x > st_upper[0] || x < st_lower[0])
        goto AKM_fail;

    if (y > st_upper[1] || y < st_lower[1])
        goto AKM_fail;

    if (z > st_upper[2] || z < st_lower[2])
        goto AKM_fail;

    result = 0;
AKM_fail:

    /*write 0 to ASTC register */
    if ((HW_AK09911 != COMPASS_SLAVE_ID) &&
            (HW_AK09912 != COMPASS_SLAVE_ID) &&
            (HW_AK09916 != COMPASS_SLAVE_ID))
    {
        result |= inv_execute_write_secondary(0, addr, REG_AKM_ST_CTRL, 0);
    }

    /*set to power down mode */
    result |= inv_execute_write_secondary(0, addr, mode, DATA_AKM_MODE_PD);

    return result;
}

/*
 *  inv_write_akm_scale() - Configure the akm scale range.
 */
int inv_write_akm_scale( int data)
{
    char d, en;
    int result;

    if (HW_AK8963 != COMPASS_SLAVE_ID)
        return 0;

    en = !!data;

    if (scale == en)
        return 0;

    d = (DATA_AKM_MODE_SM | (en << DATA_AKM8963_SCALE_SHIFT));

    result = inv_write_single_mems_reg(REG_I2C_SLV1_DO, d);

    if (result)
        return result;

    scale = en;

    return 0;
}

/*
 *  inv_read_akm_scale() - show AKM scale.
 */
int inv_read_akm_scale(int *scale)
{
    if (HW_AK8975 == COMPASS_SLAVE_ID)
        *scale = DATA_AKM8975_SCALE;
    else if (HW_AK8972 == COMPASS_SLAVE_ID)
        *scale = DATA_AKM8972_SCALE;
    else if (HW_AK8963 == COMPASS_SLAVE_ID)
        if (scale)
            *scale = DATA_AKM8963_SCALE1;
        else
            *scale = DATA_AKM8963_SCALE0;
    else if (HW_AK09911 == COMPASS_SLAVE_ID)
        *scale = DATA_AK09911_SCALE;
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
        *scale = DATA_AK09912_SCALE;
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
        *scale = DATA_AK09916_SCALE;
    else
        return -EINVAL;

    return 0;
}

int inv_suspend_akm()
{
    int result;

    if (!secondary_resume_state)
        return 0;

    /* slave 0 is disabled */
    result = inv_mems_secondary_stop_channel(COMPASS_I2C_SLV_READ);
    /* slave 1 is disabled */
    result |= inv_mems_secondary_stop_channel(COMPASS_I2C_SLV_WRITE);

    if (result)
        return result;

    /* Switch off I2C interface only if compass is alone */
    if (inv_mems_pressure_getstate() == 0)
    {
        result |= inv_mems_secondary_disable_i2c();
    }

    secondary_resume_state = 0;

    return result;
}

int inv_resume_akm()
{
    int result;
    uint8_t reg_addr, bytes;
    unsigned char lDataToWrite;

    if (secondary_resume_state)
        return 0;

    /* slave 0 is used to read data from compass */
    /*read mode */

    /* AKM status register address is 1 */
    if (HW_AK09911 == COMPASS_SLAVE_ID)
    {
        if (dmp_on)
        {
            reg_addr = REG_AK09911_DMP_READ;
            bytes = DATA_AKM_99_BYTES_DMP;
        }
        else
        {
            reg_addr = REG_AK09911_STATUS1;
            bytes = DATA_AKM_99_BYTES_DMP - 1;
        }
    }
    else if (HW_AK09912 == COMPASS_SLAVE_ID)
    {
        if (dmp_on)
        {
            reg_addr = REG_AK09912_DMP_READ;
            bytes = DATA_AKM_99_BYTES_DMP;
        }
        else
        {
            reg_addr = REG_AK09912_STATUS1;
            bytes = DATA_AKM_99_BYTES_DMP - 1;
        }
    }
    else if (HW_AK09916 == COMPASS_SLAVE_ID)
    {
        if (dmp_on)
        {
            reg_addr = REG_AK09916_DMP_READ;
            bytes = DATA_AKM_99_BYTES_DMP;
        }
        else
        {
            reg_addr = REG_AK09916_STATUS1;
            bytes = DATA_AKM_99_BYTES_DMP - 1;
        }
    }
    else
    {
        if (dmp_on)
        {
            reg_addr = REG_AKM_INFO;
            bytes = DATA_AKM_89_BYTES_DMP;
        }
        else
        {
            reg_addr = REG_AKM_STATUS;
            bytes = DATA_AKM_89_BYTES_DMP - 1;
        }
    }

    /* slave 0 is enabled, read 10 or 8 bytes from here depending on compass type, swap bytes to feed DMP */
    result = inv_read_secondary(COMPASS_I2C_SLV_READ, COMPASS_CHIP_ADDR, reg_addr, INV_MPU_BIT_GRP | INV_MPU_BIT_BYTE_SW | bytes);

    if (result)
        return result;


    /* slave 1 is used to write one-shot accquisition configuration to compass */
    /* output data for slave 1 is fixed, single measure mode */
    scale = 1;

    if (HW_AK8963 == COMPASS_SLAVE_ID)
    {
        lDataToWrite = DATA_AKM_MODE_SM |
                       (scale << DATA_AKM8963_SCALE_SHIFT);
    }
    else
        lDataToWrite = DATA_AKM_MODE_SM;

    result = inv_write_secondary(COMPASS_I2C_SLV_WRITE, COMPASS_CHIP_ADDR, sModeRegAddr, lDataToWrite);

    if (result)
        return result;

    result |= inv_mems_secondary_enable_i2c();

    secondary_resume_state = 1;

    return result;
}

char inv_mems_compass_getstate(void)
{
    return secondary_resume_state;
}

/**
*  @brief      Set up the soft-iron matrix for compass in DMP.
*  @param[in]  Accel/Gyro mounting matrix
*  @param[in]  Compass mounting matrix
*  @return     0 if successful.
*/

int inv_compass_dmp_cal(const signed char *m, const signed char *compass_m)
{
    int8_t trans[NINE_ELEM];
    int tmp_m[NINE_ELEM];
    int i, j, k;
    int sens[THREE_AXES];
    int scale;
    int shift;
    int current_compass_matrix[NINE_ELEM];

    for (i = 0; i < THREE_AXES; i++)
        for (j = 0; j < THREE_AXES; j++)
            trans[THREE_AXES * j + i] = m[THREE_AXES * i + j];

    switch (COMPASS_SLAVE_ID)
    {
        case HW_AK8972:
            scale = DATA_AKM8972_SCALE;
            shift = AK89XX_SHIFT;
            break;

        case HW_AK8975:
            scale = DATA_AKM8975_SCALE;
            shift = AK89XX_SHIFT;
            break;

        case HW_AK8963:
            scale = DATA_AKM8963_SCALE1;
            shift = AK89XX_SHIFT;
            break;

        case HW_AK09911:
            scale = DATA_AK09911_SCALE;
            shift = AK99XX_SHIFT;
            break;

        case HW_AK09912:
            scale = DATA_AK09912_SCALE;
            shift = AK89XX_SHIFT;
            break;

        case HW_AK09916:
            scale = DATA_AK09916_SCALE;
            shift = AK89XX_SHIFT;
            break;

        default:
            scale = DATA_AKM8963_SCALE1;
            shift = AK89XX_SHIFT;
            break;
    }

    for (i = 0; i < THREE_AXES; i++)
    {
        sens[i] = compass_sens[i] + 128;
        sens[i] = inv_q30_mult(sens[i] << shift, scale);
    }

    for (i = 0; i < NINE_ELEM; i++)
    {
        current_compass_matrix[i] = compass_m[i] * sens[i % THREE_AXES];
        tmp_m[i] = 0;
    }

    for (i = 0; i < THREE_AXES; i++)
    {
        for (j = 0; j < THREE_AXES; j++)
        {
            final_matrix[i * THREE_AXES + j] = 0;

            for (k = 0; k < THREE_AXES; k++)
                final_matrix[i * THREE_AXES + j] +=
                    inv_q30_mult(SOFT_IRON_MATRIX[i * THREE_AXES + k],
                                 current_compass_matrix[j + k * THREE_AXES]);
        }
    }

    for (i = 0; i < THREE_AXES; i++)
        for (j = 0; j < THREE_AXES; j++)
            for (k = 0; k < THREE_AXES; k++)
                tmp_m[THREE_AXES * i + j] +=
                    trans[THREE_AXES * i + k] *
                    final_matrix[THREE_AXES * k + j];

    return dmp_set_compass_matrix(tmp_m);
}

/**
*  @brief      Apply mounting matrix and scaling to raw compass data.
*  @param[in]  Raw compass data
*  @param[in]  Compensated compass data
*  @return     0 if successful.
*/

int inv_apply_raw_compass_matrix(short *raw_data, long *compensated_out)
{
    int i, j;
    long long tmp;

    for (i = 0; i < THREE_AXES; i++)
    {
        tmp = 0;

        for (j = 0; j < THREE_AXES; j++)
            tmp  +=
                (long long)final_matrix[i * THREE_AXES + j] * (((int)raw_data[j]) << 16);

        compensated_out[i] = (long)(tmp >> 30);
    }

    return 0;
}

#endif
