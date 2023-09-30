#include "icm20948.h"
#include "app.h"
#include "bsp.h"
#include "stdio.h"
#include "inv_mems.h"
#include <math.h>
#include "app_math.h"

#define USART_DBG_

#ifdef USART_DBG
    #include "bsp_usart.h"
#endif



static uint8_t init_state = 0;

static char usart_dbg = 0;
static icm20948_data_t i20948_data = {0};



/* add ueler angle */
float        quat[4];
// int accuracy = 0;
float  q01, q02, q03, q11, q12, q13, q22, q23, q33;
float t0, t1, t2, t3;
float values[3];

/* Set bias to DMP rather than offset registers */
// This is only for ICM20648/20948
#define BIAS_SET_TO_DMP

#if (MEMS_CHIP == HW_ICM20609)
    #undef BIAS_SET_TO_DMP
#endif

struct hal_s_ hal = { 0 };

static long long g_ul_ms_ticks = 0;
icm20948_data_t g_icm_data;


/* Every time new gyro data is available, this function is called in an
* ISR context. In this example, it sets a flag protecting the FIFO read
* function.
*/
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
/*******************************************************************************/
#define PRINT_ACCEL         (0x01)
#define PRINT_GYRO          (0x02)
#define PRINT_RAW_GYRO      (0x04)
#define PRINT_COMPASS       (0x08)
#define PRINT_RAW_COMPASS   (0x10)
#define PRINT_RV            (0x20)
#define PRINT_GRV           (0x40)
#define PRINT_ORIENT        (0x80)
#define PRINT_LINEAR_ACCEL  (0x100)
#define PRINT_GRAVITY       (0x200)
#define PRINT_STEP_COUNTER  (0x400)
#define PRINT_STEP_DETECTOR (0x800)
#define PRINT_SMD           (0x1000)
#define PRINT_GEOMAG        (0x2000)
#define PRINT_PRESSURE      (0x8000)
#define PRINT_CUBE_GRV      (0x10000)
#define PRINT_CUBE_RV       (0x20000)
#define PRINT_CUBE_GEOMAG   (0x40000)
#define PRINT_LPQ           (0x80000)
#define PRINT_BAC			(0x100000)
#define PRINT_FLIP_PICKUP	(0x200000)
#define PRINT_TILT			(0x400000)
#define PRINT_PROXIMITY		(0x800000)
#define PRINT_HRM			(0x1000000)
#define PRINT_SHAKE			(0x2000000)
#define PRINT_B2S			(0x4000000)

// for ICM20648
#define PRINT_GES_GROUP	(PRINT_STEP_COUNTER | \
                         PRINT_STEP_DETECTOR | \
                         PRINT_SMD | \
                         PRINT_BAC | \
                         PRINT_FLIP_PICKUP | \
                         PRINT_TILT)

#ifdef MEMS_20609
    #define DMP_INT_SMD		0x0400
    #define DMP_INT_PED		0x0800
#endif

//#ifdef _WIN32
//#define INV_SPRINTF(str, len, ...) sprintf_s(str, len, __VA_ARGS__)
//#else
//#define INV_SPRINTF(str, len, ...) sprintf(str, __VA_ARGS__)
//#endif

int self_test_result = 0;
int dmp_bias[9] = { 0 };
static int self_test_done = 0;
#ifndef BIAS_SET_TO_DMP
    static int a_offset_reg_save[3]; // original accel offset register values
#endif

unsigned short accel_data_was_set = 0;
unsigned short gyro_data_was_set = 0;
unsigned short raw_gyro_data_was_set = 0;
unsigned short compass_data_was_set = 0;
unsigned short raw_compass_data_was_set = 0;
unsigned short quat6_data_was_set = 0;
unsigned short quat9_data_was_set = 0;
unsigned short rv_quat_was_set = 0;
unsigned short gmrv_quat_was_set = 0;
unsigned short bac_data_was_set = 0;
unsigned short flip_data_was_set = 0;
unsigned short tilt_data_was_set = 0;

float grv_float[4];
signed long cube_grv[4] = { 0, 0, 0, 0 };
float accel_float[3];
float rv_float[4];
signed long cube_rv[4] = { 0, 0, 0, 0 };
float gyro_float[3];
float gyro_raw_float[3];
float gyro_bias_float[3];
long previous_bias[3];
float compass_float[3];
float compass_raw_float[3];
float compass_bias[3];
#if (MEMS_CHIP == HW_ICM20648)
float gmrv_float[4];
signed long cube_gmrv[4] = { 0, 0, 0, 0 };
#endif
int accel_accuracy = 0;
int gyro_accuracy = 0;
int rv_accuracy = 0;
int compass_accuracy = 0;
#if (MEMS_CHIP == HW_ICM20648)
    int gmrv_accuracy = 0;
#endif

long long ts = 0;

signed long result_quat[4] = { 0, 0, 0, 0 };

int a_average[3] = { 0, 0, 0 };
int g_average[3] = { 0, 0, 0 };
#if defined MEMS_AUGMENTED_SENSORS
    float linAccFloat[3];
    float gravityFloat[3];
    float orientationFloat[3];
#endif

unsigned long steps = 0;
uint16_t bac_state = 0;
uint8_t tilt_state = 0;
long bac_ts = 0;
uint16_t flip_pickup = 0;
uint8_t step_detected = 0;
float current_output_rate = 5;

/** @brief Set of flags for BAC state */
#define BAC_DRIVE	0x01
#define BAC_WALK	0x02
#define BAC_RUN		0x04
#define BAC_BIKE	0x08
#define BAC_TILT	0x10
#define BAC_STILL	0x20

/* Change ACCEL_GYRO_CHIP_ADDR if necessary */
const unsigned char ACCEL_GYRO_CHIP_ADDR = 0x68;

/* Change ACCEL_GYRO_ORIENTATION according to actual mount matrix */
signed char ACCEL_GYRO_ORIENTATION[] = {1, 0, 0,
                                        0, -1, 0,
                                        0, 0, -1
                                       };

/* Change the COMPASS_SLAVE_ID to the correct ID of compass used. You can find the defines in inv_mems_hw_config.h*/
const unsigned char COMPASS_SLAVE_ID = HW_AK09916;

/* Change COMPASS_CHIP_ADDR to 0x0C for ICM20948 which uses internal AK09916 */
/* Change COMPASS_CHIP_ADDR to 0x0E for other AK09912/09911/09913/8963 */
const unsigned char COMPASS_CHIP_ADDR = 0x0C;
const unsigned char PRESSURE_CHIP_ADDR = 0x00;

/* Change COMPASS_ORIENTATION according to actual mount matrix */
signed char COMPASS_ORIENTATION[] = {0, -1, 0, 1, 0, 0, 0, 0, 1};

/* Change SOFT_IRON_MATRIX if necessary (q30) */
long SOFT_IRON_MATRIX[] = {1073741824, 0, 0, 0, 1073741824, 0, 0, 0, 1073741824};

#define INV_TST_LEN 256
char tst[INV_TST_LEN] = { 0 };

#define MAX_BUF_LENGTH  (18)

enum packet_type_e
{
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_COMPASS = 7
};


//////////////////////////////////////////////////////////////////////////////////
//内部函数声明

void GPIO_INT_Config(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd(INVEN_INT_GPIO_CLK, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Configure invensense sensor interrupt pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = INVEN_INT_PIN;
    GPIO_Init(INVEN_INT_GPIO_PORT, &GPIO_InitStructure);

    /* Connect EXTI Line to inv sensor interrupt pin */
    GPIO_EXTILineConfig(INVEN_INT_EXTI_PORT, INVEN_INT_EXTI_PIN);

    /* Configure EXTI Line1 */
    EXTI_InitStructure.EXTI_Line = INVEN_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line Interrupt to the highest priority */
    NVIC_InitStructure.NVIC_IRQChannel = INVEN_INT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


inv_error_t set_output_rates(float rate)
{
    #if (MEMS_CHIP == HW_ICM20609)
    unsigned short base_rate = 200; //DMP runs at 200Hz always
    #elif (MEMS_CHIP == HW_ICM20648)
    unsigned short base_rate = 1125;
    #endif
    inv_error_t result = 0;

    #if (MEMS_CHIP == HW_ICM20648)

    if (hal.report & PRINT_GES_GROUP)
    {
        // Set 56Hz x N to feed 56Hz data to algo in DMP
        if (rate >= 225)
            rate = 225;
        else if (rate >= 102)
            rate = 112.5;
        else if (rate >= 51)
            rate = 56.5;
    }

    #endif

    if (hal.report & PRINT_ACCEL)
        result |= inv_set_odr(ANDROID_SENSOR_ACCELEROMETER, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_GYRO)
        result |= inv_set_odr(ANDROID_SENSOR_GYROSCOPE, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_RAW_GYRO)
        result |= inv_set_odr(ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_GRV)
        result |= inv_set_odr(ANDROID_SENSOR_GAME_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_CUBE_GRV)
        result |= inv_set_odr(ANDROID_SENSOR_GAME_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_STEP_DETECTOR)
        result |= inv_set_odr(ANDROID_SENSOR_STEP_DETECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_SMD)
        result |= inv_set_odr(ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_STEP_COUNTER)
        result |= inv_set_odr(ANDROID_SENSOR_STEP_COUNTER, (unsigned short)(base_rate / rate));

    #if (MEMS_CHIP == HW_ICM20648)

    if (hal.report & PRINT_LINEAR_ACCEL)
        result |= inv_set_odr(ANDROID_SENSOR_LINEAR_ACCELERATION, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_GRAVITY)
        result |= inv_set_odr(ANDROID_SENSOR_GRAVITY, (unsigned short)(base_rate / rate));

    if ((hal.report & PRINT_COMPASS) || (hal.report & PRINT_RAW_COMPASS))
        result |= inv_set_odr(ANDROID_SENSOR_GEOMAGNETIC_FIELD, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_RAW_COMPASS)
        result |= inv_set_odr(ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_RV)
        result |= inv_set_odr(ANDROID_SENSOR_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_GEOMAG)
        result |= inv_set_odr(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_BAC)
        result |= inv_set_odr(ANDROID_SENSOR_ACTIVITY_CLASSIFICATON, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_TILT)
        result |= inv_set_odr(ANDROID_SENSOR_WAKEUP_TILT_DETECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_FLIP_PICKUP)
        result |= inv_set_odr(ANDROID_SENSOR_FLIP_PICKUP, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_CUBE_RV)
        result |= inv_set_odr(ANDROID_SENSOR_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_CUBE_GEOMAG)
        result |= inv_set_odr(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, (unsigned short)(base_rate / rate));

    if (hal.report & PRINT_ORIENT)
        result |= inv_set_odr(ANDROID_SENSOR_ORIENTATION, (unsigned short)(base_rate / rate));

    #endif
    inv_reset_dmp_odr_counters();
    dmp_reset_fifo();
    return result;
}

#ifndef BIAS_SET_TO_DMP
/* Reset offset registers.
   This have to be done before running self-test
   if the registers are modified already. */
static int reset_offset_regs(void)
{
    int ret;
    uint8_t d[2];

    /* reset accel offset registers */
    if (self_test_done == 0)
    {
        /* save accel offset register values */
        if(usart_dbg)printf("Save the original accel offset register values\r\n");
        ret = inv_read_mems_reg(REG_XA_OFFS_H, 2, d);
        a_offset_reg_save[0] = (int)((d[0] << 8) | d[1]); // ax
        ret |= inv_read_mems_reg(REG_YA_OFFS_H, 2, d);
        a_offset_reg_save[1] = (int)((d[0] << 8) | d[1]); // ay
        ret |= inv_read_mems_reg(REG_ZA_OFFS_H, 2, d);
        a_offset_reg_save[2] = (int)((d[0] << 8) | d[1]); // az

        if (ret)
        {
            if(usart_dbg)printf("Failed to read accel offset registers\r\n");
        }
    }
    else
    {
        /* restore accel offset registers to the original */
        if(usart_dbg)printf("Restore the original accel offset register values\r\n");
        d[0] = (a_offset_reg_save[0] >> 8) & 0xff;
        d[1] = a_offset_reg_save[0] & 0xff;
        ret = inv_write_single_mems_reg(REG_XA_OFFS_H, d[0]);
        ret |= inv_write_single_mems_reg(REG_XA_OFFS_H + 1, d[1]);
        d[0] = (a_offset_reg_save[1] >> 8) & 0xff;
        d[1] = a_offset_reg_save[1] & 0xff;
        ret |= inv_write_single_mems_reg(REG_YA_OFFS_H, d[0]);
        ret |= inv_write_single_mems_reg(REG_YA_OFFS_H + 1, d[1]);
        d[0] = (a_offset_reg_save[2] >> 8) & 0xff;
        d[1] = a_offset_reg_save[2] & 0xff;
        ret |= inv_write_single_mems_reg(REG_ZA_OFFS_H, d[0]);
        ret |= inv_write_single_mems_reg(REG_ZA_OFFS_H + 1, d[1]);

        if (ret)
        {
            if(usart_dbg)printf("Failed to reset accel offset registers\r\n");
        }
    }

    /* reset gyro offset registers */
    if(usart_dbg)printf("Reset gyro offset register values\r\n");
    d[0] = d[1] = 0;
    ret = inv_write_single_mems_reg(REG_XG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_XG_OFFS_USR_H + 1, d[1]);
    ret |= inv_write_single_mems_reg(REG_YG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_YG_OFFS_USR_H + 1, d[1]);
    ret |= inv_write_single_mems_reg(REG_ZG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_ZG_OFFS_USR_H + 1, d[1]);

    if (ret)
    {
        if(usart_dbg)printf("Failed to reset gyro offset registers\r\n");
    }

    return ret;
}

/* Set offset registers.
   accel_off[3] : accel offset (LSB @ 2g)
   gyro_off[3]  : gyro  offset (LSB @ 250dps) */
static int set_offset_regs(int accel_off[3], int gyro_off[3])
{
    int i, ret;
    uint8_t d[3];
    int bias[3];

    /* Accel offset registers */

    /* accel bias from self-test is 2g
       convert to 16g and mask bit0 */
    for (i = 0; i < 3; i++)
    {
        bias[i] = a_offset_reg_save[i] - (accel_off[i] >> 3);
        bias[i] &= ~1;
    }

    d[0] = (bias[0] >> 8) & 0xff;
    d[1] = bias[0] & 0xff;
    ret = inv_write_single_mems_reg(REG_XA_OFFS_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_XA_OFFS_H + 1, d[1]);
    d[0] = (bias[1] >> 8) & 0xff;
    d[1] = bias[1] & 0xff;
    ret |= inv_write_single_mems_reg(REG_YA_OFFS_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_YA_OFFS_H + 1, d[1]);
    d[0] = (bias[2] >> 8) & 0xff;
    d[1] = bias[2] & 0xff;
    ret |= inv_write_single_mems_reg(REG_ZA_OFFS_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_ZA_OFFS_H + 1, d[1]);

    if (ret)
    {
        if(usart_dbg)printf("Failed to write accel offset registers\r\n");
    }

    /* Gyro offset registers */

    /* gyro bias from self-test is 250dps
       convert to 1000dps */
    for (i = 0; i < 3; i++)
        bias[i] = -(gyro_off[i] >> 2);

    d[0] = (bias[0] >> 8) & 0xff;
    d[1] = bias[0] & 0xff;
    ret = inv_write_single_mems_reg(REG_XG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_XG_OFFS_USR_H + 1, d[1]);
    d[0] = (bias[1] >> 8) & 0xff;
    d[1] = bias[1] & 0xff;
    ret |= inv_write_single_mems_reg(REG_YG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_YG_OFFS_USR_H + 1, d[1]);
    d[0] = (bias[2] >> 8) & 0xff;
    d[1] = bias[2] & 0xff;
    ret |= inv_write_single_mems_reg(REG_ZG_OFFS_USR_H, d[0]);
    ret |= inv_write_single_mems_reg(REG_ZG_OFFS_USR_H + 1, d[1]);

    if (ret)
    {
        if(usart_dbg)printf("Failed to write gyro offset registers\r\n");
    }

    return ret;
}
#endif // !BIAS_SET_TO_DMP

int handle_char_input(char c)
{
    int scale =  0;

    switch (c)
    {
        case 'a':
            dmp_reset_fifo();
            hal.report ^= PRINT_ACCEL;
            inv_enable_sensor(ANDROID_SENSOR_ACCELEROMETER, !!(hal.report & PRINT_ACCEL));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("ACCEL....output toggled, now is: %s.\r\n", (hal.report & PRINT_ACCEL) ? "ON" : "OFF");
            break;

        case 'g':
            dmp_reset_fifo();
            hal.report ^= PRINT_GYRO;
            inv_enable_sensor(ANDROID_SENSOR_GYROSCOPE, !!(hal.report & PRINT_GYRO));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("GYRO....output toggled, now is: %s.\r\n", (hal.report & PRINT_GYRO) ? "ON" : "OFF");
            break;

        case 'G':
            dmp_reset_fifo();
            hal.report ^= PRINT_RAW_GYRO;
            inv_enable_sensor(ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED, !!(hal.report & PRINT_RAW_GYRO));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("RAW GYRO....output toggled, now is: %s.\r\n", (hal.report & PRINT_RAW_GYRO) ? "ON" : "OFF");
            break;
            #if (MEMS_CHIP != HW_ICM20609)

        case 'c':
            dmp_reset_fifo();
            hal.report ^= PRINT_COMPASS;

            if ((hal.report & PRINT_RAW_COMPASS) || (hal.report & PRINT_COMPASS))
                inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 1);
            else
                inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 0);

            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Compass....output toggled, now is: %s.\r\n", (hal.report & PRINT_COMPASS) ? "ON" : "OFF");
            break;

        case 'C':
            dmp_reset_fifo();
            hal.report ^= PRINT_RAW_COMPASS;
            inv_enable_sensor(ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, !!(hal.report & PRINT_RAW_COMPASS));

            if ((hal.report & PRINT_RAW_COMPASS) || (hal.report & PRINT_COMPASS))
                inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 1); // to calculate bias in application layer
            else
                inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 0);

            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Raw Compass....output toggled, now is: %s.\r\n", (hal.report & PRINT_RAW_COMPASS) ? "ON" : "OFF");
            break;
            #endif // !HW_ICM20609
            #if 0

        case 'r':
            dmp_reset_fifo();
            hal.report ^= PRINT_GRV;
            inv_enable_sensor(ANDROID_SENSOR_GAME_ROTATION_VECTOR, !!(hal.report & PRINT_GRV));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Game RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_GRV) ? "ON" : "OFF");

            break;
            #endif

        case 'q':
            dmp_reset_fifo();
            hal.report ^= PRINT_CUBE_GRV;
            inv_enable_sensor(ANDROID_SENSOR_GAME_ROTATION_VECTOR, !!(hal.report & PRINT_CUBE_GRV));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Cube Game RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_CUBE_GRV) ? "ON" : "OFF");

            break;

        case 'd':
            dmp_reset_fifo();
            hal.report ^= PRINT_STEP_DETECTOR;
            inv_enable_sensor(ANDROID_SENSOR_STEP_DETECTOR, !!(hal.report & PRINT_STEP_DETECTOR));
            inv_reset_dmp_odr_counters();
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(current_output_rate);
            #elif (MEMS_CHIP == HW_ICM20630)
            set_output_rates(current_output_rate);
            #else
            // pedometer always runs at half the rate of BAC, to run pedometer at 56Hz, run BAC at 112Hz as pedometer divider is always 2
            set_output_rates(current_output_rate);
            #endif
            if(usart_dbg)printf("Step Detector....output toggled, now is: %s.\r\n", (hal.report & PRINT_STEP_DETECTOR) ? "ON" : "OFF");

            break;

        case 'm':
            dmp_reset_fifo();
            hal.report ^= PRINT_SMD;
            inv_enable_sensor(ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION, !!(hal.report & PRINT_SMD));
            inv_reset_dmp_odr_counters();
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(current_output_rate);
            #else
            // pedometer always runs at half the rate of BAC, to run pedometer at 56Hz, run BAC at 112Hz as pedometer divider is always 2
            set_output_rates(current_output_rate);
            #endif
            if(usart_dbg)printf("SMD....output toggled, now is: %s.\r\n", (hal.report & PRINT_SMD) ? "ON" : "OFF");

            break;

        case 'p':
            dmp_reset_fifo();
            hal.report ^= PRINT_STEP_COUNTER;
            inv_enable_sensor(ANDROID_SENSOR_STEP_COUNTER, !!(hal.report & PRINT_STEP_COUNTER));
            inv_reset_dmp_odr_counters();
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(current_output_rate);
            #else
            // pedometer always runs at half the rate of BAC, to run pedometer at 56Hz, run BAC at 112Hz as pedometer divider is always 2
            set_output_rates(current_output_rate);
            #endif
            if(usart_dbg)printf("Step Counter....output toggled, now is: %s.\r\n", (hal.report & PRINT_STEP_COUNTER) ? "ON" : "OFF");

            break;
            #if (MEMS_CHIP != HW_ICM20609)

        case 'R':
            dmp_reset_fifo();
            hal.report ^= PRINT_RV;
            inv_enable_sensor(ANDROID_SENSOR_ROTATION_VECTOR, !!(hal.report & PRINT_RV));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_RV) ? "ON" : "OFF");

            break;

        case 'e':
            dmp_reset_fifo();
            hal.report ^= PRINT_GEOMAG;
            inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, !!(hal.report & PRINT_GEOMAG));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("GeoMAG RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_GEOMAG) ? "ON" : "OFF");

            break;
            #if (MEMS_CHIP != HW_ICM20630)

        case 'b':
            dmp_reset_fifo();

            if (!(hal.report & PRINT_BAC))
                dmp_reset_bac_states();

            hal.report ^= PRINT_BAC;
            inv_enable_sensor(ANDROID_SENSOR_ACTIVITY_CLASSIFICATON, !!(hal.report & PRINT_BAC));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("BAC....output toggled, now is: %s.\r\n", (hal.report & PRINT_BAC) ? "ON" : "OFF");
            break;

        case 'T':
            dmp_reset_fifo();
            hal.report ^= PRINT_TILT;
            inv_enable_sensor(ANDROID_SENSOR_WAKEUP_TILT_DETECTOR, !!(hal.report & PRINT_TILT));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Tilt....output toggled, now is: %s.\r\n", (hal.report & PRINT_TILT) ? "ON" : "OFF");
            break;

        case 'F':
            dmp_reset_fifo();
            hal.report ^= PRINT_FLIP_PICKUP;
            inv_enable_sensor(ANDROID_SENSOR_FLIP_PICKUP, !!(hal.report & PRINT_FLIP_PICKUP));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Flip Pickup....output toggled, now is: %s.\r\n", (hal.report & PRINT_FLIP_PICKUP) ? "ON" : "OFF");
            break;
            #endif // !HW_ICM20630
            #endif // !HW_ICM20609

        case '0':
            set_output_rates(1);
            current_output_rate = 1;
            break;

        case '1':
            set_output_rates(5);
            current_output_rate = 5;
            break;

        case '2':
            set_output_rates(15);
            current_output_rate = 15;
            break;

        case '3':
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(50);
            current_output_rate = 50;
            #elif (MEMS_CHIP == HW_ICM20648)
            set_output_rates(30);
            current_output_rate = 30;
            #endif
            break;

        case '4':
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(100);
            current_output_rate = 100;
            #elif (MEMS_CHIP == HW_ICM20648)
            set_output_rates(51);
            current_output_rate = 51;
            #endif
            break;

        case '5':
            #if (MEMS_CHIP == HW_ICM20609)
            set_output_rates(200);
            current_output_rate = 200;
            #elif (MEMS_CHIP == HW_ICM20648)
            set_output_rates(56.5);
            current_output_rate = 56.5;
            break;
            #endif
            #if (MEMS_CHIP == HW_ICM20648)

        case '6':
            set_output_rates(60);
            current_output_rate = 60;
            break;

        case '7':
            set_output_rates(102);
            current_output_rate = 102;
            break;

        case '8':
            set_output_rates(112.5);
            current_output_rate = 112.5;
            break;

        case '9':
            set_output_rates(225);
            current_output_rate = 225;
            break;

        case 'Q':
            dmp_reset_fifo();
            hal.report ^= PRINT_CUBE_RV;
            inv_enable_sensor(ANDROID_SENSOR_ROTATION_VECTOR, !!(hal.report & PRINT_CUBE_RV));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Cube RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_CUBE_RV) ? "ON" : "OFF");

            break;

        case '@':
            dmp_reset_fifo();
            hal.report ^= PRINT_CUBE_GEOMAG;
            inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, !!(hal.report & PRINT_CUBE_GEOMAG));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Cube GeoMAG RV....output toggled, now is: %s.\r\n", (hal.report & PRINT_CUBE_GEOMAG) ? "ON" : "OFF");

            break;
            #endif // HW_ICM20648

        case 'v':
            dmp_reset_fifo();
            hal.report ^= PRINT_GRAVITY;
            inv_enable_sensor(ANDROID_SENSOR_GRAVITY, !!(hal.report & PRINT_GRAVITY));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Gravity....output toggled, now is: %s.\r\n", (hal.report & PRINT_GRAVITY) ? "ON" : "OFF");

            break;

        case 'l':
            dmp_reset_fifo();
            hal.report ^= PRINT_LINEAR_ACCEL;
            inv_enable_sensor(ANDROID_SENSOR_LINEAR_ACCELERATION, !!(hal.report & PRINT_LINEAR_ACCEL));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Linear Accel....output toggled, now is: %s.\r\n", (hal.report & PRINT_LINEAR_ACCEL) ? "ON" : "OFF");

            break;
            #if (MEMS_CHIP != HW_ICM20609)

        case 'o':
            #if 1
            dmp_reset_fifo();
            hal.report ^= PRINT_GRV;
            inv_enable_sensor(ANDROID_SENSOR_GAME_ROTATION_VECTOR, !!(hal.report & PRINT_GRV));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Ueler....output toggled, now is: %s.\r\n", (hal.report & PRINT_GRV) ? "ON" : "OFF");

            break;
            #else

            dmp_reset_fifo();
            hal.report ^= PRINT_ORIENT;
            inv_enable_sensor(ANDROID_SENSOR_ORIENTATION, !!(hal.report & PRINT_ORIENT));
            inv_reset_dmp_odr_counters();
            set_output_rates(current_output_rate);
            if(usart_dbg)printf("Orientation....output toggled, now is: %s.\r\n", (hal.report & PRINT_ORIENT) ? "ON" : "OFF");

            break;
            #endif
            #endif

        case 'k':
            if(usart_dbg)printf("Getting the DMP biases...\r\n");
            memset(dmp_bias, 0, sizeof(dmp_bias));
            dmp_get_bias(dmp_bias);
            #if (MEMS_CHIP != HW_ICM20609)
            if(usart_dbg)printf("DMP   Accel Bias: X=%d, Y=%d, Z=%d\r\n", dmp_bias[0], dmp_bias[1], dmp_bias[2]);
            #endif
            if(usart_dbg)printf("DMP   Gyro  Bias: X=%d, Y=%d, Z=%d\r\n", dmp_bias[3], dmp_bias[4], dmp_bias[5]);
            break;

        case 't':
            if (hal.report)
            {
                if(usart_dbg)printf("Disable all sensors before running self-test\r\n");
                break;
            }

            {
                //uint8_t d[2];
                //int ret, i;
                char pass_str[] = { "PASS" };
                char fail_str[] = { "FAIL" };
                #ifndef BIAS_SET_TO_DMP
                int accel_offset[3], gyro_offset[3];
                int ret, i;
                #endif

                if(usart_dbg)printf("Selftest...Started\r\n");


                #ifndef BIAS_SET_TO_DMP
                /* Reset offset registers before running self-test */
                ret = reset_offset_regs();

                if (ret)
                {
                    if(usart_dbg)printf("Failed to reset offset registers\r\n");
                }

                #endif

                /* Perform self-test */
                self_test_result = inv_mems_run_selftest();
                if(usart_dbg)printf("Selftest...Done...Ret=%d\r\n", self_test_result);

                if(usart_dbg)printf("Result: Compass=%s, Accel=%s, Gyro=%s\r\n", (self_test_result & 0x04) ? pass_str : fail_str, (self_test_result & 0x02) ? pass_str : fail_str, (self_test_result & 0x01) ? pass_str : fail_str);

                if(usart_dbg)printf("Accel Average (LSB@FSR 2g)\r\n");

                if(usart_dbg)printf("\tX:%d Y:%d Z:%d\r\n", a_average[0], a_average[1], a_average[2]);

                if(usart_dbg)printf("Gyro Average (LSB@FSR 250dps)\r\n");

                if(usart_dbg)printf("\tX:%d Y:%d Z:%d\r\n", g_average[0], g_average[1], g_average[2]);


                /* Nothing to do if FAIL on gyro and accel */
                if ((self_test_result & 0x03) != 0x03)
                    break;

                /* Handle bias got by self-test */
                #ifdef BIAS_SET_TO_DMP // Set to DMP
                // for ICM20648/20948
                dmp_bias[0] = a_average[0] * (1 << 11);   // Change from LSB to format expected by DMP
                dmp_bias[1] = a_average[1] * (1 << 11);
                dmp_bias[2] = (a_average[2] - 16384) * (1 << 11); //remove the gravity and scale (FSR=2 in selftest)
                scale = 2000 / 250; 						//self-test uses 250dps FSR, main() set the FSR to 2000dps
                dmp_bias[3] = g_average[0] * (1 << 15) / scale;
                dmp_bias[4] = g_average[1] * (1 << 15) / scale;
                dmp_bias[5] = g_average[2] * (1 << 15) / scale;

                if(usart_dbg)printf("Factory Cal - Accel DMP biases: \tX:%d Y:%d Z:%d\r\n", dmp_bias[0], dmp_bias[1], dmp_bias[2]);

                if(usart_dbg)printf("Factory Cal - Gyro DMP biases:  \tX:%d Y:%d Z:%d\r\n", dmp_bias[3], dmp_bias[4], dmp_bias[5]);

                /* Update bias on DMP memory */
                dmp_set_bias(dmp_bias);
                if(usart_dbg)printf("\r\nSetting the DMP biases with one-axis factory calibration values...done\r\n");

                #else // Set to offset registers

                // for ICM20648/20948/20608D/20609/20689
                for (i = 0; i < 3; i++)
                {
                    if (i == 2)
                    {
                        // assume the device is put horizontal and z axis shows 1g during self-test.
                        // self-test uses 2g FSR and 1g = 16384LSB
                        accel_offset[i] = a_average[i] - 16384;
                    }
                    else
                    {
                        accel_offset[i] = a_average[i];
                    }

                    // gyro FSR is 250dps for self-test
                    gyro_offset[i] = g_average[i];
                }

                /* Update offset registers */
                ret = set_offset_regs(accel_offset, gyro_offset);

                if (ret)
                {
                    if(usart_dbg)printf("Failed to update offset registers\r\n");
                }
                else
                {
                    if(usart_dbg)printf("\r\nSetting the offset registers with one-axis factory calibration values...done\r\n");
                }

                #endif // BIAS_SET_TO_DMP
                self_test_done = 1;
            }

            break;

        //case SHOW_COMMANDS:
        default:
            if(usart_dbg)printf("\r\n");
            if(usart_dbg)printf("Press 'a' to toggle ACCEL output....................\r\n");
            if(usart_dbg)printf("Press 'g' to toggle Gyro output.....................\r\n");
            if(usart_dbg)printf("Press 'G' to toggle Raw Gyro output.................\r\n");
            if(usart_dbg)printf("Press 'p' to toggle Step Counter output.............\r\n");
            if(usart_dbg)printf("Press 'd' to toggle Step Detector output............\r\n");
            if(usart_dbg)printf("Press 'm' to toggle SMD Significant Motion output...\r\n");
            #if (MEMS_CHIP != HW_ICM20609)
            if(usart_dbg)printf("Press 'c' to toggle Compass output..................\r\n");
            if(usart_dbg)printf("Press 'C' to toggle Raw Compass output..............\r\n");
            #endif
            #if (MEMS_CHIP != HW_ICM20630)
            #if (MEMS_CHIP != HW_ICM20609)
            if(usart_dbg)printf("Press 'b' to toggle Basic Activity Classifier output...\r\n");
            if(usart_dbg)printf("Press 'T' to toggle Tilt output...\r\n");
            if(usart_dbg)printf("Press 'F' to toggle Flip Pickup output...\r\n");
            #endif // !HW_ICM20609
            #endif // !HW_ICM20630
            if(usart_dbg)printf("Press 'r' to toggle Game Rotation Vector output.....\r\n");
            #if (MEMS_CHIP != HW_ICM20609)
            if(usart_dbg)printf("Press 'R' to toggle Rotation Vector output..........\r\n");
            if(usart_dbg)printf("Press 'e' to toggle GeoMag Vector output............\r\n");
            #if defined MEMS_AUGMENTED_SENSORS
            if(usart_dbg)printf("Press 'l' to toggle Linear Acceleration output............\r\n");
            if(usart_dbg)printf("Press 'v' to toggle Gravity output............\r\n");
            if(usart_dbg)printf("Press 'o' to toggle Orientation output............\r\n");
            #endif // MEMS_AUGMENTED_SENSORS
            #endif // !HW_ICM20609
            if(usart_dbg)printf("Press 'k' to get the DMP Biases.................\r\n");
            if(usart_dbg)printf("Press 't' to invoke Self test.......................\r\n");
            if(usart_dbg)printf("Press '0' to set ODR @ 1Hz..........................\r\n");
            if(usart_dbg)printf("Press '1' to set ODR @ 5Hz..........................\r\n");
            if(usart_dbg)printf("Press '2' to set ODR @ 15Hz.........................\r\n");
            #if (MEMS_CHIP == HW_ICM20609)
            if(usart_dbg)printf("Press '3' to set ODR @ 50Hz.........................\r\n");
            if(usart_dbg)printf("Press '4' to set ODR @ 100Hz.........................\r\n");
            if(usart_dbg)printf("Press '5' to set ODR @ 200Hz.........................\r\n");
            #endif
            #if (MEMS_CHIP != HW_ICM20609)
            if(usart_dbg)printf("Press '3' to set ODR @ 30Hz.........................\r\n");
            if(usart_dbg)printf("Press '4' to set ODR @ 51Hz.........................\r\n");
            if(usart_dbg)printf("Press '5' to set ODR @ 56Hz.........................\r\n");
            if(usart_dbg)printf("Press '6' to set ODR @ 60Hz.........................\r\n");
            if(usart_dbg)printf("Press '7' to set ODR @ 102Hz.........................\r\n");
            if(usart_dbg)printf("Press '8' to set ODR @ 112Hz.........................\r\n");
            if(usart_dbg)printf("Press '9' to set ODR @ 225Hz.........................\r\n");
            if(usart_dbg)printf("Press 'Q' to toggle Cube Rotation Vector output........\r\n");
            if(usart_dbg)printf("Press '@' to toggle Cube GeoMAG Vector output..........\r\n");
            #endif
            if(usart_dbg)printf("Press 'q' to toggle Cube Game Rotation Vector output...\r\n");
            if(usart_dbg)printf("\r\n");
            break;
    }

    return 1;
}

// 取出数据
void process_sensor_output()
{
    signed long  long_quat[3] = { 0 };

    if (hal.report & PRINT_ACCEL)
    {
        if (accel_data_was_set == 1)
        {
            if (self_test_done && (accel_accuracy == 0)) //self-test is done already
                accel_accuracy = 1; //accuracy reaches intermediate level after one-axis factory cal--yd

            if(usart_dbg)printf("Accel Data\t %8.5f, %8.5f, %8.5f, %d, %lld\r\n", accel_float[0], accel_float[1], accel_float[2], accel_accuracy, ts);
			memcpy(i20948_data.accel_float,accel_float,sizeof(accel_float));
            accel_data_was_set = 0;
        }
    }

    if (hal.report & PRINT_GYRO)
    {
        if (gyro_data_was_set == 1)
        {
            if (self_test_done && (gyro_accuracy == 0)) //self-test is done already
                gyro_accuracy = 1; //accuracy reaches intermediate level after one-axis factory cal--yd

            if(usart_dbg)printf("Gyro Data\t %7.5f, %7.5f, %7.5f, %d, %lld\r\n", gyro_float[0], gyro_float[1], gyro_float[2], gyro_accuracy, ts);
			memcpy(i20948_data.gyro_float,gyro_float,sizeof(gyro_float));
            gyro_data_was_set = 0;
        }
    }

    if (hal.report & PRINT_RAW_GYRO)
    {
        #if ((MEMS_CHIP == HW_ICM30630) || (MEMS_CHIP == HW_ICM20648) || (MEMS_CHIP == HW_ICM20609))

        if (raw_gyro_data_was_set == 1)
        {
            if(usart_dbg)printf("Raw Gyro Data\t %7.5f, %7.5f, %7.5f,%d, %lld\r\n", gyro_raw_float[0], gyro_raw_float[1], gyro_raw_float[2], 0, ts);
            if(usart_dbg)printf("Gyro Bias\t %7.5f, %7.5f, %7.5f\r\n", gyro_bias_float[0], gyro_bias_float[1], gyro_bias_float[2]);
            raw_gyro_data_was_set = 0;
        }

        #endif
    }

    if (quat6_data_was_set == 1)
    {
        if (hal.report & PRINT_GRV)
        {
            quat[0] = grv_float[0];
            quat[1] = grv_float[1];
            quat[2] = grv_float[2];
            quat[3] = grv_float[3];

            //accuracy = (float)rv_accuracy / (1 << 16);
            t0 = quat[0];
            t1 = quat[1];
            t2 = quat[2];
            t3 = quat[3];

            q01 = t0 * t1;
            q02 = t0 * t2;
            q03 = t0 * t3;
            q11 = t1 * t1;
            q12 = t1 * t2;
            q13 = t1 * t3;
            q22 = t2 * t2;
            q23 = t2 * t3;
            q33 = t3 * t3;

            values[0] = -asin(2 * (q02 - q13));                          // Roll 翻滚角
            values[1] = atan2(2 * (q03 + q12), (1 - 2 * (q22 + q33)));   // Pitch 俯仰角
            values[2] = atan2(2 * (q01 + q23), (1 - 2 * (q11 + q22)));   // Yaw 偏航角

            if(usart_dbg)printf("Orientation\t x=%7.5f, y=%7.5f, z=%7.5f\r\n", values[0], values[1], values[2]);
			memcpy(i20948_data.orientation,values,sizeof(values));
            //if(usart_dbg)printf("Game RV\t %7.5f, %7.5f, %7.5f, %7.5f, %lld\r\n", grv_float[0], grv_float[1], grv_float[2], grv_float[3], ts);
        }

        if (hal.report & PRINT_CUBE_GRV)
        {
            dmp_get_6quaternion(long_quat);
            inv_compute_scalar_part(long_quat, result_quat);
            //	send_data_packet(PACKET_TYPE_QUAT, (void *)result_quat);
        }

        #if (MEMS_CHIP != HW_ICM20609)
        #if defined MEMS_AUGMENTED_SENSORS

        if (hal.report & PRINT_GRAVITY)
        {
            if(usart_dbg)printf("Gravity\t %7.5f, %7.5f, %7.5f, %lld\r\n", gravityFloat[0], gravityFloat[1], gravityFloat[2], ts);

        }

        if (hal.report & PRINT_LINEAR_ACCEL)
        {
            if(usart_dbg)printf("LinearAcc\t %7.5f, %7.5f, %7.5f, %lld\r\n", linAccFloat[0], linAccFloat[1], linAccFloat[2], ts);

        }

        #endif // MEMS_AUGMENTED_SENSORS
        #endif // !HW_ICM20609
        quat6_data_was_set = 0;
    }

    #if (MEMS_CHIP != HW_ICM20609)

    if (hal.report & PRINT_COMPASS)
    {
        if (compass_data_was_set == 1)
        {
            if(usart_dbg)printf("Compass Data\t %7.5f, %7.5f, %7.5f,\t%d, %lld\r\n", compass_float[0], compass_float[1], compass_float[2], compass_accuracy, ts);
			memcpy(i20948_data.compass_float,compass_float,sizeof(compass_float));
            compass_data_was_set = 0;
        }
    }

    if (hal.report & PRINT_RAW_COMPASS)
    {
        if (raw_compass_data_was_set == 1)
        {
            if(usart_dbg)printf("Raw Compass Data\t %7.5f, %7.5f, %7.5f,\t%d, %lld\r\n", compass_raw_float[0], compass_raw_float[1], compass_raw_float[2], 0, ts);
            if(usart_dbg)printf("Compass Bias\t %7.3f, %7.3f, %7.3f\r\n", compass_bias[0], compass_bias[1], compass_bias[2]);
            raw_compass_data_was_set = 0;
        }
    }

    if (quat9_data_was_set == 1)
    {
        if (hal.report & PRINT_RV)
        {
            if(usart_dbg)printf("RV\t %7.5f, %7.5f, %7.5f, %7.5f, %d, %lld\r\n", rv_float[0], rv_float[1], rv_float[2], rv_float[3], rv_accuracy, ts);

        }

        if (hal.report &  PRINT_CUBE_RV)
        {
            long temp_long_quat[3];
            dmp_get_9quaternion(long_quat);
            inv_convert_rotation_vector_1(long_quat, temp_long_quat);
            inv_compute_scalar_part(temp_long_quat, result_quat);
            //	send_data_packet(PACKET_TYPE_QUAT, (void *)result_quat);
        }

        #if defined MEMS_AUGMENTED_SENSORS

        if (hal.report & PRINT_ORIENT)
        {
            if(usart_dbg)printf("Orientation\t %7.5f, %7.5f, %7.5f, %lld\r\n", orientationFloat[0], orientationFloat[1], orientationFloat[2], ts);

        }

        quat9_data_was_set = 0;
        #endif
    }

    if (gmrv_quat_was_set == 1)
    {
        if (hal.report & PRINT_GEOMAG)
        {
            if(usart_dbg)printf("GeoMAG RV\t %7.5f, %7.5f, %7.5f, %7.5f, %d, %lld\r\n", gmrv_float[0], gmrv_float[1], gmrv_float[2], gmrv_float[3], gmrv_accuracy, ts);

        }

        if (hal.report & PRINT_CUBE_GEOMAG)
        {
            //	send_data_packet(PACKET_TYPE_QUAT, (void *)cube_gmrv);
        }

        gmrv_quat_was_set = 0;
    }

    /*
    	if (hal.report & PRINT_BAC) {
    		if (bac_data_was_set == 1) {
    			if(usart_dbg)printf("BAC Ts:\t %ld\r\n", bac_ts);

    			if ((bac_state >> 8) & BAC_DRIVE)
    				print_data_console("\t Enter Drive\r\n");
    			if ((bac_state >> 8) & BAC_WALK)
    				print_data_console("\t Enter Walk\r\n");
    			if ((bac_state >> 8) & BAC_RUN)
    				print_data_console("\t Enter Run\r\n");
    			if ((bac_state >> 8) & BAC_BIKE)
    				print_data_console("\t Enter Bike\r\n");
    			if ((bac_state >> 8) & BAC_TILT)
    				print_data_console("\t Enter Tilt\r\n");
    			if ((bac_state >> 8) & BAC_STILL)
    				print_data_console("\t Enter Still\r\n");
    			if (bac_state & BAC_DRIVE)
    				print_data_console("\t Exit Drive\r\n");
    			if (bac_state & BAC_WALK)
    				print_data_console("\t Exit Walk\r\n");
    			if (bac_state & BAC_RUN)
    				print_data_console("\t Exit Run\r\n");
    			if (bac_state & BAC_BIKE)
    				print_data_console("\t Exit Bike\r\n");
    			if (bac_state & BAC_TILT)
    				print_data_console("\t Exit tilt\r\n");
    			if (bac_state & BAC_STILL)
    				print_data_console("\t Exit Still\r\n");

    			bac_data_was_set = 0;
    			//bac_prev_ts = ts;
    		}
    	}

    	if (hal.report & PRINT_FLIP_PICKUP) {
    		if (flip_data_was_set == 1) {
    			if (flip_pickup == 1)
    				print_data_console("\t Flip Detected\r\n");
    			else if (flip_pickup == 2)
    				print_data_console("\t Pickup Detected\r\n");
    			flip_data_was_set = 0;
    		}
    	}

    	if (hal.report & PRINT_TILT) {
    		if (tilt_data_was_set == 1) {
    			if (tilt_state == 2)
    				print_data_console("\t Tilt Started\r\n");
    			else if (tilt_state == 1)
    				print_data_console("\t Tilt Ended\r\n");
    			tilt_data_was_set = 0;
    		}
    	}
    */
    #endif // !HW_ICM20609

//	if (hal.report & PRINT_STEP_DETECTOR) {
//		if (step_detected == 1)
//			print_data_console("Step Detected>>>>\r\n");
//		step_detected = 0;
//	}
}

void fifo_handler(void)
{
    short int_read_back = 0;
    unsigned short header = 0, header2 = 0;
    int data_left_in_fifo = 0;
    short short_data[3] = { 0 };
    signed long  long_data[3] = { 0 };
    signed long  long_quat[3] = { 0 };
    static mpu_time_t lastIrqTimeUs = 0;
    static mpu_time_t currentIrqTimeUs = 0;
    unsigned short sample_cnt_array[GENERAL_SENSORS_MAX] = { 0 };
    int sample_nb;

    #if defined MEMS_AUGMENTED_SENSORS
    long gravityQ16[3], temp_gravityQ16[3];
    long linAccQ16[3];
    long accelQ16[3];
    #endif

    // Process Incoming INT and Get/Pack FIFO Data
    inv_identify_interrupt(&int_read_back);
    #if (MEMS_CHIP == HW_ICM20609)

    if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_2 | BIT_MSG_DMP_INT_3))
    {
    #else

    if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_0 | BIT_MSG_DMP_INT_2 | BIT_MSG_DMP_INT_5))
    {
    #endif
        // Read FIFO contents and parse it.
        unsigned short total_sample_cnt = 0;
        currentIrqTimeUs = inv_get_tick_count();

        do
        {
            if (inv_mems_fifo_swmirror(&data_left_in_fifo, &total_sample_cnt, sample_cnt_array))
                break;

            #if (MEMS_CHIP == HW_ICM20609)
            // check sample count for 20608d/20609
            sample_nb = 0;

            if (sample_cnt_array[ANDROID_SENSOR_ACCELEROMETER])
                sample_nb++;

            if (sample_cnt_array[ANDROID_SENSOR_GYROSCOPE])
                sample_nb++;

            if (sample_cnt_array[ANDROID_SENSOR_GAME_ROTATION_VECTOR])
                sample_nb++;

            sample_nb = total_sample_cnt / sample_nb;
            #else
            sample_nb = total_sample_cnt;
            #endif // HW_ICM20609

            if (lastIrqTimeUs != 0 && sample_nb)
                ts = (currentIrqTimeUs - lastIrqTimeUs) / sample_nb;

            if (sample_nb)
                lastIrqTimeUs = currentIrqTimeUs;

            while (total_sample_cnt--)
            {
                if (inv_mems_fifo_pop(&header, &header2, &data_left_in_fifo))
                    break;

                if (header & ACCEL_SET)
                {
                    float scale;
                    accel_data_was_set = 1;
                    dmp_get_accel(long_data);
                    accel_accuracy = inv_get_accel_accuracy();
                    scale = (1 << inv_get_accel_fullscale()) * 2.f / (1L << 30); // Convert from raw units to g's
                    scale *= 9.80665f; // Convert to m/s^2
                    inv_convert_dmp3_to_body(long_data, scale, accel_float);
                } // header & ACCEL_SET

                if (header & GYRO_SET)
                {
                    float scale;
                    signed long  raw_data[3] = { 0 };
                    #if ((MEMS_CHIP == HW_ICM30630) || (MEMS_CHIP == HW_ICM20648) || (MEMS_CHIP == HW_ICM20609))
                    signed long  bias_data[3] = { 0 };
                    #endif
                    raw_gyro_data_was_set = 1;
                    dmp_get_raw_gyro(short_data);
                    scale = (1 << inv_get_gyro_fullscale()) * 250.f / (1L << 15); // From raw to dps
                    scale *= (float)M_PI / 180.f; // Convert to radian.
                    raw_data[0] = (long)short_data[0];
                    raw_data[1] = (long)short_data[1];
                    raw_data[2] = (long)short_data[2];
                    inv_convert_dmp3_to_body(raw_data, scale, gyro_raw_float);

                    #if ((MEMS_CHIP == HW_ICM30630) || (MEMS_CHIP == HW_ICM20648))
                    // We have gyro bias data in raw units, scaled by 2^5
                    dmp_get_gyro_bias(short_data);
                    scale = (1 << inv_get_gyro_fullscale()) * 250.f / (1L << 20); // From raw to dps
                    scale *= (float)M_PI / 180.f; // Convert to radian.
                    bias_data[0] = (long)short_data[0];
                    bias_data[1] = (long)short_data[1];
                    bias_data[2] = (long)short_data[2];
                    inv_convert_dmp3_to_body(bias_data, scale, gyro_bias_float);

                    if (hal.report & PRINT_GYRO)
                    {
                        // shift to Q20 to do all operations in Q20
                        raw_data[0] = raw_data[0] << 5;
                        raw_data[1] = raw_data[1] << 5;
                        raw_data[2] = raw_data[2] << 5;
                        inv_mems_dmp_get_calibrated_gyro(long_data, raw_data, bias_data);
                        inv_convert_dmp3_to_body(long_data, scale, gyro_float);
                        gyro_data_was_set = 1;
                    }

                    gyro_accuracy = inv_get_gyro_accuracy();
                    #elif (MEMS_CHIP == HW_ICM20609)
                    dmp_get_gyro_bias(bias_data);
                    scale = (1 << inv_get_gyro_fullscale()) * 250.f / (1L << 20); // From raw to dps
                    scale *= (float)M_PI / 180.f; // Convert to radian.
                    inv_convert_dmp3_to_body(bias_data, scale, gyro_bias_float);

                    if (hal.report & PRINT_GYRO)
                    {
                        inv_mems_dmp_get_calibrated_gyro(long_data, short_data, bias_data);
                        // shift to Q30 to do all operations in Q30
                        scale = (1 << inv_get_gyro_fullscale()) * 250.f / (1L << 30); // From raw to dps
                        scale *= (float)M_PI / 180.f; // Convert to radian.
                        inv_convert_dmp3_to_body(long_data, scale, gyro_float);
                        gyro_data_was_set = 1;
                    }

                    if ((bias_data[0] != 0) && (bias_data[1] != 0) && (bias_data[2] != 0))
                        gyro_accuracy = 3;

                    #endif
                } // header & GYRO_SET

                if (header & GYRO_CALIBR_SET)
                {
                    #if ((MEMS_CHIP != HW_ICM30630) && (MEMS_CHIP != HW_ICM20648))
                    float scale;
                    gyro_data_was_set = 1;
                    dmp_get_calibrated_gyro(long_data);

                    gyro_accuracy = inv_get_gyro_accuracy();
                    // We have gyro data in raw units, scaled by 2^15
                    scale = (1 << inv_get_gyro_fullscale()) * 250.f / (1L << 30); // From raw to dps
                    scale *= (float)M_PI / 180.f; // Convert to radian.
                    inv_convert_dmp3_to_body(long_data, scale, gyro_float);
                    #endif
                } // header & GYRO_CALIBR_SET

                #if(MEMS_CHIP != HW_ICM20609)

                if (header & CPASS_CALIBR_SET)
                {
                    float scale;
                    compass_data_was_set = 1;
                    //signed long  long_data[3] = { 0 };
                    dmp_get_calibrated_compass(long_data);
                    // compass_accuracy = 0;
                    //scale = 0; //COMPASS_CONVERSION
                    compass_accuracy = inv_get_mag_accuracy();
                    scale = 1.52587890625e-005f; //COMPASS_CONVERSION
                    inv_convert_dmp3_to_body(long_data, scale, compass_float);
                } // header & CPASS_CALIBR_SET

                if (header & CPASS_SET)
                {
                    // Raw compass [DMP]
                    //add by lzl
                    // signed long    long_data[3] = { 0 };
                    dmp_get_raw_compass(long_data);

                    compass_raw_float[0] = long_data[0] * 1.52587890625e-005f;
                    compass_raw_float[1] = long_data[1] * 1.52587890625e-005f;
                    compass_raw_float[2] = long_data[2] * 1.52587890625e-005f;
                    compass_bias[0] = compass_raw_float[0] - compass_float[0];
                    compass_bias[1] = compass_raw_float[1] - compass_float[1];
                    compass_bias[2] = compass_raw_float[2] - compass_float[2];

                    #if 1
                    compass_raw_float[0] = 0;
                    compass_raw_float[1] = 0;
                    compass_raw_float[2] = 0;
                    compass_bias[0] = 0;
                    compass_bias[1] = 0;
                    compass_bias[2] = 0;
                    #endif
                    raw_compass_data_was_set = 1;
                } // header & CPASS_SET

                #endif

                if (header & QUAT6_SET)
                {
                    quat6_data_was_set = 1;
                    dmp_get_6quaternion(long_quat);
                    inv_convert_rotation_vector(long_quat, grv_float);
                    #if defined MEMS_AUGMENTED_SENSORS
//					long gravityQ16[3], temp_gravityQ16[3];
//					long linAccQ16[3];
//					long accelQ16[3];
                    /*Calculate Gravity*/
                    inv_convert_rotation_vector_1(long_quat, temp_gravityQ16);
                    inv_mems_augmented_sensors_get_gravity(gravityQ16, temp_gravityQ16);
                    gravityFloat[0] = inv_q16_to_float(gravityQ16[0]);
                    gravityFloat[1] = inv_q16_to_float(gravityQ16[1]);
                    gravityFloat[2] = inv_q16_to_float(gravityQ16[2]);
                    /*Calculate Linear Acceleration*/
                    accelQ16[0] = (int32_t)((float)(accel_float[0]) * (1ULL << 16) + ((accel_float[0] >= 0) - 0.5f));
                    accelQ16[1] = (int32_t)((float)(accel_float[1]) * (1ULL << 16) + ((accel_float[1] >= 0) - 0.5f));
                    accelQ16[2] = (int32_t)((float)(accel_float[2]) * (1ULL << 16) + ((accel_float[2] >= 0) - 0.5f));
                    inv_mems_augmented_sensors_get_linearacceleration(linAccQ16, gravityQ16, accelQ16);
                    linAccFloat[0] = inv_q16_to_float(linAccQ16[0]);
                    linAccFloat[1] = inv_q16_to_float(linAccQ16[1]);
                    linAccFloat[2] = inv_q16_to_float(linAccQ16[2]);
                    #endif
                } // header & QUAT6_SET

                #if (MEMS_CHIP != HW_ICM20609)

                /* 9axis orientation quaternion sample available from DMP FIFO */
                if (header & QUAT9_SET)
                {
                    #if defined MEMS_AUGMENTED_SENSORS
                    long orientationQ16[3], temp_orientationQ16[3];
                    #endif
                    quat9_data_was_set = 1;
                    dmp_get_9quaternion(long_quat);
                    #if (MEMS_CHIP == HW_ICM20630 || MEMS_CHIP == HW_ICM20648)
                    rv_accuracy = (int)((float)inv_get_rv_accuracy() / (float)(1ULL << (29)));
                    #else
                    compass_accuracy = inv_get_mag_accuracy();
                    #endif
                    inv_convert_rotation_vector(long_quat, rv_float);
                    #if defined MEMS_AUGMENTED_SENSORS

                    if (hal.report & PRINT_ORIENT)
                    {
                        inv_convert_rotation_vector_1(long_quat, temp_orientationQ16);
                        inv_mems_augmented_sensors_get_orientation(orientationQ16, temp_orientationQ16);
                        orientationFloat[0] = inv_q16_to_float(orientationQ16[0]);
                        orientationFloat[1] = inv_q16_to_float(orientationQ16[1]);
                        orientationFloat[2] = inv_q16_to_float(orientationQ16[2]);
                    }

                    #endif
                } // header & QUAT9_SET

                #if (MEMS_CHIP == HW_ICM20648)

                /* 6axis AM orientation quaternion sample available from DMP FIFO */
                if (header & GEOMAG_SET)
                {
                    /* Read 6 axis quaternion out of DMP FIFO in Q30 and convert it to Android format */
                    dmp_get_gmrvquaternion(long_quat);

                    if (hal.report & PRINT_CUBE_GEOMAG)
                        inv_compute_scalar_part(long_quat, cube_gmrv);

                    if (hal.report & PRINT_GEOMAG)
                    {
                        inv_convert_rotation_vector(long_quat, gmrv_float);
                        /* Read geomagnetic rotation vector heading accuracy out of DMP FIFO in Q29*/
                        gmrv_accuracy = (int)((float)inv_get_gmrv_accuracy() / (float)(1ULL << (29)));
                    }

                    gmrv_quat_was_set = 1;
                }

                #endif

                #if (MEMS_CHIP == HW_ICM20645e) || (MEMS_CHIP == HW_ICM20648)

                /* Activity recognition sample available from DMP FIFO */
                if (header2 & ACT_RECOG_SET)
                {
                    /* Read activity type and associated timestamp out of DMP FIFO
                    activity type is a set of 2 bytes :
                    - high byte indicates activity start
                    - low byte indicates activity end */
                    dmp_get_bac_state(&bac_state);
                    dmp_get_bac_ts(&bac_ts);

                    if (hal.report & PRINT_TILT)
                    {
                        /* Tilt information is inside BAC, so extract it out */
                        /* Check if bit tilt is set for activity start byte */
                        if ((bac_state >> 8) & BAC_TILT)
                        {
                            /* Start of tilt detected */
                            tilt_state = 2;
                        }
                        /* Check if bit tilt is set for activity end byte */
                        else if (bac_state & BAC_TILT)
                        {
                            /* End of tilt detected */
                            tilt_state = 1;
                        }
                    }

                    if (hal.report & PRINT_BAC)
                        bac_data_was_set = 1;

                    if (hal.report & PRINT_TILT)
                        tilt_data_was_set = 1;
                }

                if (header2 & FLIP_PICKUP_SET)
                {
                    if (hal.report & PRINT_FLIP_PICKUP)
                    {
                        dmp_get_flip_pickup_state(&flip_pickup);
                        flip_data_was_set = 1;
                    }
                }

                #endif // (MEMS_CHIP == HW_ICM20645e) || (MEMS_CHIP == HW_ICM20648)
                #endif // !HW_ICM20609

                process_sensor_output();
            } // total_sample_cnt

            if (!data_left_in_fifo)
                break;
        }
        while (data_left_in_fifo);

        if (int_read_back & BIT_MSG_DMP_INT_3)
        {
            if (hal.report & PRINT_STEP_DETECTOR)
            {
                if(usart_dbg)printf("Step Detected>>>>>>>\r\n");
            }
        }

        if (header & PED_STEPDET_SET)
        {
            if (hal.report & PRINT_STEP_COUNTER)
            {
                unsigned long steps = 0;
                static unsigned long old_steps;
                dmp_get_pedometer_num_of_steps(&steps);

                if (steps != old_steps)
                {
                    if(usart_dbg)printf("\tStep Counter %d\r\n", steps);

                    old_steps = steps;
                }
            }
        }

        if (int_read_back & BIT_MSG_DMP_INT_2)
        {
            if (hal.report & PRINT_SMD)
                if(usart_dbg)printf(">> SMD Interrupt *********\r\n");
        }
    } // int_read_back
}



//////////////////////////////////////////////////////////////////////////////////

static unsigned short RETRY_IN_MLSEC  = 55;

void Set_I2C_Retry(unsigned short ml_sec)
{
    RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C_Retry()
{
    return RETRY_IN_MLSEC;
}

//初始化 陀螺仪相关参数
int ICM_20948_Init(void)
{
    unsigned char tx, rx;
    inv_error_t result = 0;

    #ifdef USART_DBG
    usart_dbg = 1;
    #endif

    tx = 0x00;
    result |= inv_serial_interface_write_hook(0x7f, 1, &tx);
    tx = 0x80;
    result |= inv_serial_interface_write_hook(0x06, 1, &tx); //复位
    delay_ms(100);
    // App_Delay_ms(100);
    tx = 0x00;
    result |= inv_serial_interface_write_hook(0x7f, 1, &tx);
    result |= inv_serial_interface_read_hook(0x00, 1, &rx); //WHO_AM_I

    if(rx != 0xEA) //WHO_AM_I
    {
        printf("WHO_AM_I ERROR:0x%02X\n", rx);
        return 1;
    }

    if(usart_dbg)printf("address = 0x%02X\r\n", rx);
	
    inv_set_chip_to_body_axis_quaternion(ACCEL_GYRO_ORIENTATION, 0.0);
    result |= inv_initialize_lower_driver(SERIAL_INTERFACE_I2C, 0);
	if (result)
    {
        if(usart_dbg)printf("inv_initialize_lower_driver error.\r\n");
    }
    result |= inv_set_slave_compass_id(0x24);
    if (result)
    {
        if(usart_dbg)printf("compass_id error.\r\n");
    }
	
    self_test();
    result |= dmp_set_bias(dmp_bias);
    if (result)
    {
        if(usart_dbg)printf("dmp_set_bias error.\r\n");
    }
	
	result |= dmp_reset_fifo();
    if (result)
    {
        if(usart_dbg)printf("dmp_reset_fifo error.\r\n");
    }
	
	hal.report ^= PRINT_ACCEL;
	result |= inv_enable_sensor(ANDROID_SENSOR_ACCELEROMETER, !!(hal.report & PRINT_ACCEL));
	if (result)
    {
        if(usart_dbg)printf("inv_enable_sensor1 error.\r\n");
    }

	hal.report ^= PRINT_GYRO;
	result |= inv_enable_sensor(ANDROID_SENSOR_GYROSCOPE, !!(hal.report & PRINT_GYRO));
	if (result)
    {
        if(usart_dbg)printf("inv_enable_sensor2 error.\r\n");
    }

//	hal.report ^= PRINT_RAW_COMPASS;
//	result |= inv_enable_sensor(ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, !!(hal.report & PRINT_RAW_COMPASS));
//	result |= inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 1); // to calculate bias in application layer

	hal.report ^= PRINT_COMPASS;
	inv_enable_sensor(ANDROID_SENSOR_GEOMAGNETIC_FIELD, 1);
	
	hal.report ^= PRINT_GRV;
	inv_enable_sensor(ANDROID_SENSOR_GAME_ROTATION_VECTOR, !!(hal.report & PRINT_GRV));

	result |= inv_reset_dmp_odr_counters();
    if (result)
    {
        if(usart_dbg)printf("inv_reset_dmp_odr_counters error.\r\n");
    }
    //set speed
    // current_output_rate = 1;
	// current_output_rate = 5;
	// current_output_rate = 15;
	// current_output_rate = 30;
	// current_output_rate = 51;
	// current_output_rate = 56.5;
	// current_output_rate = 60;
	current_output_rate = 102;
	// current_output_rate = 112.5;
	// current_output_rate = 225;
    result |= set_output_rates(current_output_rate);

    if (result)
    {
        if(usart_dbg)printf("Could not initialize.\r\n");
    }
    else
    {
        if(usart_dbg)printf("ICM20948 Initialize Sucessful...\r\n");
    }

    GPIO_INT_Config();
    init_state = 1;

    //返回初始状态
    return result;
}

/**
* @brief main entry point.
* @par Parameters None
* @retval void None
* @par Required preconditions: None
*/
void self_test(void)
{
//	int i, ret;
//	uint8_t d[3];
//	int bias[3];
    float scale;

    //uint8_t d[2];
    //int ret, i;
    char pass_str[] = { "PASS" };
    char fail_str[] = { "FAIL" };
    #ifndef BIAS_SET_TO_DMP
    int accel_offset[3], gyro_offset[3];
    int ret, i;
    #endif

    if(usart_dbg)printf("Selftest...Started\r\n");


    #ifndef BIAS_SET_TO_DMP
    /* Reset offset registers before running self-test */
    ret = reset_offset_regs();

    if (ret)
    {
        if(usart_dbg)printf("Failed to reset offset registers\r\n");
    }

    #endif

    /* Perform self-test */
    self_test_result = inv_mems_run_selftest();
    if(usart_dbg)printf("Selftest...Done...Ret=%d\r\n", self_test_result);

    if(usart_dbg)printf("Result: Compass=%s, Accel=%s, Gyro=%s\r\n", (self_test_result & 0x04) ? pass_str : fail_str, (self_test_result & 0x02) ? pass_str : fail_str, (self_test_result & 0x01) ? pass_str : fail_str);

    if(usart_dbg)printf("Accel Average (LSB@FSR 2g)\r\n");

    if(usart_dbg)printf("\tX:%d Y:%d Z:%d\r\n", a_average[0], a_average[1], a_average[2]);

    if(usart_dbg)printf("Gyro Average (LSB@FSR 250dps)\r\n");

    if(usart_dbg)printf("\tX:%d Y:%d Z:%d\r\n", g_average[0], g_average[1], g_average[2]);


    /* Nothing to do if FAIL on gyro and accel */
    if ((self_test_result & 0x03) != 0x03)
        return;

    /* Handle bias got by self-test */
    #ifdef BIAS_SET_TO_DMP // Set to DMP
    // for ICM20648/20948
    dmp_bias[0] = a_average[0] * (1 << 11);   // Change from LSB to format expected by DMP
    dmp_bias[1] = a_average[1] * (1 << 11);
    dmp_bias[2] = (a_average[2] - 16384) * (1 << 11); //remove the gravity and scale (FSR=2 in selftest)
    scale = 2000 / 250; 						//self-test uses 250dps FSR, main() set the FSR to 2000dps
    dmp_bias[3] = g_average[0] * (1 << 15) / scale;
    dmp_bias[4] = g_average[1] * (1 << 15) / scale;
    dmp_bias[5] = g_average[2] * (1 << 15) / scale;

    if(usart_dbg)printf("Factory Cal - Accel DMP biases: \tX:%d Y:%d Z:%d\r\n", dmp_bias[0], dmp_bias[1], dmp_bias[2]);

    if(usart_dbg)printf("Factory Cal - Gyro DMP biases:  \tX:%d Y:%d Z:%d\r\n", dmp_bias[3], dmp_bias[4], dmp_bias[5]);

    /* Update bias on DMP memory */
    dmp_set_bias(dmp_bias);
    if(usart_dbg)printf("\r\nSetting the DMP biases with one-axis factory calibration values...done\r\n");

    #else // Set to offset registers

    // for ICM20648/20948/20608D/20609/20689
    for (i = 0; i < 3; i++)
    {
        if (i == 2)
        {
            // assume the device is put horizontal and z axis shows 1g during self-test.
            // self-test uses 2g FSR and 1g = 16384LSB
            accel_offset[i] = a_average[i] - 16384;
        }
        else
        {
            accel_offset[i] = a_average[i];
        }

        // gyro FSR is 250dps for self-test
        gyro_offset[i] = g_average[i];
    }

    /* Update offset registers */
    ret = set_offset_regs(accel_offset, gyro_offset);

    if (ret)
    {
        if(usart_dbg)printf("Failed to update offset registers\r\n");
    }
    else
    {
        if(usart_dbg)printf("\r\nSetting the offset registers with one-axis factory calibration values...done\r\n");
    }

    #endif // BIAS_SET_TO_DMP
    self_test_done = 1;
}

//返回值:温度值
float ICM20948_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    inv_serial_interface_read_hook(0x39, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 21 + ((float)raw) / 333.87f;
    return temp;
}

// 将ICM20948的数据取出来
void ICM20948_Get_Data(icm20948_data_t *data)
{
	// i20948_data.temperature = ICM20948_Get_Temperature();
	memcpy(data,&i20948_data,sizeof(icm20948_data_t));
}

// 返回ICM20948是否初始化完成
uint8_t ICM20948_Get_Init_State(void)
{
    return init_state;
}

// 获取计数值
int get_tick_count(long long *count)
{
    count[0] = g_ul_ms_ticks;
	return 0;
}

// 获取当前翻滚角
float ICM20948_Get_Roll_Now(void)
{
    return g_icm_data.orientation[0];         // 返回弧度
    // return g_icm_data.orientation[0]*RtA;  // 返回角度
}

// 获取当前俯仰角
float ICM20948_Get_Pitch_Now(void)
{
    return g_icm_data.orientation[1];         // 返回弧度
    // return g_icm_data.orientation[1]*RtA;  // 返回角度
}

// 获取当前偏航角
float ICM20948_Get_Yaw_Now(void)
{
    return g_icm_data.orientation[2];         // 返回弧度
    // return g_icm_data.orientation[2]*RtA;  // 返回角度
}

// ICM20948读取数据的
void ICM20948_Read_Data_Handle(void)
{
    if (hal.new_gyro == 1)
    {
        hal.new_gyro = 0;
        fifo_handler();    //处理函数可放于中断
        ICM20948_Get_Data(&g_icm_data);
        // printf("Accel Data\t %8.5f,\t %8.5f,\t %8.5f\r\n", g_icm_data.accel_float[0], g_icm_data.accel_float[1], g_icm_data.accel_float[2]);
        // printf("Gyro Data\t %7.5f,\t %7.5f,\t %7.5f\r\n", g_icm_data.gyro_float[0], g_icm_data.gyro_float[1], g_icm_data.gyro_float[2]);
        // printf("Compass Data\t %7.5f,\t %7.5f,\t %7.5f\r\n", g_icm_data.compass_float[0], g_icm_data.compass_float[1], g_icm_data.compass_float[2]);
        // printf("Orientation\t %7.5f, %7.5f, %7.5f\r\n", g_icm_data.orientation[0]*RtA, g_icm_data.orientation[1]*RtA,g_icm_data.orientation[2]*RtA);
        // printf("Temperature\t %.3f\r\n",g_icm_data.temperature);
    }
    g_ul_ms_ticks++;
}

// 发送原始数据到主控
void ICM20948_Send_Raw_Data(void)
{
    #define LEN        23
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN-2; // 数量
	data_buffer[3] = FUNC_REPORT_ICM_RAW; // 功能位
	data_buffer[4] = (int)(g_icm_data.gyro_float[0]*1000) & 0xff;
	data_buffer[5] = ((int)(g_icm_data.gyro_float[0]*1000) >> 8) & 0xff;
	data_buffer[6] = (int)(g_icm_data.gyro_float[1]*1000) & 0xff;
	data_buffer[7] = ((int)(g_icm_data.gyro_float[1]*1000) >> 8) & 0xff;
	data_buffer[8] = (int)(g_icm_data.gyro_float[2]*1000) & 0xff;
	data_buffer[9] = ((int)(g_icm_data.gyro_float[2]*1000) >> 8) & 0xff;

	data_buffer[10] = (int)(g_icm_data.accel_float[0]*1000) & 0xff;
	data_buffer[11] = ((int)(g_icm_data.accel_float[0]*1000) >> 8) & 0xff;
	data_buffer[12] = (int)(g_icm_data.accel_float[1]*1000) & 0xff;
	data_buffer[13] = ((int)(g_icm_data.accel_float[1]*1000) >> 8) & 0xff;
	data_buffer[14] = (int)(g_icm_data.accel_float[2]*1000) & 0xff;
	data_buffer[15] = ((int)(g_icm_data.accel_float[2]*1000) >> 8) & 0xff;

	data_buffer[16] = (int)(g_icm_data.compass_float[0]*1000) & 0xff;
	data_buffer[17] = ((int)(g_icm_data.compass_float[0]*1000) >> 8) & 0xff;
	data_buffer[18] = (int)(g_icm_data.compass_float[1]*1000) & 0xff;
	data_buffer[19] = ((int)(g_icm_data.compass_float[1]*1000) >> 8) & 0xff;
	data_buffer[20] = (int)(g_icm_data.compass_float[2]*1000) & 0xff;
	data_buffer[21] = ((int)(g_icm_data.compass_float[2]*1000) >> 8) & 0xff;

	for (i = 2; i < LEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}

// 发送姿态角数据到主控，单位：弧度
void ICM20948_Send_Attitude_Data(void)
{
    #define LENS        11
	uint8_t data_buffer[LENS] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LENS-2; // 数量
	data_buffer[3] = FUNC_REPORT_IMU_ATT;    // 功能位
	data_buffer[4] = (int)(g_icm_data.orientation[0]*10000) & 0xff;
	data_buffer[5] = ((int)(g_icm_data.orientation[0]*10000) >> 8) & 0xff;
	data_buffer[6] = (int)(g_icm_data.orientation[1]*10000) & 0xff;
	data_buffer[7] = ((int)(g_icm_data.orientation[1]*10000) >> 8) & 0xff;
	data_buffer[8] = (int)(g_icm_data.orientation[2]*10000) & 0xff;
	data_buffer[9] = ((int)(g_icm_data.orientation[2]*10000) >> 8) & 0xff;

	for (i = 2; i < LENS-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LENS-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}

