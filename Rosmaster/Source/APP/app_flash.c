#include "app_flash.h"
#include "app.h"
#include "app_pid.h"
#include "app_motion.h"
#include "app_ackerman.h"

#include "bsp_flash.h"
#include "bsp_motor.h"
#include "app_uart_servo.h"

#include "stdint.h"
#include "bsp.h"

#if ENABLE_FLASH

/*其他静态初始化函数声明********************************************************/
static void Flash_Auto_Report_Init(void);
static void Flash_PID_Init(void);
static void Flash_ARM_Mid_Offset_Init(void);
static void Flash_CarType_Init(void);
static void Flash_AKM_Angle_Init(void);


static uint16_t Flash_Read_TestMode(void);
static uint16_t Flash_Read_CarType(void);
static uint16_t Flash_Read_Auto_Report(void);
static void Flash_Read_PID(uint8_t motor_id, float* kp, float* ki, float* kd);
static void Flash_Read_Yaw_PID(float* kp, float* ki, float* kd);
static uint16_t Flash_Read_AKM_Angle(void);
static uint16_t Flash_Read_Reset_State(void);

/*工厂测试模式******************************************************************/
// 读取工厂测试模式的值。
static uint16_t Flash_Read_TestMode(void)
{
    uint16_t temp = 0xFFFF;
    Flash_Read(F_TEST_MODE_ADDR, &temp, 1);
    printf("FRead testMode:%d\n", temp);
    return temp;
}

// 设置工厂测试模式的值。
void Flash_Set_TestMode(uint8_t mode)
{
    uint16_t temp = mode;
    // if (Flash_Read_TestMode() == temp) return;
    Flash_Write(F_TEST_MODE_ADDR, &temp, 1);
    DEBUG("FSet testMode:%d\n", temp);
}

// 初始化数据
void Flash_TestMode_Init(void)
{
    uint16_t mode = Flash_Read_TestMode();
    if (mode == 0xFFFF)
    {
        mode = 0;
        Flash_Set_TestMode(0);
    }
    Bsp_Set_TestMode(mode);
}
/*工厂测试模式******************************************************************/


/*小车类型******************************************************************/
// 读取小车类型的值。
static uint16_t Flash_Read_CarType(void)
{
    uint16_t temp = 0xFFFF;
    Flash_Read(F_CAR_TYPE_ADDR, &temp, 1);
    printf("FRead carType:%d\n", temp);
    return temp;
}

// 设置小车类型的值。
void Flash_Set_CarType(uint8_t carType)
{
    #if !ENABLE_CAR_SUNRISE_ONLY
    uint16_t temp = carType;
    if (Flash_Read_CarType() == temp) return;
    Flash_Write(F_CAR_TYPE_ADDR, &temp, 1);
    DEBUG("FSet carType:%d\n", temp);
    #endif
}

// 初始化数据
static void Flash_CarType_Init(void)
{
    car_type_t car_type = (car_type_t)Flash_Read_CarType();
    if (car_type == 0xFFFF)
    {
        car_type = CAR_MECANUM;
        Flash_Set_CarType(car_type);
    }
    Motion_Set_Car_Type(car_type);
}
/*小车类型******************************************************************/


/*自动上报开关******************************************************************/
// 读取自动传输数据的值。
static uint16_t Flash_Read_Auto_Report(void)
{
    uint16_t state = 0xFFFF;
    Flash_Read(F_AUTO_REPORT_ADDR, &state, 1);
    printf("FRead report:%d\n", state);
    return state;
}

// 设置自动上传的值。
void Flash_Set_Auto_Report(uint8_t enable)
{
    uint16_t state = 0x00;
    if (Flash_Read_Auto_Report() == enable) return;
    if (enable)
    {
        state = 0x01;
    }
    Flash_Write(F_AUTO_REPORT_ADDR, &state, 1);
    DEBUG("FSet report:%d\n", state);
}

// 自动报告初始化
static void Flash_Auto_Report_Init(void)
{
    uint16_t state = Flash_Read_Auto_Report();
    if (state == 0xFFFF)
    {
        state = ENABLE_AUTO_REPORT;
    }
    Set_Auto_Report(state);
}
/*自动上报开关******************************************************************/

/*PID参数数据******************************************************************/
// PID参数初始化
static void Flash_PID_Init(void)
{
    float kp = 0, ki = 0, kd = 0;
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     Flash_Read_PID(i, &kp, &ki, &kd);
    //     DEBUG("Read PID-M%d:%.3f, %.3f, %.3f\n", i+1, kp, ki, kd);
    //     if (kp>10 || ki>10 || kd>10)
    //     {
    //         kp = PID_DEF_KP;
    //         ki = PID_DEF_KI;
    //         kd = PID_DEF_KD;
    //         Flash_Set_PID(i, kp, ki, kd);
    //     }
    //     PID_Set_Motor_Parm(i, kp, ki, kd);
    //     kp = 0, ki = 0, kd = 0;
    // }
    Flash_Read_PID(MAX_MOTOR, &kp, &ki, &kd);
    printf("FRead PID:%.3f, %.3f, %.3f\n", kp, ki, kd);
    if (kp>10 || ki>10 || kd>10)
    {
        kp = PID_DEF_KP;
        ki = PID_DEF_KI;
        kd = PID_DEF_KD;
        Flash_Set_PID(MAX_MOTOR, kp, ki, kd);
    }
    PID_Set_Motor_Parm(MAX_MOTOR, kp, ki, kd);
    kp = 0, ki = 0, kd = 0;

    Flash_Read_Yaw_PID(&kp, &ki, &kd);
    if (kp>10 || ki>10 || kd>10)
    {
        kp = PID_YAW_DEF_KP;
        ki = PID_YAW_DEF_KI;
        kd = PID_YAW_DEF_KD;
        Flash_Set_Yaw_PID(PID_YAW_DEF_KP, PID_YAW_DEF_KI, PID_YAW_DEF_KD);
    }
    PID_Yaw_Set_Parm(kp, ki, kd);
}

// 读取PID数据，保留三位小数
static void Flash_Read_PID(uint8_t motor_id, float* kp, float* ki, float* kd)
{
    uint16_t pid_1k[3] = {0};
    // uint16_t offset = 6 * motor_id;
    uint16_t offset = 0;
    Flash_Read(F_PID_ADDR + offset, pid_1k, 3);
    *kp = pid_1k[0] / 1000.0;
    *ki = pid_1k[1] / 1000.0;
    *kd = pid_1k[2] / 1000.0;
    // DEBUG("FRead PID:%d, %d, %d\n", pid_1k[0], pid_1k[1], pid_1k[2]);
}

// 设置PID参数，保留三位小数。
void Flash_Set_PID(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id > MAX_MOTOR) return;
    uint16_t pid_1k[] = {1000*kp, 1000*ki, 1000*kd};

    // if (motor_id == MAX_MOTOR)
    // {
    //     uint16_t all_pid_1k[] = {
    //         pid_1k[0], pid_1k[1], pid_1k[2], 
    //         pid_1k[0], pid_1k[1], pid_1k[2], 
    //         pid_1k[0], pid_1k[1], pid_1k[2], 
    //         pid_1k[0], pid_1k[1], pid_1k[2]};
    //     Flash_Write(F_PID_ADDR, all_pid_1k, sizeof(all_pid_1k)/sizeof(all_pid_1k[0]));
    // }
    // else
    // {
    //     uint16_t offset = 6 * motor_id;
    //     Flash_Write(F_PID_ADDR + offset, pid_1k, 3);
    // }
    uint16_t offset = 0;
    Flash_Write(F_PID_ADDR + offset, pid_1k, 3);
    DEBUG("FSet PID:%d, %d, %d, %d\n", motor_id+1, pid_1k[0], pid_1k[1], pid_1k[2]);
}

// 读取偏航角PID数据，保留三位小数
static void Flash_Read_Yaw_PID(float* kp, float* ki, float* kd)
{
    uint16_t pid_1k[3] = {0};
    Flash_Read(F_PID_YAW_ADDR, pid_1k, 3);
    *kp = pid_1k[0] / 1000.0;
    *ki = pid_1k[1] / 1000.0;
    *kd = pid_1k[2] / 1000.0;
    // DEBUG("FRead Yaw PID:%d, %d, %d\n", pid_1k[0], pid_1k[1], pid_1k[2]);
}

// 设置偏航角PID参数，保留三位小数。
void Flash_Set_Yaw_PID(float kp, float ki, float kd)
{
    uint16_t pid_1k[] = {1000*kp, 1000*ki, 1000*kd};
    Flash_Write(F_PID_YAW_ADDR, pid_1k, 3);
    DEBUG("FSet Yaw PID:%d, %d, %d\n", pid_1k[0], pid_1k[1], pid_1k[2]);
}


/*PID参数数据******************************************************************/



/*机械臂中位偏差****************************************************************/

static void Flash_ARM_Mid_Offset_Init(void)
{
    uint16_t value_def[] = {MAX_SERVO_NUM, MEDIAN_VALUE, MEDIAN_VALUE, 
        MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};
    for (int i = 0; i < MAX_SERVO_NUM; i++)
    {
        uint16_t value = 0;
        Flash_Read_ARM_Median_Value(i+1, &value);
        if (value > ARM_MAX_VALUE)
        {
            Flash_Reset_ARM_Median_Value();
            value = value_def[i+1];
        }
        UartServo_Set_Median_Value(i+1, value);
    }
}

/* 重置FLASH中机械臂中位偏差的值为默认值,包括全局变量值也一起改了 */
void Flash_Reset_ARM_Median_Value(void)
{
    uint16_t value[] = {MAX_SERVO_NUM, MEDIAN_VALUE, MEDIAN_VALUE, 
        MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};
    Flash_Write(F_ARM_OFFSET_ADDR, value, 7);

    for (int i = 0; i < MAX_SERVO_NUM; i++)
    {
        UartServo_Set_Median_Value(i+1, value[i+1]);
    }
    DEBUG("FReset all arm offset\n");
}

/* 写入中位偏差到flash内保存 */
void Flash_Set_ARM_Median_Value(uint8_t id, uint16_t value)
{
    if ((id >= 1) && (id <= MAX_SERVO_NUM))
    {
        uint16_t mid_value = value;
        uint16_t addr_offset = 2 * id;
        Flash_Write(F_ARM_OFFSET_ADDR + addr_offset, &mid_value, 1);
        UartServo_Set_Median_Value(id, value);
    }
}

/* 从flash读取中位偏差 */
void Flash_Read_ARM_Median_Value(uint8_t id, uint16_t* value)
{
    uint16_t mid_value = 0;
    if ((id >= 1) && (id <= MAX_SERVO_NUM))
    {
        uint16_t addr_offset = 2 * id;
        Flash_Read(F_ARM_OFFSET_ADDR + addr_offset, &mid_value, 1);
    }
    *value = mid_value;
    printf("FRead ARM Median:%d, %d\n", id, mid_value);
}

/*机械臂中位偏差****************************************************************/


/*阿克曼小车数据******************************************************************/
// 读取阿克曼小车角度的值。
static uint16_t Flash_Read_AKM_Angle(void)
{
    uint16_t temp = 0xFFFF;
    Flash_Read(F_AKM_ANGLE_ADDR, &temp, 1);
    printf("FRead AKM Angle:%d\n", temp);
    return temp;
}

// 设置阿克曼小车角度的值。
void Flash_Set_AKM_Angle(uint16_t angle)
{
    uint16_t temp = angle;
    if (Flash_Read_AKM_Angle() == temp) return;
    Flash_Write(F_AKM_ANGLE_ADDR, &temp, 1);
    DEBUG("FSet AKM Angle:%d\n", temp);
}

// 初始化数据
static void Flash_AKM_Angle_Init(void)
{
    uint16_t temp = Flash_Read_AKM_Angle();
    if (temp > 180)
    {
        temp = AKM_ANGLE_INIT;
        Flash_Set_AKM_Angle(temp);
    }
    Ackerman_Set_Default_Angle(temp, 0);
}
/*阿克曼小车数据******************************************************************/



/*Flash初始化设置**************************************************************/
// 重置所有数值。
void Flash_Reset_All_Value(void)
{
    // 将保存到FLASH的值设置为默认。
    Flash_Set_TestMode(MODE_STANDARD);
    Flash_Set_Auto_Report(ENABLE_AUTO_REPORT);
    Flash_Set_PID(MAX_MOTOR, PID_DEF_KP, PID_DEF_KI, PID_DEF_KD);
    Flash_Set_Yaw_PID(PID_YAW_DEF_KP, PID_YAW_DEF_KI, PID_YAW_DEF_KD);
    Flash_Reset_ARM_Median_Value();
    Flash_Set_CarType(CAR_MECANUM);
    Flash_Set_AKM_Angle(AKM_ANGLE_INIT);


    // 保存FLASH 数值状态为OK
    uint16_t state = FLASH_RESET_OK;
    Flash_Write(F_RESET_ALL_ADDR, &state, 1);
    DEBUG("FReset All OK:%d\n", state);
}

// 读取FLASH是否已经初始化数值完毕
static uint16_t Flash_Read_Reset_State(void)
{
    uint16_t val = 0xFFFF;
    Flash_Read(F_RESET_ALL_ADDR, &val, 1);
    printf("FRead Init Value:0x%X\n", val);
    return val;
}

// 初始化Flash的值。
void Flash_Init(void)
{
    // 手动复位Flash数据
    #if ENABLE_RESET_FLASH
    Flash_Reset_All_Value();
    #endif

    // 用于首次烧录芯片，将数据设置为默认值。
    if (Flash_Read_Reset_State() != FLASH_RESET_OK)
    {
        Flash_Reset_All_Value();
    }
    Flash_AKM_Angle_Init();
    Flash_CarType_Init();
    Flash_Auto_Report_Init();
    Flash_PID_Init();
    Flash_ARM_Mid_Offset_Init();
}
/*Flash初始化设置**************************************************************/



#else
void Flash_Init(void){}
void Flash_Reset_All_Value(void){}

void Flash_Set_CarType(uint8_t carType){}

void Flash_Set_Auto_Report(uint8_t enable){}


void Flash_Set_PID(uint8_t motor_id, float kp, float ki, float kd){}

void Flash_Set_Yaw_PID(float kp, float ki, float kd){}


void Flash_Reset_ARM_Median_Value(void){}
void Flash_Set_ARM_Median_Value(uint8_t id, uint16_t value){}
void Flash_Read_ARM_Median_Value(uint8_t id, uint16_t* value){}


void Flash_Set_AKM_Angle(uint16_t angle){}

void Flash_TestMode_Init(void){}
void Flash_Set_TestMode(uint8_t mode){}

#endif
