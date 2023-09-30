#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__

#include "stdint.h"
#include "bsp_flash.h"

/******************************Flash 地址配置**********************************************/
// 要保存数据的扇区
#define FLASH_DATA_SECTOR     120             //STM32F103RCT6 扇区总共有0~127个扇区，每个扇区大小为2K

// 设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

// 工厂测试模式的开关。 大小为2字节。
#define F_TEST_MODE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x00)

// 重置Flash的值的开关。 大小为2字节。
#define F_RESET_ALL_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x02)

// 自动上传数据开关的状态保存地址，大小为2字节
#define F_CAR_TYPE_ADDR           (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x04)

// 自动上传数据开关的状态保存地址，大小为2字节
#define F_AUTO_REPORT_ADDR        (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x06)

// 陀螺仪调节偏航角状态保存地址，大小为2字节
#define F_IMU_STATE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x0A)

// RGB灯特效，大小为2字节
#define F_RGB_EFFECT_ADDR         (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x10)

// YAW PID参数，大小为6字节
#define F_PID_YAW_ADDR            (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x20)

// PID参数，大小为24字节
#define F_PID_ADDR                (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x30)

// 机械臂中位偏差参数，大小为14字节
#define F_ARM_OFFSET_ADDR         (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x50)



#define F_AKM_ANGLE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x70)

/******************************Flash 地址配置**********************************************/


/******************************Flash宏定义变量配置******************************************/
// 如果F_RESET_ALL_ADDR地址读出来的值不是FLASH_RESET_OK，则自动恢复所有值为默认。
#define FLASH_RESET_OK               0xAA55


/******************************Flash宏定义变量配置******************************************/


void Flash_Init(void);
void Flash_Reset_All_Value(void);

void Flash_Set_CarType(uint8_t carType);

void Flash_Set_Auto_Report(uint8_t enable);

void Flash_Set_PID(uint8_t motor_id, float kp, float ki, float kd);

void Flash_Set_Yaw_PID(float kp, float ki, float kd);


void Flash_Reset_ARM_Median_Value(void);
void Flash_Set_ARM_Median_Value(uint8_t id, uint16_t value);
void Flash_Read_ARM_Median_Value(uint8_t id, uint16_t* value);

void Flash_Set_AKM_Angle(uint16_t angle);

void Flash_TestMode_Init(void);
void Flash_Set_TestMode(uint8_t mode);

#endif /* __APP_FLASH_H__ */
