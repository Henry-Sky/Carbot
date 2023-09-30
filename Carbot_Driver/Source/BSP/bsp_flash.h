#ifndef __Flash_H__
#define __Flash_H__
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE      256      //所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN      1        //使能FLASH写入(0，不使能;1，使能)

// 因为大容量每个扇区定义为2K，而小容量和中容量都定义为1K
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 //字节
#else
#define STM_SECTOR_SIZE 2048
#endif


//FLASH起始地址
#define STM32_FLASH_BASE      0x08000000      //STM32 FLASH的起始地址
#define STM32_FLASH_END       0x0803FFFF      //STM32F1RCT6 FLASH的结束地址


//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t Flash_ErasePage(uint32_t paddr);                                    //擦除页

void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);    //从指定地址开始写入指定长度的数据
void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);       //从指定地址开始读出指定长度的数据

#endif
