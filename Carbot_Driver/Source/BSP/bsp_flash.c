#include "bsp_flash.h"
#include "bsp.h"
#include "bsp_usart.h"

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)


uint16_t Flash_BUF[STM_SECTOR_SIZE / 2]; //最多是2K字节


//解锁STM32的FLASH
static void Flash_Unlock(void)
{
	FLASH->KEYR = FLASH_KEY1; //写入解锁序列.
	FLASH->KEYR = FLASH_KEY2;
}

//flash上锁
static void Flash_Lock(void)
{
	FLASH->CR |= 1 << 7; //上锁
}

//得到FLASH状态
static uint8_t Flash_GetStatus(void)
{
	uint32_t res;
	res = FLASH->SR;
	if (res & (1 << 0))
		return 1; //忙
	else if (res & (1 << 2))
		return 2; //编程错误
	else if (res & (1 << 4))
		return 3; //写保护错误
	return 0;	  //操作完成
}

//等待操作完成
//time:要延时的长短
//返回值:状态.
static uint8_t Flash_WaitDone(uint16_t time)
{
	uint8_t res, i;
	do
	{
		res = Flash_GetStatus();
		if (res != 1)
			break; //非忙,无需等待了,直接退出.
		for (i = 0; i < 10; i++); // 增加延迟
		time--;
	} while (time);
	if (time == 0)
		res = 0xff; //TIMEOUT
	return res;
}


//在FLASH指定地址写入半字
//faddr:指定地址(此地址必须为2的倍数!!)
//dat:要写入的数据
//返回值:写入的情况
static uint8_t Flash_WriteHalfWord(uint32_t faddr, uint16_t dat)
{
	uint8_t res;
	res = Flash_WaitDone(0XFF);
	if (res == 0) //OK
	{
		FLASH->CR |= 1 << 0;		   //编程使能
		*(vu16 *)faddr = dat;		   //写入数据
		res = Flash_WaitDone(0XFF); //等待操作完成
		if (res != 1)				   //操作成功
		{
			FLASH->CR &= ~(1 << 0); //清除PG位.
		}
	}
	return res;
}

//读取指定地址的半字(16位数据)
//faddr:读地址
//返回值:对应数据.
static uint16_t Flash_ReadHalfWord(uint32_t faddr)
{
	return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN //如果使能了写

//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
static void Flash_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint16_t i;
	for (i = 0; i < NumToWrite; i++)
	{
		Flash_WriteHalfWord(WriteAddr, pBuffer[i]);
		WriteAddr += 2; //地址增加2.
	}
}

//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint32_t secpos;	   //扇区地址
	uint16_t secoff;	   //扇区内偏移地址(16位字计算)
	uint16_t secremain; //扇区内剩余地址(16位字计算)
	uint16_t i;
	uint32_t offaddr; //去掉0X08000000后的地址
	if (WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
		return;								  //非法地址
	Flash_Unlock();						  //解锁
	offaddr = WriteAddr - STM32_FLASH_BASE;	  //实际偏移地址.
	secpos = offaddr / STM_SECTOR_SIZE;		  //扇区地址  0~127 for STM32F103RCT6
	secoff = (offaddr % STM_SECTOR_SIZE) / 2; //在扇区内的偏移(2个字节为基本单位.)
	secremain = STM_SECTOR_SIZE / 2 - secoff; //扇区剩余空间大小
	if (NumToWrite <= secremain)
		secremain = NumToWrite; //不大于该扇区范围
	while (1)
	{
		Flash_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, Flash_BUF, STM_SECTOR_SIZE / 2); //读出整个扇区的内容
		for (i = 0; i < secremain; i++)															 //校验数据
		{
			if (Flash_BUF[secoff + i] != 0XFFFF)
				break; //需要擦除
		}
		if (i < secremain) //需要擦除
		{
			Flash_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE); //擦除这个扇区
			for (i = 0; i < secremain; i++)								  //复制
			{
				Flash_BUF[i + secoff] = pBuffer[i];
			}
			Flash_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, Flash_BUF, STM_SECTOR_SIZE / 2); //写入整个扇区
		}
		else
			Flash_Write_NoCheck(WriteAddr, pBuffer, secremain); //写已经擦除了的,直接写入扇区剩余区间.
		if (NumToWrite == secremain)
			break; //写入结束了
		else	   //写入未结束
		{
			secpos++;					//扇区地址增1
			secoff = 0;					//偏移位置为0
			pBuffer += secremain;		//指针偏移
			WriteAddr += secremain * 2; //写地址偏移(16位数据地址,需要*2)
			NumToWrite -= secremain;	//字节(16位)数递减
			if (NumToWrite > (STM_SECTOR_SIZE / 2))
				secremain = STM_SECTOR_SIZE / 2; //下一个扇区还是写不完
			else
				secremain = NumToWrite; //下一个扇区可以写完了
		}
	};
	Flash_Lock(); //上锁
}
#endif

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
	uint16_t i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = Flash_ReadHalfWord(ReadAddr);   //读取2个字节.
		ReadAddr += 2;								 //偏移2个字节.
	}
}


//擦除页
//paddr:页地址
//返回值:执行情况
uint8_t Flash_ErasePage(uint32_t paddr)
{
	uint8_t res = 0;
	res = Flash_WaitDone(0X5FFF); //等待上次操作结束,>20ms
	if (res == 0)
	{
		FLASH->CR |= 1 << 1;			 //页擦除
		FLASH->AR = paddr;				 //设置页地址
		FLASH->CR |= 1 << 6;			 //开始擦除
		res = Flash_WaitDone(0X5FFF); //等待操作结束,>20ms
		if (res != 1)					 //非忙
		{
			FLASH->CR &= ~(1 << 1); //清除页擦除标志.
		}
	}
	return res;
}

