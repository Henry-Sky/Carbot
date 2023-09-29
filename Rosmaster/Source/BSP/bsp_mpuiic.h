#ifndef __MPUMPU_IIC_H
#define __MPUMPU_IIC_H
#include "bsp.h"

#define ENABLE_I2C_PB10_PB11      0

#if ENABLE_I2C_PB10_PB11
#define MPU_SDA_IN()  	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO操作函数
#define MPU_IIC_SCL    PBout(10) //SCL
#define MPU_IIC_SDA    PBout(11) //SDA	 
#define MPU_READ_SDA   PBin(11)  //输入SDA 

#else
///*PB13 PB15 OK*/
//IO方向设置
#define MPU_SDA_IN()  	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)8<<28;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)3<<28;}

//IO操作函数	 
#define MPU_IIC_SCL    PBout(13) //SCL
#define MPU_IIC_SDA    PBout(15) //SDA	 
#define MPU_READ_SDA   PBin(15)  //输入SDA 

#endif

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

uint8_t MPU_Write_Byte(uint8_t devaddr,uint8_t reg,uint8_t data);
uint8_t MPU_Read_Byte(uint8_t devaddr,uint8_t reg);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

void MPU_Scanf_Addr(void);

#endif
















