#ifndef __BSP_IO_I2C_H__
#define __BSP_IO_I2C_H__

#include "stm32f10x.h"
extern uint8_t g_addr[128];

//IIC所有操作函数
void IIC_Init(void);                  //初始化IIC的IO口				 
int IIC_Start(void);                  //发送IIC开始信号
void IIC_Stop(void);                  //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);           //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);  //IIC读取一个字节
int IIC_Wait_Ack(void);               //IIC等待ACK信号
void IIC_Ack(void);                   //IIC发送ACK信号
void IIC_NAck(void);                  //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

void i2c_scanf_addr(void);

#endif
