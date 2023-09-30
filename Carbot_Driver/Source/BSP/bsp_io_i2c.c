#include "bsp_io_i2c.h"
#include "bsp.h"


//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)3<<12;}

//IO操作函数	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //输入SDA 

// 等待引脚的时间，可根据芯片时钟修改，只要符合通讯要求即可。
#define DELAY_FOR_COUNT      10


static void Delay_For_Pin(vu8 nCount) 
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < DELAY_FOR_COUNT; i++); 
    }
}


/**
 * @Brief: 初始化I2C对应的接口引脚
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Init(void)
{
    RCC->APB2ENR |= 1 << 3;   //先使能外设IO PORTB时钟
    GPIOB->CRH &= 0XFFFF00FF; //PB 10/11 推挽输出
    GPIOB->CRH |= 0X00003300;
}

/**
 * @Brief: 产生IIC起始信号
 * @Note:
 * @Parm: void
 * @Retval: void
 */
int IIC_Start(void)
{
    SDA_OUT(); //sda线输出
    IIC_SDA = 1;
    if (!READ_SDA)
        return 0;
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    if (READ_SDA)
        return 0;
    Delay_For_Pin(2);
    IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
    return 1;
}

/**
 * @Brief: 产生IIC停止信号
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Stop(void)
{
    SDA_OUT(); //sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    Delay_For_Pin(2);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 1; //发送I2C总线结束信号
    Delay_For_Pin(2);
}

/**
 * @Brief: 等待应答信号到来
 * @Note:
 * @Parm:
 * @Retval: 1:接收应答成功 | 0:接收应答失败
 */
int IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); //SDA设置为输入
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 0;
        }
        Delay_For_Pin(1);
    }
    IIC_SCL = 0; //时钟输出0
    return 1;
}

/**
 * @Brief: 产生ACK应答
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: 产生NACK应答
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: IIC发送一个字节
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL = 0; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        Delay_For_Pin(1);
        IIC_SCL = 1;
        Delay_For_Pin(1);
        IIC_SCL = 0;
        Delay_For_Pin(1);
    }
}

uint8_t g_addr[128] = {0};


void i2c_scanf_addr(void)
{
    uint8_t i2c_count = 0;
    for (int i = 1; i < 128; i++)
    {
        if (!IIC_Start())
        {
            printf("External IIC Start Error:%d\n", i);
            return;
        }
        IIC_Send_Byte(i << 1);
        if (!IIC_Wait_Ack())
        {
            if (i == 127)
            {
                printf("External IIC Scanf End, Count=%d\n", i2c_count);
            }
            continue;
        }
        IIC_Stop();
        i2c_count++;
        printf("External IIC Found address:0x%2x\n", i);
        g_addr[i] = i;
    }
}


/**
 * @Brief: I2C写数据函数
 * @Note:
 * @Parm: addr:I2C地址 | reg:寄存器 | len:数据长度 | data:数据指针
 * @Retval: 0:停止 | 1:连续写
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack())
        {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: I2C读函数
 * @Note:
 * @Parm: 参数同写类似
 * @Retval:
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1) + 1);
    IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
 * @Note:
 * @Parm:
 * @Retval:
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        Delay_For_Pin(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        Delay_For_Pin(2);
    }
    if (ack)
        IIC_Ack(); //发送ACK
    else
        IIC_NAck(); //发送nACK
    return receive;
}

/**
 * @Brief: 读取指定设备 指定寄存器的一个值
 * @Note: 
 * @Parm: I2C_Addr  目标设备地址 | addr     寄存器地址
 * @Retval: 
 */
//该函数不可用,有问题待查!!!!
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;

    IIC_Start();
    IIC_Send_Byte(I2C_Addr); //发送写命令
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    res++; //发送地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(I2C_Addr + 1);
    res++; //进入接收模式
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop(); //产生一个停止条件

    return res;
}

/**
 * @Brief: 读取指定设备 指定寄存器的 length个值
 * @Note: 
 * @Parm: dev  目标设备地址 | reg   寄存器地址 | length 要读的字节数 | *data  读出的数据将要存放的指针
 * @Retval: 读出来的字节数量
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;

    IIC_Start();
    IIC_Send_Byte(dev); //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //发送地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev + 1); //进入接收模式
    IIC_Wait_Ack();

    for (count = 0; count < length; count++)
    {

        if (count != length - 1)
            data[count] = IIC_Read_Byte(1); //带ACK的读数据
        else
            data[count] = IIC_Read_Byte(0); //最后一个字节NACK
    }
    IIC_Stop(); //产生一个停止条件
    return count;
}

/**
 * @Brief: 将多个字节写入指定设备 指定寄存器
 * @Note: 
 * @Parm: dev  目标设备地址 | reg   寄存器地址 | length 要写的字节数 | *data  将要写的数据的首地址
 * @Retval: 返回是否成功
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{

    uint8_t count = 0;
    IIC_Start();
    IIC_Send_Byte(dev); //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //发送地址
    IIC_Wait_Ack();
    for (count = 0; count < length; count++)
    {
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop(); //产生一个停止条件

    return 1; //status == 0;
}

/**
 * @Brief: 读取指定设备 指定寄存器的一个值
 * @Note: 
 * @Parm: dev  目标设备地址 | reg    寄存器地址 | *data  读出的数据将要存放的地址
 * @Retval: 1
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    *data = I2C_ReadOneByte(dev, reg);
    return 1;
}

/**
 * @Brief: 写入指定设备 指定寄存器一个字节
 * @Note: 
 * @Parm: dev  目标设备地址 | reg    寄存器地址 | data  将要写入的字节
 * @Retval: 1
 */
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**
 * @Brief: 读 修改 写 指定设备 指定寄存器一个字节 中的多个位
 * @Note: 
 * @Parm: dev  目标设备地址 | reg    寄存器地址 | bitStart  目标字节的起始位 | length   位长度 | data    存放改变目标字节位的值
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0)
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}

/**
 * @Brief: 读 修改 写 指定设备 指定寄存器一个字节 中的1个位
 * @Note: 
 * @Parm: dev  目标设备地址 | reg    寄存器地址 | bitNum  要修改目标字节的bitNum位 | data  为0 时，目标位将被清0 否则将被置位
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}
